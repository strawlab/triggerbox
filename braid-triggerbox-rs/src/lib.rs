// SPDX-License-Identifier: MIT OR Apache-2.0

//! Host-side driver for the Braid camera synchronization triggerbox.

mod datetime_conversion;

mod arduino_udev;
use crate::arduino_udev::serial_handshake;

use anyhow::{Context, Result};
use chrono::Duration;
use log::{debug, error, info, trace, warn};

use nalgebra as na;

use std::collections::BTreeMap;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    sync::mpsc::{Receiver, Sender},
};

use braid_triggerbox_comms::{DEVICE_FIRMWARE_VERSION, Prescaler, TopAndPrescaler};

// ----- name type handling

/// Length of the device name (in bytes).
pub const DEVICE_NAME_LEN: usize = 8;

/// A device name.
pub type InnerNameType = [u8; DEVICE_NAME_LEN];
/// An optional device name.
pub type NameType = Option<InnerNameType>;

/// Callback called with each new clock model, or `None` when the model is
/// reset.
pub type ClockModelCallback = Box<dyn FnMut(Option<ClockModel>) + Send>;

/// Convert a string to a device name.
///
/// # Errors
///
/// Returns an error if `x` is longer than [`DEVICE_NAME_LEN`] bytes.
pub fn to_name_type(x: &str) -> Result<InnerNameType> {
    let bytes = x.as_bytes();
    if bytes.len() > DEVICE_NAME_LEN {
        anyhow::bail!("Maximum name length ({DEVICE_NAME_LEN} chars) exceeded.");
    }
    let mut name = [0; DEVICE_NAME_LEN];
    for (dest, src) in name.iter_mut().zip(bytes) {
        *dest = *src;
    }
    Ok(name)
}

/// Format a device name for display.
#[must_use]
pub fn name_display(name: &NameType) -> String {
    if let Some(name) = name {
        format!("\"{}\"", String::from_utf8_lossy(name))
    } else {
        "none".into()
    }
}

// ------ clock model types

/// A linear model relating triggerbox pulse number to host time.
#[derive(Debug, PartialEq, Clone)]
pub struct ClockModel {
    /// Seconds per pulse.
    pub gain: f64,
    /// Host time (in seconds since the UNIX epoch) of pulse zero.
    pub offset: f64,
    /// Sum of squared residuals of the fit.
    pub residuals: f64,
    /// Number of measurements used for the fit.
    pub n_measurements: u64,
}

/// A raw clock measurement.
#[derive(Debug)]
pub struct TriggerClockInfoRow {
    // changes to this should update BraidMetadataSchemaTag
    /// Host time at which the measurement was requested.
    pub start_timestamp: chrono::DateTime<chrono::Utc>,
    /// Triggerbox pulse number.
    pub framecount: i64,
    /// Fraction of the current pulse elapsed, scaled to 0-255.
    pub tcnt: u8,
    /// Host time at which the measurement was received.
    pub stop_timestamp: chrono::DateTime<chrono::Utc>,
}

/// A Braid Triggerbox device.
pub struct TriggerboxDevice {
    icr1_and_prescaler: Option<TopAndPrescaler>,
    version_check_done: bool,
    qi: u8,
    queries: BTreeMap<u8, chrono::DateTime<chrono::Utc>>,
    ser: tokio_serial::SerialStream,
    outq: Receiver<Cmd>,
    vquery_time: chrono::DateTime<chrono::Utc>,
    last_time: chrono::DateTime<chrono::Utc>,
    past_data: Vec<(f64, f64)>,
    allow_requesting_clock_sync: bool,
    on_new_model_cb: ClockModelCallback,
    triggerbox_data_tx: Option<Sender<TriggerClockInfoRow>>,
    max_acceptable_measurement_error: Duration,
}

impl std::fmt::Debug for TriggerboxDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TriggerboxDevice")
            .field("icr1_and_prescaler", &self.icr1_and_prescaler)
            .field("version_check_done", &self.version_check_done)
            .field("ser", &self.ser)
            .finish_non_exhaustive()
    }
}

/// A command for the triggerbox.
#[derive(Debug, Clone)]
pub enum Cmd {
    /// Set the trigger rate.
    TopAndPrescaler(TopAndPrescaler),
    /// Stop the trigger pulses and reset the pulse number.
    StopPulsesAndReset,
    /// Start the trigger pulses.
    StartPulses,
    /// Store a device name on the device.
    SetDeviceName(InnerNameType),
    /// Set the analog outputs (in volts).
    SetAOut((f64, f64)),
}

/// A sample returned by the device.
#[derive(Debug, PartialEq, Eq)]
struct TimedSample {
    value: u8,
    pulsenumber: u32,
    count: u16,
}

/// A packet sent by the device.
#[derive(Debug, PartialEq, Eq)]
enum DevicePacket {
    Timestamp(TimedSample),
    Version(TimedSample),
    Unknown(u8),
}

/// Remove the first complete packet from `buf` and return it.
///
/// Returns `Ok(None)` if `buf` does not yet hold a complete packet.
fn take_packet(buf: &mut Vec<u8>) -> Result<Option<DevicePacket>> {
    // A packet is a header (type and payload length), the payload and a
    // checksum.
    let [packet_type, payload_len, rest @ ..] = buf.as_slice() else {
        return Ok(None);
    };
    let packet_type = *packet_type;
    let payload_len = usize::from(*payload_len);
    let Some((payload, [expected_chksum, ..])) = rest.split_at_checked(payload_len) else {
        return Ok(None);
    };

    let actual_chksum = payload.iter().fold(0u8, |acc, x| acc.wrapping_add(*x));
    if actual_chksum != *expected_chksum {
        anyhow::bail!("checksum mismatch");
    }
    trace!("checksum OK");

    let packet = match (packet_type, payload) {
        (b'P' | b'V', &[value, p0, p1, p2, p3, c0, c1]) => {
            let sample = TimedSample {
                value,
                pulsenumber: u32::from_le_bytes([p0, p1, p2, p3]),
                count: u16::from_le_bytes([c0, c1]),
            };
            if packet_type == b'P' {
                DevicePacket::Timestamp(sample)
            } else {
                DevicePacket::Version(sample)
            }
        }
        (b'P' | b'V', _) => anyhow::bail!(
            "unexpected payload length {payload_len} for packet type '{}'",
            char::from(packet_type)
        ),
        _ => DevicePacket::Unknown(packet_type),
    };

    // header (2) + payload + checksum (1)
    let n_used = payload_len.saturating_add(3);
    buf.drain(..n_used);
    Ok(Some(packet))
}

/// Truncate `x` towards zero to a `u16`, or `None` if out of range.
fn f64_to_u16_checked(x: f64) -> Option<u16> {
    if (0.0..65536.0).contains(&x) {
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "range checked above, truncation is intended"
        )]
        Some(x as u16)
    } else {
        None
    }
}

/// Truncate `x` towards zero to a `u8`, or `None` if out of range.
fn f64_to_u8_checked(x: f64) -> Option<u8> {
    f64_to_u16_checked(x).and_then(|x| u8::try_from(x).ok())
}

/// Convert a voltage to a 12-bit DAC value, clamping to the DAC range.
fn volts_to_dac(volts: f64) -> u16 {
    // Convert voltage to fraction and clamp.
    let frac = (volts / 4.096).clamp(0.0, 1.0);
    // Compute integer DAC value. (A NaN voltage gives zero.)
    f64_to_u16_checked((frac * 4095.0).round()).unwrap_or(0)
}

impl TriggerboxDevice {
    /// Connect to the triggerbox at `device_path`.
    ///
    /// # Errors
    ///
    /// Returns an error if the device cannot be opened, does not respond or
    /// does not have the name `assert_device_name` (if given).
    pub async fn new(
        on_new_model_cb: ClockModelCallback,
        device_path: String,
        outq: Receiver<Cmd>,
        triggerbox_data_tx: Option<Sender<TriggerClockInfoRow>>,
        assert_device_name: NameType,
        max_acceptable_measurement_error: std::time::Duration,
        sleep_dur: std::time::Duration,
    ) -> Result<Self> {
        let baud_rate = 115_200;
        let max_acceptable_measurement_error = Duration::from_std(max_acceptable_measurement_error)
            .context("max_acceptable_measurement_error out of range")?;
        let now = chrono::Utc::now();

        // wait 1 second before first version query
        let vquery_time = add_duration(now, Duration::seconds(1))?;
        let last_time = add_duration(vquery_time, Duration::seconds(1))?;

        debug!("Opening device at path {device_path}");

        let (ser, name) = match tokio::time::timeout(
            std::time::Duration::from_secs(15),
            serial_handshake(&device_path, baud_rate, sleep_dur),
        )
        .await
        {
            Ok(r) => r,
            Err(elapsed) => Err(anyhow::Error::from(elapsed)),
        }
        .with_context(|| format!("opening device {device_path}"))?;

        if let Some(name) = &name {
            let name_str = String::from_utf8_lossy(name);
            debug!("Connected to device named \"{name_str}\".");
        } else {
            debug!("Connected to unnamed device.");
        }

        if assert_device_name.is_some() && name != assert_device_name {
            anyhow::bail!(
                "Found name {}, but expected {}. ({:?} vs {:?}.)",
                name_display(&name),
                name_display(&assert_device_name),
                name,
                assert_device_name,
            );
        }

        Ok(Self {
            icr1_and_prescaler: None,
            version_check_done: false,
            qi: 0,
            queries: BTreeMap::new(),
            ser,
            outq,
            vquery_time,
            last_time,
            past_data: Vec::new(),
            allow_requesting_clock_sync: false,
            on_new_model_cb,
            triggerbox_data_tx,
            max_acceptable_measurement_error,
        })
    }

    async fn write(&mut self, buf: &[u8]) -> tokio::io::Result<()> {
        trace!("sending: \"{}\"", String::from_utf8_lossy(buf));
        for byte in buf {
            trace!("sending byte: {byte}");
        }
        AsyncWriteExt::write_all(&mut self.ser, buf).await?;
        Ok(())
    }

    async fn handle_host_command(&mut self, cmd: Cmd) -> Result<()> {
        debug!("got command {cmd:?}");
        match cmd {
            Cmd::TopAndPrescaler(new_value) => {
                self.set_top_and_prescaler(new_value).await?;
            }
            Cmd::StopPulsesAndReset => {
                debug!("will reset counters. dropping outstanding info requests.");
                self.allow_requesting_clock_sync = false;
                self.queries.clear();
                self.past_data.clear();
                (self.on_new_model_cb)(None);
                self.write(b"S0").await?;
            }
            Cmd::StartPulses => {
                self.allow_requesting_clock_sync = true;
                self.write(b"S1").await?;
            }
            Cmd::SetDeviceName(name) => {
                let computed_crc = format!("{:X}", arduino_udev::CRC_MAXIM.checksum(&name));
                trace!("computed CRC: {computed_crc:?}");

                self.write(b"N=").await?;
                self.write(&name).await?;
                self.write(computed_crc.as_bytes()).await?;
            }
            Cmd::SetAOut((volts1, volts2)) => {
                let val1 = volts_to_dac(volts1);
                let val2 = volts_to_dac(volts2);

                self.write(b"O=").await?;
                self.write(&val1.to_le_bytes()).await?;
                self.write(&val2.to_le_bytes()).await?;
                self.write(b"x").await?;

                // Now wait for return value.
                tokio::time::sleep(std::time::Duration::from_millis(50)).await;

                let mut buf = vec![0; 100];
                let len = self.ser.read(&mut buf).await?;
                buf.truncate(len);
                debug!("AOUT ignoring values: {buf:?}");
            }
        }
        Ok(())
    }

    /// Run forever, handling interaction with the triggerbox hardware device.
    ///
    /// Drop all instances of the `Sender<Cmd>` which could send messages to the
    /// `Receiver<Cmd>` passed to [`Self::new`] to exit.
    ///
    /// # Errors
    ///
    /// Returns an error if communication with the device fails.
    pub async fn run_forever(
        mut self: TriggerboxDevice,
        query_dt: std::time::Duration,
    ) -> Result<()> {
        let query_dt = Duration::from_std(query_dt)?;

        let connect_time = chrono::Utc::now();

        let mut buf: Vec<u8> = Vec::new();
        let mut read_buf: Vec<u8> = vec![0; 100];
        let mut version_check_started = false;
        let mut new_data = false;
        let mut interval = tokio::time::interval(std::time::Duration::from_millis(100));

        loop {
            if self.version_check_done {
                tokio::select! {
                    // Handle command queue iff version check done.
                    opt_cmd_tup = self.outq.recv() => {
                        if let Some(cmd) = opt_cmd_tup {
                            self.handle_host_command(cmd).await?;
                        } else {
                            // no more commands, sender hung up
                            info!("exiting run loop");
                            return Ok(());
                        }
                    },
                    res_r = self.ser.read(&mut read_buf) => {
                        let n_bytes_read = res_r?;
                        if n_bytes_read > 0 {
                            update_read_buffer(n_bytes_read, &read_buf, &mut buf);
                            new_data = true;
                        }
                    },
                    _ = interval.tick() => {}
                }
            } else {
                // Same as above except `self.outq` is not checked. This is done
                // at startup before the version number is confirmed.
                tokio::select! {
                    res_r = self.ser.read(&mut read_buf) => {
                        let n_bytes_read = res_r?;
                        if n_bytes_read > 0 {
                            update_read_buffer(n_bytes_read, &read_buf, &mut buf);
                            new_data = true;
                        }
                    },
                    _ = interval.tick() => {}
                }
            }

            // handle pending data, if any
            if new_data {
                self.handle_data_from_device(&mut buf).await?;
                new_data = false;
            }

            let now = chrono::Utc::now();

            if self.version_check_done {
                if self.allow_requesting_clock_sync
                    && (now.signed_duration_since(self.last_time) > query_dt)
                {
                    // request sample
                    debug!("making clock sample request. qi: {}, now: {now}", self.qi);
                    self.queries.insert(self.qi, now);
                    let send_buf = [b'P', self.qi];
                    self.write(&send_buf).await?;
                    self.qi = self.qi.wrapping_add(1);
                    self.last_time = now;
                }
            } else {
                // request firmware version
                if !version_check_started && now >= self.vquery_time {
                    info!("checking firmware version");
                    self.write(b"V?").await?;
                    version_check_started = true;
                    self.vquery_time = now;
                }

                // retry every second
                if now.signed_duration_since(self.vquery_time) > Duration::seconds(1) {
                    version_check_started = false;
                }
                // give up after 20 seconds
                if now.signed_duration_since(connect_time) > Duration::seconds(20) {
                    return Err(anyhow::anyhow!("no version response"));
                }
            }
        }
    }

    async fn set_top_and_prescaler(&mut self, new_value: TopAndPrescaler) -> Result<()> {
        let [icr1_lo, icr1_hi] = new_value.avr_icr1().to_le_bytes();
        let buf = [icr1_lo, icr1_hi, new_value.prescaler_key()];

        self.icr1_and_prescaler = Some(new_value);

        self.write(b"T=").await?;
        self.write(&buf).await?;
        Ok(())
    }

    async fn handle_returned_timestamp(&mut self, sample: TimedSample) -> Result<()> {
        let TimedSample {
            value: qi,
            pulsenumber,
            count,
        } = sample;
        debug!("got returned timestamp with qi: {qi}, pulsenumber: {pulsenumber}, count: {count}");
        let now = chrono::Utc::now();
        if self.queries.len() > 50 {
            self.queries.clear();
            error!("too many outstanding queries");
        }

        let Some(send_timestamp) = self.queries.remove(&qi) else {
            warn!("could not find original data for query {qi:?}");
            return Ok(());
        };
        trace!("this query has send_timestamp: {send_timestamp}");

        let max_error = now.signed_duration_since(send_timestamp);
        if max_error > self.max_acceptable_measurement_error {
            debug!("clock sample took {max_error:?}. Ignoring value.");
            return Ok(());
        }

        trace!("max_error: {max_error:?}");

        let half_max_error = Duration::microseconds(
            max_error
                .num_microseconds()
                .context("measurement error out of range")?
                / 2,
        );
        let ino_time_estimate = add_duration(send_timestamp, half_max_error)?;

        let Some(s) = &self.icr1_and_prescaler else {
            warn!("No clock measurements until framerate set.");
            return Ok(());
        };

        let frac = f64::from(count) / f64::from(s.avr_icr1());
        if !(0.0..=1.0).contains(&frac) {
            warn!("ignoring clock sample with invalid count {count}");
            return Ok(());
        }
        let ino_stamp = f64::from(pulsenumber) + frac;

        if let Some(tbox_tx) = &self.triggerbox_data_tx {
            // send our newly acquired data to be saved to disk
            let to_save = TriggerClockInfoRow {
                start_timestamp: send_timestamp,
                framecount: i64::from(pulsenumber),
                tcnt: f64_to_u8_checked(frac * 255.0).unwrap_or(u8::MAX),
                stop_timestamp: now,
            };
            if let Err(e) = tbox_tx.send(to_save).await {
                warn!("ignoring {e}");
            }
        }

        // delete old data
        while self.past_data.len() >= 100 {
            self.past_data.remove(0);
        }

        self.past_data.push((
            ino_stamp,
            datetime_conversion::datetime_to_f64(&ino_time_estimate),
        ));

        if self.past_data.len() >= 5 {
            let (gain, offset, residuals) =
                fit_time_model(&self.past_data).map_err(|e| anyhow::anyhow!("lstsq err: {e}"))?;

            let n_measurements = u32::try_from(self.past_data.len())?;
            let per_point_residual = residuals / f64::from(n_measurements);
            // TODO only accept this if residuals less than some amount?
            debug!(
                "new: ClockModel{{gain: {gain}, offset: {offset}}}, per_point_residual: {per_point_residual}"
            );
            (self.on_new_model_cb)(Some(ClockModel {
                gain,
                offset,
                residuals,
                n_measurements: u64::from(n_measurements),
            }));
        }
        Ok(())
    }

    fn handle_version(&mut self, sample: &TimedSample) -> Result<()> {
        let value = sample.value;
        trace!("got returned version with value: {value}");
        if value != DEVICE_FIRMWARE_VERSION {
            anyhow::bail!(
                "triggerbox firmware version {value} is not supported (expected version \
                {DEVICE_FIRMWARE_VERSION})"
            );
        }
        self.vquery_time = chrono::Utc::now();
        self.version_check_done = true;
        info!("connected to triggerbox firmware version {value}");
        Ok(())
    }

    async fn handle_data_from_device(&mut self, buf: &mut Vec<u8>) -> Result<()> {
        while let Some(packet) = take_packet(buf)? {
            match packet {
                DevicePacket::Timestamp(sample) => self.handle_returned_timestamp(sample).await?,
                DevicePacket::Version(sample) => self.handle_version(&sample)?,
                DevicePacket::Unknown(packet_type) => {
                    warn!("ignoring unknown packet type {packet_type}");
                }
            }
        }
        Ok(())
    }
}

fn update_read_buffer(n_bytes_read: usize, read_buf: &[u8], buf: &mut Vec<u8>) {
    for &byte in read_buf.iter().take(n_bytes_read) {
        trace!("read byte {byte} (char {})", char::from(byte));
        buf.push(byte);
    }
}

fn add_duration(
    t: chrono::DateTime<chrono::Utc>,
    d: Duration,
) -> Result<chrono::DateTime<chrono::Utc>> {
    t.checked_add_signed(d).context("time out of range")
}

fn fit_time_model(past_data: &[(f64, f64)]) -> Result<(f64, f64, f64), &'static str> {
    use na::{OMatrix, OVector, U2};

    let mut a: Vec<f64> = Vec::with_capacity(past_data.len().saturating_mul(2));
    let mut b: Vec<f64> = Vec::with_capacity(past_data.len());

    for row in past_data {
        a.push(row.0);
        a.push(1.0);
        b.push(row.1);
    }
    let a = OMatrix::<f64, na::Dyn, U2>::from_row_slice(&a);
    let b = OVector::<f64, na::Dyn>::from_row_slice(&b);

    let epsilon = 1e-10;
    let results = lstsq::lstsq(&a, &b, epsilon)?;

    let gain = results.solution.x;
    let offset = results.solution.y;
    let residuals = results.residuals;

    Ok((gain, offset, residuals))
}

/// Options for [`run_triggerbox`].
#[derive(Clone, Debug)]
pub struct TriggerboxOptions {
    /// Path of the serial device.
    pub device_path: String,
    /// Interval between clock measurements.
    pub query_dt: std::time::Duration,
    /// If given, the required device name.
    pub assert_device_name: NameType,
    /// Clock measurements which take longer than this are discarded.
    pub max_acceptable_measurement_error: std::time::Duration,
    /// Time to wait for the device to reset after opening it.
    pub sleep_dur: std::time::Duration,
}

/// Connect to a triggerbox and run until `outq` is closed.
///
/// # Errors
///
/// Returns an error if connecting to or communicating with the device fails.
pub async fn run_triggerbox(
    on_new_model_cb: ClockModelCallback,
    outq: Receiver<Cmd>,
    triggerbox_data_tx: Option<Sender<TriggerClockInfoRow>>,
    opts: TriggerboxOptions,
) -> Result<()> {
    let TriggerboxOptions {
        device_path,
        query_dt,
        assert_device_name,
        max_acceptable_measurement_error,
        sleep_dur,
    } = opts;

    let triggerbox = TriggerboxDevice::new(
        on_new_model_cb,
        device_path,
        outq,
        triggerbox_data_tx,
        assert_device_name,
        max_acceptable_measurement_error,
        sleep_dur,
    )
    .await?;
    triggerbox.run_forever(query_dt).await
}

fn get_rate(rate_ideal: f64, prescaler: Prescaler) -> (u16, f64) {
    let xtal = 16e6; // 16 MHz clock
    let base_clock = xtal / prescaler.as_f64();
    let new_top_ideal = base_clock / rate_ideal;
    let new_icr1_f64 = new_top_ideal.round().clamp(0.0, f64::from(u16::MAX));
    // A NaN rate gives zero.
    let new_icr1 = f64_to_u16_checked(new_icr1_f64).unwrap_or(0);
    let rate_actual = base_clock / f64::from(new_icr1);
    (new_icr1, rate_actual)
}

/// Given an ideal frame rate (in frames per second), compute the triggerbox
/// command which best approximates this frame rate.
///
/// Returns the triggerbox command and the expected actual frame rate (in frames
/// per second).
#[must_use]
pub fn make_trig_fps_cmd(rate_ideal: f64) -> (Cmd, f64) {
    let (top_8, rate_actual_8) = get_rate(rate_ideal, Prescaler::Scale8);
    let (top_64, rate_actual_64) = get_rate(rate_ideal, Prescaler::Scale64);

    let error_8 = (rate_ideal - rate_actual_8).abs();
    let error_64 = (rate_ideal - rate_actual_64).abs();

    let (top, rate_actual, prescaler) = if error_8 < error_64 {
        (top_8, rate_actual_8, Prescaler::Scale8)
    } else {
        (top_64, rate_actual_64, Prescaler::Scale64)
    };

    (
        Cmd::TopAndPrescaler(TopAndPrescaler::new_avr(top, prescaler)),
        rate_actual,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(packet_type: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = vec![packet_type, u8::try_from(payload.len()).unwrap()];
        buf.extend_from_slice(payload);
        buf.push(payload.iter().fold(0u8, |acc, x| acc.wrapping_add(*x)));
        buf
    }

    #[test]
    fn test_fit_time_model() {
        let epsilon = 1e-12;

        let data = vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0)];
        let (gain, offset, _residuals) = fit_time_model(&data).unwrap();
        assert!((gain - 1.0).abs() < epsilon);
        assert!((offset - 0.0).abs() < epsilon);

        let data = vec![(0.0, 12.0), (1.0, 22.0), (2.0, 32.0), (3.0, 42.0)];
        let (gain, offset, _residuals) = fit_time_model(&data).unwrap();
        assert!((gain - 10.0).abs() < epsilon);
        assert!((offset - 12.0).abs() < epsilon);
    }

    #[test]
    fn take_packets() {
        let mut buf = packet(b'P', &[3, 1, 0, 0, 0, 2, 0]);
        buf.extend(packet(b'V', &[14, 0, 0, 0, 0, 0, 0]));
        buf.extend(packet(b'X', &[]));
        // A partial packet.
        buf.extend(&[b'P', 7, 0]);

        assert_eq!(
            take_packet(&mut buf).unwrap(),
            Some(DevicePacket::Timestamp(TimedSample {
                value: 3,
                pulsenumber: 1,
                count: 2,
            }))
        );
        assert_eq!(
            take_packet(&mut buf).unwrap(),
            Some(DevicePacket::Version(TimedSample {
                value: 14,
                pulsenumber: 0,
                count: 0,
            }))
        );
        assert_eq!(
            take_packet(&mut buf).unwrap(),
            Some(DevicePacket::Unknown(b'X'))
        );
        assert_eq!(take_packet(&mut buf).unwrap(), None);
        assert_eq!(buf, [b'P', 7, 0]);
    }

    #[test]
    fn take_packet_errors() {
        let mut buf = packet(b'P', &[1, 2, 3, 4, 5, 6, 7]);
        *buf.last_mut().unwrap() ^= 0xFF;
        assert!(take_packet(&mut buf).is_err());

        let mut buf = packet(b'V', &[1, 2, 3]);
        assert!(take_packet(&mut buf).is_err());
    }

    #[test]
    fn name_type() {
        assert_eq!(to_name_type("abc").unwrap(), *b"abc\0\0\0\0\0");
        assert_eq!(to_name_type("abcdefgh").unwrap(), *b"abcdefgh");
        assert!(to_name_type("abcdefghi").is_err());
    }

    #[test]
    fn dac_values() {
        assert_eq!(volts_to_dac(-1.0), 0);
        assert_eq!(volts_to_dac(0.0), 0);
        assert_eq!(volts_to_dac(4.096), 4095);
        assert_eq!(volts_to_dac(100.0), 4095);
        assert_eq!(volts_to_dac(f64::NAN), 0);
    }

    #[test]
    fn trig_fps_cmd() {
        let (_cmd, rate_actual) = make_trig_fps_cmd(100.0);
        assert!((rate_actual - 100.0).abs() < 1e-6);
        // Extreme requests are clamped rather than panicking.
        for rate in [0.0, 1e-6, 1e12, f64::NAN, f64::INFINITY] {
            let _ = make_trig_fps_cmd(rate);
        }
    }
}
