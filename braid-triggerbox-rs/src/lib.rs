#[macro_use]
extern crate log;

mod datetime_conversion;

mod arduino_udev;
use crate::arduino_udev::serial_handshake;

use anyhow::{Context, Result};
use chrono::Duration;

use nalgebra as na;

use std::collections::BTreeMap;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    sync::mpsc::{Receiver, Sender},
};

use braid_triggerbox_comms::{Prescaler, TopAndPrescaler, DEVICE_FIRMWARE_VERSION};

// ----- name type handling
pub const DEVICE_NAME_LEN: usize = 8;

pub type InnerNameType = [u8; DEVICE_NAME_LEN];
pub type NameType = Option<InnerNameType>;

pub type ClockModelCallback = Box<dyn FnMut(Option<ClockModel>) + Send>;

pub fn to_name_type(x: &str) -> anyhow::Result<InnerNameType> {
    let mut name = [0; DEVICE_NAME_LEN];
    let bytes = x.as_bytes();
    if bytes.len() > DEVICE_NAME_LEN {
        anyhow::bail!("Maximum name length ({} chars) exceeded.", DEVICE_NAME_LEN);
    }
    name[..bytes.len()].copy_from_slice(bytes);
    Ok(name)
}

pub fn name_display(name: &NameType) -> String {
    if let Some(name) = name {
        format!("\"{}\"", String::from_utf8_lossy(name))
    } else {
        "none".into()
    }
}

// ------ clock model types

#[derive(Debug, PartialEq, Clone)]
pub struct ClockModel {
    pub gain: f64,
    pub offset: f64,
    pub residuals: f64,
    pub n_measurements: u64,
}

#[derive(Debug)]
pub struct TriggerClockInfoRow {
    // changes to this should update BraidMetadataSchemaTag
    pub start_timestamp: chrono::DateTime<chrono::Utc>,
    pub framecount: i64,
    pub tcnt: u8,
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

#[derive(Debug, Clone)]
pub enum Cmd {
    TopAndPrescaler(TopAndPrescaler),
    StopPulsesAndReset,
    StartPulses,
    SetDeviceName(InnerNameType),
    SetAOut((f64, f64)),
}

impl TriggerboxDevice {
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
        let max_acceptable_measurement_error =
            Duration::from_std(max_acceptable_measurement_error).unwrap();
        let now = chrono::Utc::now();

        // wait 1 second before first version query
        let vquery_time = now + Duration::seconds(1);

        debug!("Opening device at path {}", device_path);

        let (ser, name) = match tokio::time::timeout(
            std::time::Duration::from_millis(15_000),
            serial_handshake(&device_path, baud_rate, sleep_dur),
        )
        .await
        {
            Ok(r) => r,
            Err(elapsed) => Err(elapsed).map_err(anyhow::Error::from),
        }
        .with_context(|| format!("opening device {}", device_path))?;

        if let Some(name) = &name {
            let name_str = String::from_utf8_lossy(name);
            debug!("Connected to device named \"{}\".", name_str);
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
            last_time: vquery_time + Duration::seconds(1),
            past_data: Vec::new(),
            allow_requesting_clock_sync: false,
            on_new_model_cb,
            triggerbox_data_tx,
            max_acceptable_measurement_error,
        })
    }

    async fn write(&mut self, buf: &[u8]) -> tokio::io::Result<()> {
        trace!("sending: \"{}\"", String::from_utf8_lossy(buf));
        for byte in buf.iter() {
            trace!("sending byte: {}", byte);
        }
        AsyncWriteExt::write_all(&mut self.ser, buf).await?;
        Ok(())
    }

    async fn handle_host_command(&mut self, cmd: Cmd) -> Result<()> {
        debug!("got command {:?}", cmd);
        match cmd {
            Cmd::TopAndPrescaler(new_value) => {
                self._set_top_and_prescaler(new_value).await?;
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
                trace!("computed CRC: {:?}", computed_crc);

                self.write(b"N=").await?;
                self.write(&name).await?;
                self.write(computed_crc.as_bytes()).await?;
            }
            Cmd::SetAOut((volts1, volts2)) => {
                fn volts_to_dac(volts: f64) -> u16 {
                    // Convert voltage to fraction and clamp.
                    let frac = (volts / 4.096).clamp(0.0, 1.0);
                    // Compute integer DAC value.
                    let val: u16 = (frac * 4095.0).round() as u16;
                    val
                }
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
                let buf = &buf[..len];
                debug!("AOUT ignoring values: {:?}", buf);
            }
        }
        Ok(())
    }

    /// Run forever, handling interaction with the triggerbox hardware device.
    ///
    /// Drop all instances of the `Sender<Cmd>` which could send messages to the
    /// `Receiver<Cmd>` passed to [Self::new] to exit.
    pub async fn run_forever(
        mut self: TriggerboxDevice,
        query_dt: std::time::Duration,
    ) -> Result<()> {
        let query_dt = Duration::from_std(query_dt)?;

        let mut now = chrono::Utc::now();

        let connect_time = now;

        let mut buf: Vec<u8> = Vec::new();
        let mut read_buf: Vec<u8> = vec![0; 100];
        let mut version_check_started = false;
        let mut new_data = false;
        let mut interval = tokio::time::interval(std::time::Duration::from_millis(100));

        fn update_read_buffer(n_bytes_read: usize, read_buf: &[u8], buf: &mut Vec<u8>) {
            for i in 0..n_bytes_read {
                let byte = read_buf[i];
                trace!(
                    "read byte {} (char {})",
                    byte,
                    String::from_utf8_lossy(&read_buf[i..i + 1])
                );
                buf.push(byte);
            }
        }

        loop {
            if self.version_check_done {
                tokio::select! {
                    // Handle command queue iff version check done.
                    opt_cmd_tup = self.outq.recv() => {
                        match opt_cmd_tup {
                            Some(cmd) => {
                                self.handle_host_command(cmd).await?;
                            }
                            None => {
                                // no more commands, sender hung up
                                info!("exiting run loop");
                                return Ok(());
                            }
                        }
                    },
                    res_r = self.ser.read(&mut read_buf) => {
                        let n_bytes_read = res_r?;
                        if n_bytes_read > 0 {
                            update_read_buffer(n_bytes_read,&read_buf,&mut buf);
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
                            update_read_buffer(n_bytes_read,&read_buf,&mut buf);
                            new_data = true;
                        }
                    },
                    _ = interval.tick() => {}
                }
            }

            // handle pending data, if any
            if new_data {
                buf = self.handle_data_from_device(buf).await?;
                new_data = false;
            }

            now = chrono::Utc::now();

            if self.version_check_done {
                if self.allow_requesting_clock_sync
                    & (now.signed_duration_since(self.last_time) > query_dt)
                {
                    // request sample
                    debug!("making clock sample request. qi: {}, now: {}", self.qi, now);
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

    async fn _set_top_and_prescaler(&mut self, new_value: TopAndPrescaler) -> Result<()> {
        use byteorder::{ByteOrder, LittleEndian};

        let mut buf = [0, 0, 0];
        LittleEndian::write_u16(&mut buf[0..2], new_value.avr_icr1());
        buf[2] = new_value.prescaler_key();

        self.icr1_and_prescaler = Some(new_value);

        self.write(b"T=").await?;
        self.write(&buf).await?;
        Ok(())
    }

    async fn _handle_returned_timestamp(
        &mut self,
        qi: u8,
        pulsenumber: u32,
        count: u16,
    ) -> Result<()> {
        debug!(
            "got returned timestamp with qi: {}, pulsenumber: {}, count: {}",
            qi, pulsenumber, count
        );
        let now = chrono::Utc::now();
        while self.queries.len() > 50 {
            self.queries.clear();
            error!("too many outstanding queries");
        }

        let send_timestamp = match self.queries.remove(&qi) {
            Some(send_timestamp) => send_timestamp,
            None => {
                warn!("could not find original data for query {:?}", qi);
                return Ok(());
            }
        };
        trace!("this query has send_timestamp: {}", send_timestamp);

        let max_error = now.signed_duration_since(send_timestamp);
        if max_error > self.max_acceptable_measurement_error {
            debug!("clock sample took {:?}. Ignoring value.", max_error);
            return Ok(());
        }

        trace!("max_error: {:?}", max_error);

        let ino_time_estimate = send_timestamp + (max_error / 2);

        match &self.icr1_and_prescaler {
            Some(s) => {
                let frac = count as f64 / s.avr_icr1() as f64;
                debug_assert!(0.0 <= frac);
                debug_assert!(frac <= 1.0);
                let ino_stamp = na::convert(pulsenumber as f64 + frac);

                if let Some(ref tbox_tx) = self.triggerbox_data_tx {
                    // send our newly acquired data to be saved to disk
                    let to_save = TriggerClockInfoRow {
                        start_timestamp: send_timestamp,
                        framecount: pulsenumber as i64,
                        tcnt: (frac * 255.0) as u8,
                        stop_timestamp: now,
                    };
                    match tbox_tx.send(to_save).await {
                        Ok(()) => {}
                        Err(e) => {
                            warn!("ignoring {}", e);
                        }
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
                    let (gain, offset, residuals) = fit_time_model(&self.past_data)
                        .map_err(|e| anyhow::anyhow!("lstsq err: {}", e))?;

                    let n_measurements = self.past_data.len() as u64;
                    let per_point_residual = residuals / n_measurements as f64;
                    // TODO only accept this if residuals less than some amount?
                    debug!(
                        "new: ClockModel{{gain: {}, offset: {}}}, per_point_residual: {}",
                        gain, offset, per_point_residual
                    );
                    (self.on_new_model_cb)(Some(ClockModel {
                        gain,
                        offset,
                        residuals,
                        n_measurements,
                    }));
                }
            }
            None => {
                warn!("No clock measurements until framerate set.");
            }
        }
        Ok(())
    }

    fn _handle_version(&mut self, value: u8, _pulsenumber: u32, _count: u16) -> Result<()> {
        trace!("got returned version with value: {}", value);
        assert_eq!(value, DEVICE_FIRMWARE_VERSION);
        self.vquery_time = chrono::Utc::now();
        self.version_check_done = true;
        info!("connected to triggerbox firmware version {}", value);
        Ok(())
    }

    async fn handle_data_from_device(&mut self, buf: Vec<u8>) -> Result<Vec<u8>> {
        if buf.len() >= 3 {
            // header, length, checksum is minimum
            let mut valid_n_chars = None;

            let packet_type = buf[0] as char;
            let payload_len = buf[1];

            let min_valid_packet_size = 3 + payload_len as usize; // header (2) + payload + checksum (1)
            if buf.len() >= min_valid_packet_size {
                let expected_chksum = buf[2 + payload_len as usize];

                let check_buf = &buf[2..buf.len() - 1];
                let bytes = check_buf;
                let actual_chksum = bytes.iter().fold(0, |acc: u8, x| acc.wrapping_add(*x));

                if actual_chksum == expected_chksum {
                    trace!("checksum OK");
                    valid_n_chars = Some(bytes.len() + 3)
                } else {
                    return Err(anyhow::anyhow!("checksum mismatch"));
                }

                if (packet_type == 'P') | (packet_type == 'V') {
                    assert!(payload_len == 7);
                    let value = bytes[0];

                    use byteorder::{ByteOrder, LittleEndian};
                    let pulsenumber = LittleEndian::read_u32(&bytes[1..5]);
                    let count = LittleEndian::read_u16(&bytes[5..7]);

                    match packet_type {
                        'P' => {
                            self._handle_returned_timestamp(value, pulsenumber, count)
                                .await?
                        }
                        'V' => self._handle_version(value, pulsenumber, count)?,
                        _ => unreachable!(),
                    };
                }
            }

            if let Some(n_used_chars) = valid_n_chars {
                return Ok(buf[n_used_chars..].to_vec());
            }
        }
        Ok(buf)
    }
}

fn fit_time_model(past_data: &[(f64, f64)]) -> Result<(f64, f64, f64), &'static str> {
    use na::{OMatrix, OVector, U2};

    let mut a: Vec<f64> = Vec::with_capacity(past_data.len() * 2);
    let mut b: Vec<f64> = Vec::with_capacity(past_data.len());

    for row in past_data.iter() {
        a.push(row.0);
        a.push(1.0);
        b.push(row.1);
    }
    let a = OMatrix::<f64, na::Dyn, U2>::from_row_slice(&a);
    let b = OVector::<f64, na::Dyn>::from_row_slice(&b);

    let epsilon = 1e-10;
    let results = lstsq::lstsq(&a, &b, epsilon)?;

    let gain = results.solution[0];
    let offset = results.solution[1];
    let residuals = results.residuals;

    Ok((gain, offset, residuals))
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

#[derive(Clone, Debug)]
pub struct TriggerboxOptions {
    pub device_path: String,
    pub query_dt: std::time::Duration,
    pub assert_device_name: NameType,
    pub max_acceptable_measurement_error: std::time::Duration,
    pub sleep_dur: std::time::Duration,
}

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
    let new_icr1_f64 = new_top_ideal.round();
    let new_icr1: u16 = if new_icr1_f64 > 0xFFFF as f64 {
        0xFFFF
    } else if new_icr1_f64 < 0.0 {
        0
    } else {
        new_icr1_f64 as u16
    };
    let rate_actual = base_clock / new_icr1 as f64;
    (new_icr1, rate_actual)
}

/// Given an ideal frame rate (in frames per second), compute the triggerbox
/// command which best approximates this frame rate.
///
/// Returns the triggerbox command and the expected actual frame rate (in frames
/// per second).
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
