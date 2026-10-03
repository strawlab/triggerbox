// SPDX-License-Identifier: MIT OR Apache-2.0

//! Communication between the Braid triggerbox host library and device
//! firmware.
//!
//! This crate is `no_std` when the default `std` feature is disabled, so it
//! can be used by the device firmware.

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
use defmt::Format;

#[cfg(not(feature = "std"))]
use defmt::info;

#[cfg(feature = "std")]
use log::info;

/// The firmware version spoken by this version of the protocol.
pub const DEVICE_FIRMWARE_VERSION: u8 = 14;

/// Timestamp with microsecond resolution.
pub type Instant = fugit::MonotonicTimerInstantU64<1_000_000>;
type Duration = fugit::MicrosDurationU64;

/// Value of a host request to start or stop the trigger pulses.
#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum SyncVal {
    /// Stop the pulses and reset the pulse number.
    Sync0,
    /// Start the pulses.
    Sync1,
    /// Stop the pulses.
    Sync2,
}

/// The PWM top value and prescaler of the emulated Arduino Nano timer.
#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub struct TopAndPrescaler {
    avr_icr1: u16,
    prescaler_key: u8,
}

impl TopAndPrescaler {
    /// Create a new value from an Arduino Nano timer top and prescaler.
    #[must_use]
    pub fn new_avr(top: u16, prescaler: Prescaler) -> Self {
        Self {
            avr_icr1: top,
            prescaler_key: prescaler.key(),
        }
    }

    /// The Arduino Nano timer top value (the ICR1 register).
    #[inline]
    #[must_use]
    pub fn avr_icr1(&self) -> u16 {
        self.avr_icr1
    }

    /// The prescaler key as sent on the wire.
    #[inline]
    #[must_use]
    pub fn prescaler_key(&self) -> u8 {
        self.prescaler_key
    }

    /// The prescaler, or `None` if the key received on the wire is unknown.
    #[must_use]
    pub fn prescaler(&self) -> Option<Prescaler> {
        Prescaler::from_key(self.prescaler_key)
    }
}

/// The Arduino Nano timer prescaler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum Prescaler {
    /// Divide the clock by 8.
    Scale8,
    /// Divide the clock by 64.
    Scale64,
}

impl Prescaler {
    /// The clock divisor.
    #[must_use]
    pub fn as_f64(self) -> f64 {
        match self {
            Prescaler::Scale8 => 8.0,
            Prescaler::Scale64 => 64.0,
        }
    }

    fn key(self) -> u8 {
        match self {
            Prescaler::Scale8 => b'1',
            Prescaler::Scale64 => b'2',
        }
    }

    fn from_key(key: u8) -> Option<Self> {
        match key {
            b'1' => Some(Prescaler::Scale8),
            b'2' => Some(Prescaler::Scale64),
            _ => None,
        }
    }
}

/// Errors computing the emulated Arduino Nano PWM clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum PwmClockError {
    /// Only a 125 MHz system clock is supported.
    UnsupportedClock,
    /// The requested top value cannot be represented at this clock rate.
    TopOutOfRange,
}

/// Helper to calculate emulated Arduino Nano PWM clock.
#[derive(Debug)]
pub struct EmulatedNanoPwmClock {
    /// The clock divisor.
    div_int: u8,
    /// The original (Nano) PWM top value.
    orig_top: u16,
    /// The relative counter tick of the Pico vs Nano.
    clock_gain: f64,
    /// The actual top value.
    actual_top: u16,
    /// The system clock frequency (Hz).
    system_clock_freq_hz: u32,
}

impl EmulatedNanoPwmClock {
    /// Compute the PWM settings which emulate the Arduino Nano top value
    /// `orig_top`.
    ///
    /// # Errors
    ///
    /// Returns an error if the system clock is not supported or if `orig_top`
    /// cannot be emulated at this clock rate.
    pub fn new(
        orig_top: u16,
        is_mode2: bool,
        system_clock_freq_hz: u32,
    ) -> Result<Self, PwmClockError> {
        // These are the clock frequencies on the original trigger device which we
        // want to emulate.
        const EMULATE_CLOCK_MODE1: u32 = 2_000_000;
        const EMULATE_CLOCK_MODE2: u32 = 250_000;

        if system_clock_freq_hz != 125_000_000 {
            return Err(PwmClockError::UnsupportedClock);
        }

        let (div_int, emulate_clock) = if is_mode2 {
            (255, EMULATE_CLOCK_MODE2)
        } else {
            (62, EMULATE_CLOCK_MODE1)
        };

        let pwm_clock = f64::from(system_clock_freq_hz) / f64::from(div_int);

        let clock_gain = f64::from(emulate_clock) / pwm_clock;

        let new_top_f64 = f64::from(orig_top) / clock_gain;
        let actual_top = f64_to_u16_checked(new_top_f64)
            .and_then(|top| top.checked_sub(1))
            .ok_or(PwmClockError::TopOutOfRange)?;

        let result = Self {
            div_int,
            orig_top,
            clock_gain,
            actual_top,
            system_clock_freq_hz,
        };

        info!(
            "PWM orig_top: {}, to_top: {}, clock_gain: {}, emulate_clock: {}, div_int: {}, pwm_clock: {}",
            result.orig_top,
            result.to_top(),
            result.clock_gain,
            emulate_clock,
            result.div_int,
            pwm_clock,
        );

        Ok(result)
    }

    /// get integer part of clock divider
    #[must_use]
    pub fn div_int(&self) -> u8 {
        self.div_int
    }

    /// get system clock frequency (in Hz)
    #[must_use]
    pub fn system_clock_freq_hz(&self) -> u32 {
        self.system_clock_freq_hz
    }

    /// best effort to convert top value
    #[must_use]
    pub fn to_top(&self) -> u16 {
        self.actual_top
    }

    /// convert real ticks to scaled ticks
    #[must_use]
    pub fn scale_ticks(&self, ticks_real: u16) -> u16 {
        let ticks_scaled = f64::from(ticks_real) * self.clock_gain;
        // Due to rounding error, it is conceivable that we could exceed the
        // original top value. Here we ensure this is not possible.
        f64_to_u16_checked(ticks_scaled)
            .unwrap_or(self.orig_top)
            .min(self.orig_top)
    }
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

/// New analog output values requested by the host.
#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub struct NewAOut {
    /// Value of analog output 0.
    pub aout0: u16,
    /// Value of analog output 1.
    pub aout1: u16,
    /// Sequence number echoed back to the host.
    pub aout_sequence: u8,
}

/// Device name ("udev") message.
#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum UdevMsg {
    /// Query the device name.
    Query,
    /// Set the device name.
    Set([u8; 8]),
}

/// A message from the host to the device.
#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum UsbEvent {
    /// Request a timestamp, tagged with this value.
    TimestampQuery(u8),
    /// Request the firmware version.
    VersionRequest,
    /// Start or stop the trigger pulses.
    Sync(SyncVal),
    /// Set the trigger rate.
    SetTop(TopAndPrescaler),
    /// Set the analog outputs.
    SetAOut(NewAOut),
    /// Query or set the device name.
    Udev(UdevMsg),
}

#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
struct AccumState {
    last_update: Instant,
}

impl Default for AccumState {
    fn default() -> Self {
        Self {
            last_update: Instant::from_ticks(0),
        }
    }
}

/// Maximum number of bytes buffered by [`PacketParser`].
pub const BUF_MAX_SZ: usize = 32;

/// Drop received data older than this (0.5 seconds).
const MAX_AGE: Duration = Duration::from_ticks(500_000);

/// Errors parsing host messages.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum Error {
    /// A partial message was received.
    AwaitingMoreData,
    /// The value of a sync message was not recognized.
    UnknownSyncValue,
    /// The message was not recognized.
    UnknownData,
    /// More data was received than can be buffered. All buffered data was
    /// dropped.
    BufferOverflow,
}

#[derive(Debug)]
enum PState {
    Empty,
    Accumulating(AccumState),
}

/// Parse a stream of host messages.
#[derive(Debug)]
pub struct PacketParser {
    /// buffer of accumulated input data
    buf: [u8; BUF_MAX_SZ],
    /// number of valid bytes in `buf`
    len: usize,
    state: PState,
}

impl Default for PacketParser {
    fn default() -> Self {
        Self::new()
    }
}

impl PacketParser {
    /// Create a new parser with no buffered data.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0u8; BUF_MAX_SZ],
            len: 0,
            state: PState::Empty,
        }
    }

    /// parse incoming data
    ///
    /// # Errors
    ///
    /// Returns [`Error::AwaitingMoreData`] if no complete message has been
    /// received yet, or another [`Error`] if the data is invalid.
    pub fn got_buf(&mut self, now_usec: Instant, buf: &[u8]) -> Result<UsbEvent, Error> {
        let mut accum_state = match &self.state {
            PState::Empty => AccumState::default(),
            PState::Accumulating(old_accum_state) => {
                match now_usec.checked_duration_since(old_accum_state.last_update) {
                    Some(age) if age <= MAX_AGE => old_accum_state.clone(),
                    _ => {
                        // data expired (or the clock went backwards)
                        self.len = 0;
                        AccumState::default()
                    }
                }
            }
        };

        let Some((end, dest)) = self.len.checked_add(buf.len()).and_then(|end| {
            self.buf
                .get_mut(self.len..end)
                .map(|dest: &mut [u8]| (end, dest))
        }) else {
            self.len = 0;
            self.state = PState::Empty;
            return Err(Error::BufferOverflow);
        };
        dest.copy_from_slice(buf);
        self.len = end;
        accum_state.last_update = now_usec;

        self.state = PState::Accumulating(accum_state);

        let (result, consumed_bytes) = parse(self.buf.get(..self.len).unwrap_or_default());

        self.buf.copy_within(consumed_bytes..self.len, 0);
        self.len = self.len.saturating_sub(consumed_bytes);
        result
    }
}

/// Parse the first message in `buf`, returning it and the number of bytes it
/// used.
fn parse(buf: &[u8]) -> (Result<UsbEvent, Error>, usize) {
    match *buf {
        [b'V', b'?', ..] => (Ok(UsbEvent::VersionRequest), 2),
        [b'P', value, ..] => (Ok(UsbEvent::TimestampQuery(value)), 2),
        [b'N', b'?', ..] => (Ok(UsbEvent::Udev(UdevMsg::Query)), 2),
        [b'S', value, ..] => {
            let result = match value {
                b'0' => Ok(UsbEvent::Sync(SyncVal::Sync0)),
                b'1' => Ok(UsbEvent::Sync(SyncVal::Sync1)),
                b'2' => Ok(UsbEvent::Sync(SyncVal::Sync2)),
                _ => Err(Error::UnknownSyncValue),
            };
            (result, 2)
        }
        [b'T', b'=', value0, value1, prescaler_key, ..] => {
            let avr_icr1 = u16::from_le_bytes([value0, value1]);
            let event = UsbEvent::SetTop(TopAndPrescaler {
                avr_icr1,
                prescaler_key,
            });
            (Ok(event), 5)
        }
        [
            b'O',
            b'=',
            aout0_0,
            aout0_1,
            aout1_0,
            aout1_1,
            aout_sequence,
            ..,
        ] => {
            let event = UsbEvent::SetAOut(NewAOut {
                aout0: u16::from_le_bytes([aout0_0, aout0_1]),
                aout1: u16::from_le_bytes([aout1_0, aout1_1]),
                aout_sequence,
            });
            (Ok(event), 7)
        }
        [] | [_] | [b'T' | b'O', b'=', ..] => (Err(Error::AwaitingMoreData), 0),
        [_, _, ..] => (Err(Error::UnknownData), 2),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_simple(buf: &[u8], expected: &UsbEvent) {
        // test 1 - simple normal situation
        let mut pp = PacketParser::new();
        let parsed = pp.got_buf(Instant::from_ticks(0), buf);
        assert_eq!(parsed, Ok(expected.clone()));
    }

    fn check_stale(buf: &[u8], expected: &UsbEvent) {
        // test 2 - old stale data present
        let mut pp = PacketParser::new();
        let zero = Instant::from_ticks(0);
        pp.got_buf(zero, b"P").ok();
        // Just after `MAX_AGE` has elapsed.
        let later = Instant::from_ticks(500_001);
        assert!(later.checked_duration_since(zero) > Some(MAX_AGE));
        let parsed = pp.got_buf(later, buf);
        assert_eq!(parsed, Ok(expected.clone()));
    }

    fn check_multiple(buf: &[u8], expected: &UsbEvent) {
        // test 3 - multiple messages
        let mut pp = PacketParser::new();
        let zero = Instant::from_ticks(0);
        assert_eq!(Ok(UsbEvent::TimestampQuery(b'2')), pp.got_buf(zero, b"P2"));
        let parsed = pp.got_buf(zero, buf);
        assert_eq!(parsed, Ok(expected.clone()));
        assert_eq!(Ok(UsbEvent::TimestampQuery(b'3')), pp.got_buf(zero, b"P3"));
    }

    fn check_many_partial_messages(buf: &[u8], expected: &UsbEvent) {
        // test 4 - many partial messages
        let mut pp = PacketParser::new();
        let zero = Instant::from_ticks(0);
        for sz in 1..10 {
            for _ in 0..100 {
                assert_eq!(Ok(UsbEvent::TimestampQuery(b'2')), pp.got_buf(zero, b"P2"));

                let mut parsed = Err(Error::AwaitingMoreData);

                let mut chunks = buf.chunks(sz).peekable();
                while let Some(chunk) = chunks.next() {
                    parsed = pp.got_buf(zero, chunk);
                    if chunks.peek().is_some() {
                        assert_eq!(parsed, Err(Error::AwaitingMoreData));
                    }
                }
                assert_eq!(parsed, Ok(expected.clone()));
                assert_eq!(Ok(UsbEvent::TimestampQuery(b'3')), pp.got_buf(zero, b"P3"));
            }
        }
    }

    #[test]
    fn manual_serialization() {
        for (buf, expected) in &[
            (&b"V?"[..], UsbEvent::VersionRequest),
            (&b"P1"[..], UsbEvent::TimestampQuery(b'1')),
            (&b"S0"[..], UsbEvent::Sync(SyncVal::Sync0)),
            (&b"S1"[..], UsbEvent::Sync(SyncVal::Sync1)),
            (&b"S2"[..], UsbEvent::Sync(SyncVal::Sync2)),
            (
                &b"T=321"[..],
                UsbEvent::SetTop(TopAndPrescaler::new_avr(
                    (u16::from(b'2') << 8) | u16::from(b'3'),
                    Prescaler::Scale8,
                )),
            ),
            (
                &b"O=54321"[..],
                UsbEvent::SetAOut(NewAOut {
                    aout0: (u16::from(b'4') << 8) | u16::from(b'5'),
                    aout1: (u16::from(b'2') << 8) | u16::from(b'3'),
                    aout_sequence: b'1',
                }),
            ),
            (&b"N?"[..], UsbEvent::Udev(UdevMsg::Query)),
        ] {
            check_simple(buf, expected);
            check_stale(buf, expected);
            check_multiple(buf, expected);
            check_many_partial_messages(buf, expected);
        }
    }

    #[test]
    fn unknown_data_and_sync_value() {
        let zero = Instant::from_ticks(0);
        let mut pp = PacketParser::new();
        assert_eq!(pp.got_buf(zero, b"xy"), Err(Error::UnknownData));
        assert_eq!(pp.got_buf(zero, b"S9"), Err(Error::UnknownSyncValue));
        assert_eq!(pp.got_buf(zero, b"V?"), Ok(UsbEvent::VersionRequest));
    }

    #[test]
    fn overflow_drops_buffered_data() {
        let zero = Instant::from_ticks(0);
        let mut pp = PacketParser::new();
        assert_eq!(pp.got_buf(zero, b"T="), Err(Error::AwaitingMoreData));
        assert_eq!(
            pp.got_buf(zero, &[0u8; BUF_MAX_SZ]),
            Err(Error::BufferOverflow)
        );
        // The parser recovers after the overflow.
        assert_eq!(pp.got_buf(zero, b"V?"), Ok(UsbEvent::VersionRequest));
    }

    #[test]
    fn clock_going_backwards_drops_buffered_data() {
        let mut pp = PacketParser::new();
        assert_eq!(
            pp.got_buf(Instant::from_ticks(10), b"T="),
            Err(Error::AwaitingMoreData)
        );
        assert_eq!(
            pp.got_buf(Instant::from_ticks(0), b"V?"),
            Ok(UsbEvent::VersionRequest)
        );
    }

    #[test]
    fn prescaler_round_trip() {
        for prescaler in [Prescaler::Scale8, Prescaler::Scale64] {
            let tp = TopAndPrescaler::new_avr(1234, prescaler);
            assert_eq!(tp.prescaler(), Some(prescaler));
        }
        let unknown = TopAndPrescaler {
            avr_icr1: 0,
            prescaler_key: b'x',
        };
        assert_eq!(unknown.prescaler(), None);
    }

    #[test]
    fn emulated_clock() {
        const CLOCK: u32 = 125_000_000;
        assert_eq!(
            EmulatedNanoPwmClock::new(50_000, false, 1).err(),
            Some(PwmClockError::UnsupportedClock)
        );
        // A top value too small to emulate.
        assert_eq!(
            EmulatedNanoPwmClock::new(0, false, CLOCK).err(),
            Some(PwmClockError::TopOutOfRange)
        );
        // A top value too large to emulate.
        assert_eq!(
            EmulatedNanoPwmClock::new(u16::MAX, true, CLOCK).err(),
            Some(PwmClockError::TopOutOfRange)
        );

        let clock = EmulatedNanoPwmClock::new(50_000, false, CLOCK).unwrap();
        assert_eq!(clock.div_int(), 62);
        assert_eq!(clock.scale_ticks(0), 0);
        assert!(clock.scale_ticks(clock.to_top()) <= 50_000);
        assert_eq!(clock.scale_ticks(u16::MAX), 50_000);
    }
}
