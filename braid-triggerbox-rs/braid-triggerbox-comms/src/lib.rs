#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
use defmt::Format;

#[cfg(not(feature = "std"))]
extern crate core as std;

pub const DEVICE_FIRMWARE_VERSION: u8 = 14;

#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum SyncVal {
    Sync0,
    Sync1,
    Sync2,
}

#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub struct TopAndPrescaler {
    avr_icr1: u16,
    prescaler_key: u8,
}

impl TopAndPrescaler {
    pub fn new_avr(top: u16, prescaler: Prescaler) -> Self {
        let prescaler_key = match prescaler {
            Prescaler::Scale8 => b'1',
            Prescaler::Scale64 => b'2',
        };

        Self {
            avr_icr1: top,
            prescaler_key,
        }
    }
    #[inline]
    pub fn avr_icr1(&self) -> u16 {
        self.avr_icr1
    }

    #[inline]
    pub fn prescaler_key(&self) -> u8 {
        self.prescaler_key
    }
}

#[derive(Debug, Clone)]
pub enum Prescaler {
    Scale8,
    Scale64,
}

impl Prescaler {
    pub fn as_f64(&self) -> f64 {
        match self {
            Prescaler::Scale8 => 8.0,
            Prescaler::Scale64 => 64.0,
        }
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub struct NewAOut {
    pub aout0: u16,
    pub aout1: u16,
    pub aout_sequence: u8,
}

#[derive(Clone, PartialEq, Eq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum UdevMsg {
    Query,
    Set([u8; 8]),
}

#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum UsbEvent {
    TimestampQuery(u8),
    VersionRequest,
    Sync(SyncVal),
    SetTop(TopAndPrescaler),
    SetAOut(NewAOut),
    Udev(UdevMsg),
    SetLed(LedInfo),
}

#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub struct LedInfo {
    /// The duration the LED is on in each pulse (in microseceonds)
    ///
    /// This is the maximum value. Regardless of the value specified here, the
    /// LED on time cannot be longer than the inter-frame interval. Furthermore,
    /// it may also be limited by [Self::max_overall_duty_cycle].
    pub max_duration_usec: u32,
    /// The maximum fraction of time the LED is on
    ///
    /// This is a safety measure to reduce risk of destroying LEDs. When
    /// overdriving LEDs, it may be that they should only be on for a fraction
    /// of the time, e.g. 10% of the time. Setting this value to anything less
    /// than 1.0 will limit the LED on time to a fraction of total time. The
    /// [Self::nth_frame] field is taken into account within this computation.
    pub max_overall_duty_cycle: f32,
    /// What interval the LED is on
    ///
    /// 0: never
    /// 1: every frame
    /// 2: every second frame
    /// 3: every third frame
    /// 4: every fourth frame
    pub nth_frame: u8,
}

impl Default for LedInfo {
    fn default() -> Self {
        Self {
            max_duration_usec: 5_000,
            max_overall_duty_cycle: 1.0,
            nth_frame: 2,
        }
    }
}

const LED_MSG_BYTES: usize = 11;

impl LedInfo {
    #[cfg(feature = "std")]
    pub fn encode(&self) -> [u8; LED_MSG_BYTES] {
        let mut result: [u8; LED_MSG_BYTES] = [0; LED_MSG_BYTES];

        result[0..2].copy_from_slice(b"L=");
        result[2..6].copy_from_slice(&self.max_duration_usec.to_le_bytes());
        result[6..10].copy_from_slice(&self.max_overall_duty_cycle.to_le_bytes());
        result[10] = self.nth_frame;

        result
    }

    pub fn raw_decode(buf: &[u8]) -> Result<LedInfo, Error> {
        assert_eq!(buf.len(), LED_MSG_BYTES);
        let arr = buf[2..6].try_into().unwrap();
        let max_duration_usec = u32::from_le_bytes(arr);

        let arr = buf[6..10].try_into().unwrap();
        let max_overall_duty_cycle = f32::from_le_bytes(arr);

        let nth_frame = buf[10];
        Ok(LedInfo {
            max_duration_usec,
            max_overall_duty_cycle,
            nth_frame,
        })
    }
}

#[derive(Clone, PartialEq, Eq, Debug, Default)]
#[cfg_attr(not(feature = "std"), derive(Format))]
struct AccumState {
    last_update: u64,
}

pub const BUF_MAX_SZ: usize = 32;

/// Drop received data older than this (0.5 seconds).
const MAX_AGE_USEC: u64 = 500_000;

#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum Error {
    AwaitingMoreData,
    UnknownSyncValue,
    UnknownData,
}

enum PState {
    Empty,
    Accumulating(AccumState),
}

pub struct PacketParser<'bb> {
    /// buffer of accumulated input data
    prod: bbqueue::Producer<'bb, BUF_MAX_SZ>,
    cons: bbqueue::Consumer<'bb, BUF_MAX_SZ>,
    state: PState,
}

impl<'bb> PacketParser<'bb> {
    pub fn new(backing_store: &'bb bbqueue::BBBuffer<BUF_MAX_SZ>) -> Self {
        // let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let (prod, cons) = backing_store.try_split().unwrap();

        Self {
            prod,
            cons,
            state: PState::Empty,
        }
    }

    /// parse incoming data
    pub fn got_buf(&mut self, now_usec: u64, buf: &[u8]) -> Result<UsbEvent, Error> {
        let mut accum_state = match &self.state {
            PState::Empty => AccumState::default(),
            PState::Accumulating(old_accum_state) => {
                if (now_usec - old_accum_state.last_update) <= MAX_AGE_USEC {
                    old_accum_state.clone()
                } else {
                    match self.cons.read() {
                        Ok(rgrant) => {
                            let sz = rgrant.len();
                            rgrant.release(sz);
                        }
                        Err(bbqueue::Error::InsufficientSize) => { /*already empty*/ }
                        Err(e) => {
                            panic!("error: {:?}", e);
                        }
                    }
                    AccumState::default()
                }
            }
        };

        let mut wgrant = self.prod.grant_exact(buf.len()).unwrap();
        wgrant.clone_from_slice(buf);
        wgrant.commit(buf.len());
        accum_state.last_update = now_usec;

        self.state = PState::Accumulating(accum_state);

        let rgrant = self.cons.read().unwrap();
        let buf = rgrant.buf();

        let mut consumed_bytes = 0;

        let mut result = Err(Error::AwaitingMoreData);

        if buf.len() >= 2 {
            if buf == b"V?" {
                result = Ok(UsbEvent::VersionRequest);
                consumed_bytes = 2;
            } else if buf[0] == b'P' {
                result = Ok(UsbEvent::TimestampQuery(buf[1]));
                consumed_bytes = 2;
            } else if buf == b"N?" {
                result = Ok(UsbEvent::Udev(UdevMsg::Query));
                consumed_bytes = 2;
            } else if buf[0] == b'S' {
                result = match buf[1] {
                    b'0' => Ok(UsbEvent::Sync(SyncVal::Sync0)),
                    b'1' => Ok(UsbEvent::Sync(SyncVal::Sync1)),
                    b'2' => Ok(UsbEvent::Sync(SyncVal::Sync2)),
                    _ => Err(Error::UnknownSyncValue),
                };
                consumed_bytes = 2;
            } else if buf[0] == b'T' && buf[1] == b'=' {
                if buf.len() < 5 {
                    // wait for more data
                } else {
                    let value0 = buf[2];
                    let value1 = buf[3];
                    let prescaler_key = buf[4];

                    let avr_icr1 = u16::from_le_bytes([value0, value1]);
                    result = Ok(UsbEvent::SetTop(TopAndPrescaler {
                        avr_icr1,
                        prescaler_key,
                    }));
                    consumed_bytes = 5;
                }
            } else if buf[0] == b'L' && buf[1] == b'=' {
                if buf.len() < LED_MSG_BYTES {
                    // wait for more data
                } else {
                    let x2 = LedInfo::raw_decode(&buf[..LED_MSG_BYTES]).unwrap();
                    result = Ok(UsbEvent::SetLed(x2));
                    consumed_bytes = LED_MSG_BYTES;
                }
            } else if buf[0] == b'O' && buf[1] == b'=' {
                if buf.len() < 7 {
                    // wait for more data
                } else {
                    let aout0_0 = buf[2];
                    let aout0_1 = buf[3];
                    let aout0 = u16::from_le_bytes([aout0_0, aout0_1]);

                    let aout1_0 = buf[4];
                    let aout1_1 = buf[5];
                    let aout1 = u16::from_le_bytes([aout1_0, aout1_1]);

                    let aout_sequence = buf[6];

                    result = Ok(UsbEvent::SetAOut(NewAOut {
                        aout0,
                        aout1,
                        aout_sequence,
                    }));
                    consumed_bytes = 7;
                }
            } else {
                result = Err(Error::UnknownData);
                consumed_bytes = 2;
                // let str_buf = core::str::from_utf8(buf).unwrap_or("??");
                // todo!("unprocessed: '{}' ({:?})", str_buf, buf);
            }
        }

        rgrant.release(consumed_bytes);
        result
    }
}

// Run with: cargo test --target x86_64-pc-windows-msvc --features std
#[cfg(test)]
mod tests {
    use super::*;

    fn check_simple(buf: &[u8], expected: &UsbEvent) {
        // test 1 - simple normal situation
        let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let mut pp = PacketParser::new(&bb);
        let parsed = pp.got_buf(0, buf);
        assert_eq!(parsed, Ok(expected.clone()));
    }

    fn check_stale(buf: &[u8], expected: &UsbEvent) {
        // test 2 - old stale data present
        let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let mut pp = PacketParser::new(&bb);
        pp.got_buf(0, b"P").ok();
        let parsed = pp.got_buf(MAX_AGE_USEC + 1, buf);
        assert_eq!(parsed, Ok(expected.clone()));
    }

    fn check_multiple(buf: &[u8], expected: &UsbEvent) {
        // test 2 - old stale data present
        let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let mut pp = PacketParser::new(&bb);
        assert_eq!(Ok(UsbEvent::TimestampQuery(b'2')), pp.got_buf(0, b"P2"));
        let parsed = pp.got_buf(0, buf);
        assert_eq!(parsed, Ok(expected.clone()));
        assert_eq!(Ok(UsbEvent::TimestampQuery(b'3')), pp.got_buf(0, b"P3"));
    }

    #[test]
    fn manual_serialization() {
        for (buf, expected) in &[
            (&b"V?"[..], UsbEvent::VersionRequest),
            (&b"P1"[..], UsbEvent::TimestampQuery(b'1')),
            (&b"S0"[..], UsbEvent::Sync(SyncVal::Sync0)),
            (&b"S1"[..], UsbEvent::Sync(SyncVal::Sync1)),
            (&b"S2"[..], UsbEvent::Sync(SyncVal::Sync2)),
            /*
            SetTop(TopAndPrescaler),
            SetAOut(NewAOut),
            Udev(UdevMsg),
            SetLed(LedInfo),
                    */
        ][..]
        {
            check_simple(buf, &expected);
            check_stale(buf, &expected);
            check_multiple(buf, &expected);
        }
    }

    #[test]
    fn set_led_roundtrip() {
        let x1 = LedInfo::default();
        let buf = x1.encode();
        let x2 = LedInfo::raw_decode(&buf[..]).unwrap();
        assert_eq!(x1, x2);
    }
}
