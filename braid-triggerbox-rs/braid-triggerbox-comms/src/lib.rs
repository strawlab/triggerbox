#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
use defmt::Format;

#[cfg(not(feature = "std"))]
extern crate core as std;

pub const DEVICE_FIRMWARE_VERSION: u8 = 14;

type Instant = fugit::Instant<u64, 1, 1_000_000>;
type Duration = fugit::Duration<u64, 1, 1_000_000>;

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

pub const BUF_MAX_SZ: usize = 32;

/// Drop received data older than this (0.5 seconds).
const MAX_AGE: Duration = Duration::from_ticks(500_000);

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
    pub fn got_buf(&mut self, now_usec: Instant, buf: &[u8]) -> Result<UsbEvent, Error> {
        let mut accum_state = match &self.state {
            PState::Empty => AccumState::default(),
            PState::Accumulating(old_accum_state) => {
                if (now_usec - old_accum_state.last_update) <= MAX_AGE {
                    old_accum_state.clone()
                } else {
                    // data expired
                    match self.cons.split_read() {
                        Ok(grant) => {
                            let bufs = grant.bufs();
                            let sz = bufs.0.len() + bufs.1.len();
                            grant.release(sz);
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

        // This is ugly and inefficient, but our tests pass.
        let grant = self.cons.split_read().unwrap();
        let bufs = grant.bufs();

        let mut fullbuf = [0u8; BUF_MAX_SZ];
        fullbuf[..bufs.0.len()].copy_from_slice(bufs.0);
        fullbuf[bufs.0.len()..bufs.0.len() + bufs.1.len()].copy_from_slice(bufs.1);
        let buf = &fullbuf[..bufs.0.len() + bufs.1.len()];

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

        grant.release(consumed_bytes);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn check_simple(buf: &[u8], expected: &UsbEvent) {
        // test 1 - simple normal situation
        let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let mut pp = PacketParser::new(&bb);
        let parsed = pp.got_buf(Instant::from_ticks(0), buf);
        assert_eq!(parsed, Ok(expected.clone()));
    }

    fn check_stale(buf: &[u8], expected: &UsbEvent) {
        // test 2 - old stale data present
        let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let mut pp = PacketParser::new(&bb);
        let zero = Instant::from_ticks(0);
        pp.got_buf(zero, b"P").ok();
        let parsed = pp.got_buf(zero + MAX_AGE + Duration::from_ticks(1), buf);
        assert_eq!(parsed, Ok(expected.clone()));
    }

    fn check_multiple(buf: &[u8], expected: &UsbEvent) {
        // test 3 - multiple messages
        let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let mut pp = PacketParser::new(&bb);
        let zero = Instant::from_ticks(0);
        assert_eq!(Ok(UsbEvent::TimestampQuery(b'2')), pp.got_buf(zero, b"P2"));
        let parsed = pp.got_buf(zero, buf);
        assert_eq!(parsed, Ok(expected.clone()));
        assert_eq!(Ok(UsbEvent::TimestampQuery(b'3')), pp.got_buf(zero, b"P3"));
    }

    fn check_many_partial_messages(buf: &[u8], expected: &UsbEvent) {
        // test 4 - many partial messages
        let bb: bbqueue::BBBuffer<BUF_MAX_SZ> = bbqueue::BBBuffer::new();
        let mut pp = PacketParser::new(&bb);
        let zero = Instant::from_ticks(0);
        for sz in 1..10 {
            for _ in 0..100 {
                assert_eq!(Ok(UsbEvent::TimestampQuery(b'2')), pp.got_buf(zero, b"P2"));

                let mut parsed = Err(Error::AwaitingMoreData);

                let chunk_iter = buf.chunks(sz);
                let mut n_sent = 0;
                for chunk in chunk_iter {
                    n_sent += chunk.len();
                    parsed = pp.got_buf(zero, chunk);
                    if n_sent < buf.len() {
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
                    ((b'2' as u16) << 8) | b'3' as u16,
                    Prescaler::Scale8,
                )),
            ),
            (
                &b"O=54321"[..],
                UsbEvent::SetAOut(NewAOut {
                    aout0: ((b'4' as u16) << 8) | b'5' as u16,
                    aout1: ((b'2' as u16) << 8) | b'3' as u16,
                    aout_sequence: b'1',
                }),
            ),
            (&b"N?"[..], UsbEvent::Udev(UdevMsg::Query)),
        ] {
            check_simple(buf, &expected);
            check_stale(buf, &expected);
            check_multiple(buf, &expected);
            check_many_partial_messages(buf, &expected);
        }
    }
}
