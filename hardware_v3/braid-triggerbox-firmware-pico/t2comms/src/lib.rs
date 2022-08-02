#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
use defmt::Format;

#[cfg(not(feature = "std"))]
extern crate core as std;

#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub enum SyncVal {
    Sync0,
    Sync1,
    Sync2,
}

#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub struct NewTop {
    pub avr_icr1: u16,
    pub prescaler_key: u8,
}

#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
pub struct NewAOut {
    pub aout0: u16,
    pub aout1: u16,
    pub aout_sequence: u8,
}

#[derive(Clone, PartialEq, Debug)]
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
    SetTop(NewTop),
    SetAOut(NewAOut),
    Udev(UdevMsg),
}

#[derive(Clone, PartialEq, Debug)]
#[cfg_attr(not(feature = "std"), derive(Format))]
struct AccumState {
    last_update: u64,
}

impl Default for AccumState {
    fn default() -> Self {
        Self { last_update: 0 }
    }
}

pub const BUF_MAX_SZ: usize = 20;

/// Drop received data older than this (0.5 seconds).
const MAX_AGE_USEC: u64 = 500_000;

#[derive(Debug, Clone, PartialEq)]
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
                    result = Ok(UsbEvent::SetTop(NewTop {
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
    fn it_works() {
        for (buf, expected) in &[
            (&b"V?"[..], UsbEvent::VersionRequest),
            (&b"P1"[..], UsbEvent::TimestampQuery(b'1')),
            (&b"S0"[..], UsbEvent::Sync(SyncVal::Sync0)),
            (&b"S1"[..], UsbEvent::Sync(SyncVal::Sync1)),
            (&b"S2"[..], UsbEvent::Sync(SyncVal::Sync2)),
            /*
            SetTop(NewTop),
            SetAOut(NewAOut),
            Udev(UdevMsg),
                    */
        ][..]
        {
            check_simple(buf, &expected);
            check_stale(buf, &expected);
            check_multiple(buf, &expected);
        }
    }
}
