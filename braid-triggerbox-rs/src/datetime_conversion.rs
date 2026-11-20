// SPDX-License-Identifier: MIT OR Apache-2.0

use jiff::Timestamp;

/// Convert a time to seconds since the UNIX epoch.
pub(crate) fn timestamp_to_f64(ts: Timestamp) -> f64 {
    ts.as_duration().as_secs_f64()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn after_2038() {
        // Times after 2038 do not fit in 32 bit seconds.
        let ts = Timestamp::new(4_000_000_000, 500_000_000).unwrap();
        assert!((timestamp_to_f64(ts) - 4_000_000_000.5).abs() < 1e-6);
    }
}
