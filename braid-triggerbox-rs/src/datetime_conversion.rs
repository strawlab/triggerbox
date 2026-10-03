// SPDX-License-Identifier: MIT OR Apache-2.0

use chrono::{DateTime, TimeZone};

/// Convert a time to seconds since the UNIX epoch.
pub(crate) fn datetime_to_f64<TZ>(dt: &DateTime<TZ>) -> f64
where
    TZ: TimeZone,
{
    #[expect(
        clippy::cast_precision_loss,
        reason = "seconds since the epoch are exact in f64 until year ~285 million"
    )]
    let secs = dt.timestamp() as f64;
    let nsecs = f64::from(dt.timestamp_subsec_nanos());
    secs + (nsecs * 1e-9)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn after_2038() {
        // Times after 2038 do not fit in 32 bit seconds.
        let dt = chrono::Utc
            .timestamp_opt(4_000_000_000, 500_000_000)
            .unwrap();
        assert!((datetime_to_f64(&dt) - 4_000_000_000.5).abs() < 1e-6);
    }
}
