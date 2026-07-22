const NANOSECONDS_TO_SECONDS: f64 = 1.0e-9;
const MAX_SAFE_NANOSECOND_DELTA: u64 = 1_u64 << f64::MANTISSA_DIGITS;

/// Signed time difference in seconds: preserve the sign of the difference
/// and reject integer deltas that cannot be represented exactly by `f64`.
pub(crate) fn diff_nanos_to_seconds(first_nanos: u64, second_nanos: u64) -> f64 {
    let (magnitude_nanos, sign) = if first_nanos >= second_nanos {
        (first_nanos - second_nanos, 1.0)
    } else {
        (second_nanos - first_nanos, -1.0)
    };

    if magnitude_nanos > MAX_SAFE_NANOSECOND_DELTA {
        log::error!("nanosecond difference {magnitude_nanos} exceeds the exact f64 integer range");
        return f64::NAN;
    }

    sign * magnitude_nanos as f64 * NANOSECONDS_TO_SECONDS
}

#[cfg(test)]
mod tests {
    use super::{diff_nanos_to_seconds, MAX_SAFE_NANOSECOND_DELTA};

    #[test]
    fn matches_upstream_signed_and_precision_checked_conversion() {
        assert_eq!(diff_nanos_to_seconds(750_000_000, 250_000_000), 0.5);
        assert_eq!(diff_nanos_to_seconds(250_000_000, 750_000_000), -0.5);
        assert!(diff_nanos_to_seconds(MAX_SAFE_NANOSECOND_DELTA + 1, 0).is_nan());
        assert!(diff_nanos_to_seconds(0, MAX_SAFE_NANOSECOND_DELTA + 1).is_nan());
        assert_eq!(
            diff_nanos_to_seconds(MAX_SAFE_NANOSECOND_DELTA, 0),
            MAX_SAFE_NANOSECOND_DELTA as f64 * 1.0e-9
        );
    }
}
