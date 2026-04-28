/// Geometry of a single quadrature encoder + gearbox.
#[derive(Clone, Copy, Debug)]
pub struct EncoderConfig {
    /// Encoder counts per full revolution of the **motor** shaft (pre-gearbox).
    pub counts_per_motor_rev: u32,
    /// Output revolutions per motor revolution. For a 1:19.225 reduction this
    /// is `1.0 / 19.225`.
    pub gear_ratio: f32,
    /// Negate the measured count delta. Use this when the encoder is mounted
    /// such that positive shaft rotation produces decreasing counts in the
    /// peripheral.
    pub inverted: bool,
}

/// Tracks a wrapping `u32` encoder count and yields per-call output RPS.
pub struct EncoderTracker {
    prev: u32,
    config: EncoderConfig,
}

impl EncoderTracker {
    pub fn new(initial_count: u32, config: EncoderConfig) -> Self {
        Self {
            prev: initial_count,
            config,
        }
    }

    /// Read the new wrapping count and return output-shaft RPS over the last
    /// `period_s` seconds. Deltas are reduced through `i16` so the maximum
    /// representable change between calls is +/- 32_768 counts.
    pub fn update(&mut self, current_count: u32, period_s: f32) -> f32 {
        debug_assert!(period_s > 0.0);
        debug_assert!(self.config.counts_per_motor_rev > 0);
        let raw_delta = current_count.wrapping_sub(self.prev) as i16 as i32;
        self.prev = current_count;
        let signed = if self.config.inverted {
            -raw_delta
        } else {
            raw_delta
        };
        signed as f32 / self.config.counts_per_motor_rev as f32 * self.config.gear_ratio
            / period_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 100 counts per motor rev, 1:2 reduction → 200 counts per output rev.
    fn cfg() -> EncoderConfig {
        EncoderConfig {
            counts_per_motor_rev: 100,
            gear_ratio: 0.5,
            inverted: false,
        }
    }

    #[test]
    fn positive_delta_yields_positive_rps() {
        let mut t = EncoderTracker::new(0, cfg());
        // 200 counts in 1 second = 1 output revolution per second
        let rps = t.update(200, 1.0);
        assert!((rps - 1.0).abs() < 1e-6, "got {rps}");
    }

    #[test]
    fn negative_delta_yields_negative_rps() {
        let mut t = EncoderTracker::new(200, cfg());
        let rps = t.update(0, 1.0);
        assert!((rps + 1.0).abs() < 1e-6, "got {rps}");
    }

    #[test]
    fn inverted_flips_sign() {
        let cfg_inv = EncoderConfig {
            inverted: true,
            ..cfg()
        };
        let mut t = EncoderTracker::new(0, cfg_inv);
        let rps = t.update(200, 1.0);
        assert!((rps + 1.0).abs() < 1e-6, "got {rps}");
    }

    #[test]
    fn wraparound_is_signed_via_i16() {
        let mut t = EncoderTracker::new(u32::MAX - 5, cfg());
        // Wrap forward by 10 counts: u32::MAX-5 -> 4 (i.e. delta = 10)
        let rps = t.update(4, 1.0);
        let expected = 10.0 / 100.0 * 0.5;
        assert!(
            (rps - expected).abs() < 1e-6,
            "got {rps}, expected {expected}"
        );
    }

    #[test]
    fn wraparound_backwards_signed_negative() {
        let mut t = EncoderTracker::new(4, cfg());
        // Wrap back by 10 counts: 4 -> u32::MAX-5 (delta = -10 via i16)
        let rps = t.update(u32::MAX - 5, 1.0);
        let expected = -10.0 / 100.0 * 0.5;
        assert!(
            (rps - expected).abs() < 1e-6,
            "got {rps}, expected {expected}"
        );
    }

    #[test]
    fn period_scales_inversely() {
        let mut t = EncoderTracker::new(0, cfg());
        let rps_short = t.update(200, 0.01);
        let mut t2 = EncoderTracker::new(0, cfg());
        let rps_long = t2.update(200, 0.02);
        assert!((rps_short - 2.0 * rps_long).abs() < 1e-3);
    }

    #[test]
    fn zero_delta_is_zero_rps() {
        let mut t = EncoderTracker::new(12345, cfg());
        assert_eq!(t.update(12345, 0.01), 0.0);
    }

    #[test]
    fn u16_sourced_wraparound_is_signed() {
        // The QEI peripheral on this board returns u16. The firmware widens
        // to u32 before calling update(); the math must still recover the
        // smallest signed delta. 0xFFF5 -> 0x0005 = +16 forward.
        let mut t = EncoderTracker::new(0xFFF5, cfg());
        let rps = t.update(0x0005, 1.0);
        let expected = 16.0 / 100.0 * 0.5;
        assert!(
            (rps - expected).abs() < 1e-6,
            "got {rps}, expected {expected}"
        );

        // 0x0005 -> 0xFFF5 = -16 backward.
        let mut t2 = EncoderTracker::new(0x0005, cfg());
        let rps2 = t2.update(0xFFF5, 1.0);
        assert!(
            (rps2 + expected).abs() < 1e-6,
            "got {rps2}, expected {}",
            -expected
        );
    }

    #[test]
    fn matches_legacy_constants() {
        // Legacy firmware used: counts/rev = 12 (3 PPR x4), gear = 1/19.225,
        // period = 0.01s. A 23-count delta should produce ~10 output rps.
        let cfg = EncoderConfig {
            counts_per_motor_rev: 12,
            gear_ratio: 1.0 / 19.225,
            inverted: false,
        };
        let mut t = EncoderTracker::new(0, cfg);
        let rps = t.update(23, 0.01);
        let expected = 23.0 / 12.0 * (1.0 / 19.225) / 0.01;
        assert!(
            (rps - expected).abs() < 1e-3,
            "got {rps}, expected {expected}"
        );
    }
}
