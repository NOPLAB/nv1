use pid::Pid;

/// Per-axis PID gains and saturation limits.
#[derive(Clone, Copy, Debug)]
pub struct SpeedControllerConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub p_limit: f32,
    pub i_limit: f32,
    pub d_limit: f32,
    /// Symmetric clamp on the controller output.
    pub output_limit: f32,
}

impl SpeedControllerConfig {
    pub const fn new(kp: f32, ki: f32, kd: f32, output_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            p_limit: output_limit,
            i_limit: output_limit / 2.0,
            d_limit: output_limit,
            output_limit,
        }
    }
}

fn build_pid(config: SpeedControllerConfig) -> Pid<f32> {
    let mut pid = Pid::new(0.0, config.output_limit);
    pid.p(config.kp, config.p_limit)
        .i(config.ki, config.i_limit)
        .d(config.kd, config.d_limit);
    pid
}

/// Closed-loop wrapper around `pid::Pid<f32>` that:
///
/// - Skips the update and returns `None` when the measurement is non-finite,
///   keeping the integral term frozen instead of corrupting it
/// - Clamps the final output symmetrically to `output_limit`
pub struct SpeedController {
    pid: Pid<f32>,
    config: SpeedControllerConfig,
}

impl SpeedController {
    pub fn new(config: SpeedControllerConfig) -> Self {
        Self {
            pid: build_pid(config),
            config,
        }
    }

    pub fn setpoint(&mut self, sp: f32) {
        self.pid.setpoint = sp;
    }

    /// Drops the integral accumulation and clears the derivative history.
    pub fn reset(&mut self) {
        let setpoint = self.pid.setpoint;
        self.pid = build_pid(self.config);
        self.pid.setpoint = setpoint;
    }

    /// Returns the clamped output. `None` if the measurement is non-finite —
    /// caller should hold the previous command or trigger the failsafe.
    pub fn next(&mut self, measurement: f32) -> Option<f32> {
        if !measurement.is_finite() {
            return None;
        }
        let raw = self.pid.next_control_output(measurement).output;
        let limit = self.config.output_limit;
        let clamped = if raw > limit {
            limit
        } else if raw < -limit {
            -limit
        } else {
            raw
        };
        Some(clamped)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> SpeedControllerConfig {
        SpeedControllerConfig::new(8.0, 4.0, 0.0, 100.0)
    }

    #[test]
    fn zero_error_yields_zero_output() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(0.0);
        let out = c.next(0.0).unwrap();
        assert!(out.abs() < 1e-6);
    }

    #[test]
    fn positive_error_drives_positive_output() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(5.0);
        let out = c.next(0.0).unwrap();
        assert!(out > 0.0, "got {out}");
    }

    #[test]
    fn output_is_clamped_to_limit() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(1_000.0);
        for _ in 0..100 {
            let out = c.next(0.0).unwrap();
            assert!((-100.0..=100.0).contains(&out), "saturation breach: {out}");
        }
    }

    #[test]
    fn negative_setpoint_drives_negative_output() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(-5.0);
        let out = c.next(0.0).unwrap();
        assert!(out < 0.0, "got {out}");
    }

    #[test]
    fn nan_measurement_returns_none() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(5.0);
        assert!(c.next(f32::NAN).is_none());
    }

    #[test]
    fn infinite_measurement_returns_none() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(5.0);
        assert!(c.next(f32::INFINITY).is_none());
        assert!(c.next(f32::NEG_INFINITY).is_none());
    }

    #[test]
    fn nan_skip_does_not_corrupt_subsequent_step() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(5.0);
        let _ = c.next(0.0);
        assert!(c.next(f32::NAN).is_none());
        let out = c.next(0.0).unwrap();
        assert!(out.is_finite());
        assert!(out > 0.0);
    }

    #[test]
    fn reset_zeros_integral_accumulation() {
        let mut c = SpeedController::new(cfg());
        c.setpoint(5.0);
        for _ in 0..50 {
            let _ = c.next(0.0);
        }
        c.reset();
        c.setpoint(0.0);
        let out = c.next(0.0).unwrap();
        // After reset with zero error and zero measurement, output should be ~0
        assert!(out.abs() < 1e-3, "expected near-zero, got {out}");
    }
}
