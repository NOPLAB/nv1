use libm::{cosf, sinf};

#[derive(Debug, Clone, Copy)]
pub struct OmniWheel {
    wheel_angle: f32,
    wheel_r: f32,
    tread: f32,
}

impl OmniWheel {
    pub fn new(wheel_angle: f32, wheel_r: f32, tread: f32) -> OmniWheel {
        OmniWheel {
            wheel_angle,
            wheel_r,
            tread,
        }
    }

    pub fn calculate(&self, linier_x: f32, linier_y: f32, angle: f32, angle_speed: f32) -> f32 {
        (-sinf(angle + self.wheel_angle) * cosf(angle) * linier_x
            + cosf(angle + self.wheel_angle) * cosf(angle) * linier_y
            + angle_speed * self.tread)
            / self.wheel_r
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    fn wheel(angle_deg: f32) -> OmniWheel {
        OmniWheel::new(angle_deg.to_radians(), 0.025, 0.108)
    }

    #[test]
    fn rest_input_yields_zero_speed() {
        let w = wheel(45.0);
        assert!(approx(w.calculate(0.0, 0.0, 0.0, 0.0), 0.0));
    }

    #[test]
    fn pure_rotation_scales_with_tread_over_radius() {
        let w = wheel(45.0);
        let expected = 0.108 / 0.025;
        assert!(approx(w.calculate(0.0, 0.0, 0.0, 1.0), expected));
        assert!(approx(w.calculate(0.0, 0.0, 0.0, -1.0), -expected));
    }

    #[test]
    fn pure_rotation_independent_of_wheel_angle() {
        let speeds: [f32; 4] = [45.0, 135.0, 225.0, 315.0]
            .map(|a| wheel(a).calculate(0.0, 0.0, 0.0, 1.0));
        for s in &speeds {
            assert!(approx(*s, speeds[0]));
        }
    }

    #[test]
    fn opposite_wheels_oppose_under_translation() {
        let w1 = wheel(45.0);
        let w3 = wheel(225.0);
        let s1 = w1.calculate(1.0, 0.0, 0.0, 0.0);
        let s3 = w3.calculate(1.0, 0.0, 0.0, 0.0);
        assert!(approx(s1, -s3), "s1={} s3={}", s1, s3);
    }

    #[test]
    fn quarter_turn_yaw_zeroes_x_translation_projection() {
        // cos(90°) = 0, so projecting +X body velocity at quarter-turn yaw
        // zeroes the wheel speed contribution.
        let w = wheel(45.0);
        let s_quarter = w.calculate(1.0, 0.0, FRAC_PI_2, 0.0);
        assert!(approx(s_quarter, 0.0));
    }

    #[test]
    fn larger_radius_lowers_required_wheel_speed() {
        let small = OmniWheel::new(FRAC_PI_4, 0.020, 0.108);
        let large = OmniWheel::new(FRAC_PI_4, 0.040, 0.108);
        let s_small = small.calculate(1.0, 0.0, 0.0, 0.0).abs();
        let s_large = large.calculate(1.0, 0.0, 0.0, 0.0).abs();
        assert!(approx(s_small, 2.0 * s_large));
    }

    #[test]
    fn full_turn_yaw_returns_to_initial() {
        let w = wheel(45.0);
        let s0 = w.calculate(1.0, 2.0, 0.0, 0.5);
        let s_full = w.calculate(1.0, 2.0, 2.0 * PI, 0.5);
        assert!(approx(s0, s_full));
    }
}
