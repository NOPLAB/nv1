/// Low-level PWM-pair abstraction. An implementor drives two physical motors,
/// each with two PWM channels (forward / reverse half-bridge).
///
/// Implementations are platform-specific and live in the firmware crate.
pub trait MotorGroup {
    fn set_speed1(&mut self, speed: i16);
    fn stop1(&mut self);
    fn set_speed2(&mut self, speed: i16);
    fn stop2(&mut self);
}

/// Aggregates two [`MotorGroup`] implementations to expose a flat 4-motor API.
///
/// The mapping is fixed:
/// - motor 1 → group1 channel 1
/// - motor 2 → group1 channel 2
/// - motor 3 → group2 channel 1
/// - motor 4 → group2 channel 2
pub struct Motors<G1, G2>
where
    G1: MotorGroup,
    G2: MotorGroup,
{
    pub group1: G1,
    pub group2: G2,
}

impl<G1, G2> Motors<G1, G2>
where
    G1: MotorGroup,
    G2: MotorGroup,
{
    pub fn new(group1: G1, group2: G2) -> Motors<G1, G2> {
        Motors { group1, group2 }
    }

    pub fn set_speed(&mut self, idx: usize, speed: i16) {
        match idx {
            0 => self.group1.set_speed1(speed),
            1 => self.group1.set_speed2(speed),
            2 => self.group2.set_speed1(speed),
            3 => self.group2.set_speed2(speed),
            _ => {}
        }
    }

    pub fn stop(&mut self, idx: usize) {
        match idx {
            0 => self.group1.stop1(),
            1 => self.group1.stop2(),
            2 => self.group2.stop1(),
            3 => self.group2.stop2(),
            _ => {}
        }
    }

    pub fn stop_all(&mut self) {
        self.group1.stop1();
        self.group1.stop2();
        self.group2.stop1();
        self.group2.stop2();
    }
}

/// Saturating cast from a clamped controller output (typically `f32` in the
/// range `[-output_limit, output_limit]`) to the `i16` PWM command.
pub fn pwm_command_from_output(output: f32, command_inverted: bool) -> i16 {
    let signed = if command_inverted { -output } else { output };
    if !signed.is_finite() {
        return 0;
    }
    if signed >= i16::MAX as f32 {
        i16::MAX
    } else if signed <= i16::MIN as f32 {
        i16::MIN
    } else {
        signed as i16
    }
}

/// Convert a signed motor command into a `(forward, reverse)` duty pair with
/// hard saturation against the timer's `max_duty`.
///
/// `multiply` is the timer's `max_duty / nominal_full_scale_command` (e.g.
/// `max_duty / 128` if the firmware treats `±128` as full speed). Without the
/// `max_duty` clamp, an out-of-range command from a saturated controller would
/// silently overflow when cast to `u32`, producing runaway PWM.
pub fn duty_pair(speed: i16, multiply: f32, max_duty: u32) -> (u32, u32) {
    let scaled = (speed as f32) * multiply;
    let magnitude = scaled.abs();
    let clamped = if magnitude > max_duty as f32 {
        max_duty
    } else {
        magnitude as u32
    };
    if speed > 0 {
        (clamped, 0)
    } else if speed < 0 {
        (0, clamped)
    } else {
        (0, 0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::RefCell;

    #[derive(Default, Debug, PartialEq, Clone, Copy)]
    struct Channel {
        speed: Option<i16>,
        stopped: bool,
    }

    #[derive(Default)]
    struct MockGroup {
        ch1: RefCell<Channel>,
        ch2: RefCell<Channel>,
    }

    impl MotorGroup for MockGroup {
        fn set_speed1(&mut self, speed: i16) {
            *self.ch1.borrow_mut() = Channel {
                speed: Some(speed),
                stopped: false,
            };
        }
        fn stop1(&mut self) {
            *self.ch1.borrow_mut() = Channel {
                speed: None,
                stopped: true,
            };
        }
        fn set_speed2(&mut self, speed: i16) {
            *self.ch2.borrow_mut() = Channel {
                speed: Some(speed),
                stopped: false,
            };
        }
        fn stop2(&mut self) {
            *self.ch2.borrow_mut() = Channel {
                speed: None,
                stopped: true,
            };
        }
    }

    #[test]
    fn dispatch_per_index() {
        let mut motors = Motors::new(MockGroup::default(), MockGroup::default());
        motors.set_speed(0, 10);
        motors.set_speed(1, 20);
        motors.set_speed(2, 30);
        motors.set_speed(3, 40);
        assert_eq!(motors.group1.ch1.borrow().speed, Some(10));
        assert_eq!(motors.group1.ch2.borrow().speed, Some(20));
        assert_eq!(motors.group2.ch1.borrow().speed, Some(30));
        assert_eq!(motors.group2.ch2.borrow().speed, Some(40));
    }

    #[test]
    fn out_of_range_index_is_a_noop() {
        let mut motors = Motors::new(MockGroup::default(), MockGroup::default());
        motors.set_speed(99, 10);
        motors.stop(99);
        assert_eq!(motors.group1.ch1.borrow().speed, None);
    }

    #[test]
    fn stop_all_marks_every_channel_stopped() {
        let mut motors = Motors::new(MockGroup::default(), MockGroup::default());
        motors.set_speed(0, 10);
        motors.set_speed(3, 40);
        motors.stop_all();
        assert!(motors.group1.ch1.borrow().stopped);
        assert!(motors.group1.ch2.borrow().stopped);
        assert!(motors.group2.ch1.borrow().stopped);
        assert!(motors.group2.ch2.borrow().stopped);
    }

    #[test]
    fn pwm_command_passes_through_in_range() {
        assert_eq!(pwm_command_from_output(50.0, false), 50);
        assert_eq!(pwm_command_from_output(-50.0, false), -50);
    }

    #[test]
    fn pwm_command_inversion_flips_sign() {
        assert_eq!(pwm_command_from_output(50.0, true), -50);
        assert_eq!(pwm_command_from_output(-50.0, true), 50);
    }

    #[test]
    fn pwm_command_saturates_to_i16_range() {
        assert_eq!(pwm_command_from_output(1.0e9, false), i16::MAX);
        assert_eq!(pwm_command_from_output(-1.0e9, false), i16::MIN);
    }

    #[test]
    fn pwm_command_nan_or_inf_is_zero() {
        assert_eq!(pwm_command_from_output(f32::NAN, false), 0);
        assert_eq!(pwm_command_from_output(f32::INFINITY, false), 0);
        assert_eq!(pwm_command_from_output(f32::NEG_INFINITY, false), 0);
    }

    #[test]
    fn duty_pair_positive_drives_forward_only() {
        let (f, r) = duty_pair(64, 10.0, 1000);
        assert_eq!(f, 640);
        assert_eq!(r, 0);
    }

    #[test]
    fn duty_pair_negative_drives_reverse_only() {
        let (f, r) = duty_pair(-64, 10.0, 1000);
        assert_eq!(f, 0);
        assert_eq!(r, 640);
    }

    #[test]
    fn duty_pair_zero_is_brake() {
        let (f, r) = duty_pair(0, 10.0, 1000);
        assert_eq!(f, 0);
        assert_eq!(r, 0);
    }

    #[test]
    fn duty_pair_saturates_at_max_duty() {
        let (f, _) = duty_pair(150, 10.0, 1000);
        assert_eq!(f, 1000);
        let (_, r) = duty_pair(-150, 10.0, 1000);
        assert_eq!(r, 1000);
    }

    #[test]
    fn duty_pair_saturates_when_multiply_overshoots_max_duty() {
        // Plain cast `(150 * 10.0) as u32` = 1500, which exceeds max_duty=1000
        // and would have been written verbatim before the clamp.
        let (f, _) = duty_pair(150, 10.0, 1000);
        assert!(f <= 1000);
    }
}
