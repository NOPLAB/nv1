use crate::{constants::*, line::LineProcessor, omni::OmniWheel, types::Vector2};
use core::f32::consts::PI;

pub struct MotorController {
    pub rotation_pid: pid::Pid<f32>,
    pub wheel_calc1: OmniWheel,
    pub wheel_calc2: OmniWheel,
    pub wheel_calc3: OmniWheel,
    pub wheel_calc4: OmniWheel,
    pub line_processor: LineProcessor,
}

impl Default for MotorController {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorController {
    pub fn new() -> Self {
        let mut rotation_pid: pid::Pid<f32> = pid::Pid::new(0.0, ROTATION_PID_LIMIT);
        rotation_pid.p(ROTATION_PID_P, ROTATION_PID_LIMIT);

        let wheel_calc1 = OmniWheel::new(WHEEL1_ANGLE.to_radians(), WHEEL_R, THREAD);
        let wheel_calc2 = OmniWheel::new(WHEEL2_ANGLE.to_radians(), WHEEL_R, THREAD);
        let wheel_calc3 = OmniWheel::new(WHEEL3_ANGLE.to_radians(), WHEEL_R, THREAD);
        let wheel_calc4 = OmniWheel::new(WHEEL4_ANGLE.to_radians(), WHEEL_R, THREAD);

        let line_processor = LineProcessor::new();

        Self {
            rotation_pid,
            wheel_calc1,
            wheel_calc2,
            wheel_calc3,
            wheel_calc4,
            line_processor,
        }
    }

    pub fn calculate_motor_values(
        &mut self,
        vel_x: f32,
        vel_y: f32,
        yaw: f32,
    ) -> (f32, f32, f32, f32) {
        // Refuse to feed non-finite inputs into the PID or kinematics.
        // A single NaN measurement would otherwise corrupt the PID's
        // integral term forever and propagate NaN motor commands over
        // the wire to the MD.
        if !vel_x.is_finite() || !vel_y.is_finite() || !yaw.is_finite() {
            return (0.0, 0.0, 0.0, 0.0);
        }

        self.rotation_pid.setpoint(0.0);
        let rotation_pid_result = self.rotation_pid.next_control_output(yaw);
        let rotation_vel = rotation_pid_result.output;

        let motor1 = self.wheel_calc1.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor2 = self.wheel_calc2.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor3 = self.wheel_calc3.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor4 = self.wheel_calc4.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);

        (motor1, motor2, motor3, motor4)
    }

    pub fn process_line(
        &mut self,
        on_line: Option<Vector2>,
        line_threshold: f32,
    ) -> Option<Vector2> {
        self.line_processor.process_line(on_line, line_threshold)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn zero_input_yields_zero_motor_speeds() {
        let mut c = MotorController::new();
        let (m1, m2, m3, m4) = c.calculate_motor_values(0.0, 0.0, 0.0);
        assert!(approx(m1, 0.0));
        assert!(approx(m2, 0.0));
        assert!(approx(m3, 0.0));
        assert!(approx(m4, 0.0));
    }

    #[test]
    fn positive_yaw_drives_rotation_in_one_direction() {
        let mut c = MotorController::new();
        // Yaw error > 0 should produce non-zero rotational output, and all
        // four wheels should turn in the same sign under pure rotation.
        let (m1, m2, m3, m4) = c.calculate_motor_values(0.0, 0.0, 1.0);
        assert!(m1.abs() > 0.0);
        assert_eq!(m1.signum(), m2.signum());
        assert_eq!(m1.signum(), m3.signum());
        assert_eq!(m1.signum(), m4.signum());
    }

    #[test]
    fn opposite_yaw_inputs_produce_opposite_outputs() {
        let mut c_pos = MotorController::new();
        let mut c_neg = MotorController::new();
        let pos = c_pos.calculate_motor_values(0.0, 0.0, 1.0);
        let neg = c_neg.calculate_motor_values(0.0, 0.0, -1.0);
        assert!(approx(pos.0, -neg.0));
        assert!(approx(pos.1, -neg.1));
        assert!(approx(pos.2, -neg.2));
        assert!(approx(pos.3, -neg.3));
    }

    #[test]
    fn translation_distributes_across_wheels() {
        let mut c = MotorController::new();
        let (m1, m2, m3, m4) = c.calculate_motor_values(1.0, 0.0, 0.0);
        // Under pure translation along +X, opposite wheels (1↔3, 2↔4) must
        // produce opposite-sign output to roll the chassis straight.
        assert!(approx(m1, -m3));
        assert!(approx(m2, -m4));
    }

    #[test]
    fn nan_yaw_returns_all_zero() {
        let mut c = MotorController::new();
        assert_eq!(
            c.calculate_motor_values(1.0, 0.0, f32::NAN),
            (0.0, 0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn nan_velocity_returns_all_zero() {
        let mut c = MotorController::new();
        assert_eq!(
            c.calculate_motor_values(f32::NAN, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0)
        );
        assert_eq!(
            c.calculate_motor_values(0.0, f32::NAN, 0.0),
            (0.0, 0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn infinite_inputs_return_all_zero() {
        let mut c = MotorController::new();
        assert_eq!(
            c.calculate_motor_values(f32::INFINITY, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0)
        );
        assert_eq!(
            c.calculate_motor_values(0.0, 0.0, f32::NEG_INFINITY),
            (0.0, 0.0, 0.0, 0.0)
        );
    }

    #[test]
    fn nan_input_does_not_corrupt_subsequent_computation() {
        let mut c = MotorController::new();
        // A non-finite tick must not poison the PID state.
        let _ = c.calculate_motor_values(f32::NAN, 0.0, 0.0);
        let (m1, _, _, _) = c.calculate_motor_values(0.0, 0.0, 1.0);
        assert!(m1.is_finite());
        assert!(m1.abs() > 0.0);
    }

    #[test]
    fn process_line_passes_through_to_processor() {
        let mut c = MotorController::new();
        // No line: passthrough should return None.
        assert!(c.process_line(None, 0.1).is_none());
        // Detect line: returns negated correction vector.
        let v = Vector2::new(1.0, 0.0);
        let out = c.process_line(Some(v), 0.1).unwrap();
        assert!(approx(out.x, -1.0));
    }
}
