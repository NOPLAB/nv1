use crate::{constants::*, omni::OmniWheel, sensors::LineProcessor, types::Vector2};
use core::f32::consts::PI;
use pid;

pub struct MotorController {
    pub rotation_pid: pid::Pid<f32>,
    pub wheel_calc1: OmniWheel,
    pub wheel_calc2: OmniWheel,
    pub wheel_calc3: OmniWheel,
    pub wheel_calc4: OmniWheel,
    pub line_processor: LineProcessor,
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
