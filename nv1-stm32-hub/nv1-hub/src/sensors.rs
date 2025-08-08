use core::f32::consts::PI;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals;
use num_traits::{AsPrimitive, Num};

use crate::constants::*;
use crate::types::Vector2;

pub enum AdcState {
    OnGround,
    OnLine(f32, f32, f32, u32),
    OutOfLineOverCenter(f32, f32, f32, u32),
}

pub struct AdcSensor {
    pub adc: Adc<'static, peripherals::ADC1>,
    pub line_sensor: LineSensor,
    pub ir_ball_sensor: IrBallSensor,
}

pub struct LineSensor {
    pub s0: Output<'static>,
    pub s1: Output<'static>,
    pub s2: Output<'static>,
    pub s3: Output<'static>,
    pub sin_values: [f32; LINE_SENSORS_COUNT],
    pub cos_values: [f32; LINE_SENSORS_COUNT],
}

pub struct IrBallSensor {
    pub s0: Output<'static>,
    pub s1: Output<'static>,
    pub s2: Output<'static>,
    pub s3: Output<'static>,
    pub sin_values: [f32; IR_SENSORS_COUNT],
    pub cos_values: [f32; IR_SENSORS_COUNT],
}

impl AdcSensor {
    pub fn new(
        mut adc: Adc<'static, peripherals::ADC1>,
        line_s0: Output<'static>,
        line_s1: Output<'static>,
        line_s2: Output<'static>,
        line_s3: Output<'static>,
        ir_s0: Output<'static>,
        ir_s1: Output<'static>,
        ir_s2: Output<'static>,
        ir_s3: Output<'static>,
    ) -> Self {
        adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES56);

        let mut line_sin = [0.0_f32; LINE_SENSORS_COUNT];
        let mut line_cos = [0.0_f32; LINE_SENSORS_COUNT];
        generate_adc_vec(
            &mut line_sin,
            &mut line_cos,
            90.0_f32.to_radians(),
            -(360.0_f32 / LINE_SENSORS_COUNT as f32).to_radians(),
            1.0,
        );

        let mut ir_sin = [0.0_f32; IR_SENSORS_COUNT];
        let mut ir_cos = [0.0_f32; IR_SENSORS_COUNT];
        generate_adc_vec(
            &mut ir_sin,
            &mut ir_cos,
            90_f32.to_radians(),
            -(360.0_f32 / IR_SENSORS_COUNT as f32).to_radians(),
            1.0,
        );

        let line_sensor = LineSensor {
            s0: line_s0,
            s1: line_s1,
            s2: line_s2,
            s3: line_s3,
            sin_values: line_sin,
            cos_values: line_cos,
        };

        let ir_ball_sensor = IrBallSensor {
            s0: ir_s0,
            s1: ir_s1,
            s2: ir_s2,
            s3: ir_s3,
            sin_values: ir_sin,
            cos_values: ir_cos,
        };

        Self {
            adc,
            line_sensor,
            ir_ball_sensor,
        }
    }

    pub fn read_sensors(
        &mut self,
        pc0: &mut peripherals::PC0,
        pc1: &mut peripherals::PC1,
        pc2: &mut peripherals::PC2,
        pc3: &mut peripherals::PC3,
    ) -> SensorReadings {
        let mut adc_line = [0u16; LINE_SENSORS_COUNT];
        let mut adc_ir = [0u16; IR_SENSORS_COUNT];
        let adc_have_ball = self.adc.blocking_read(pc3);

        for i in 0..IR_SENSORS_COUNT {
            self.set_multiplexer_pins(i);

            adc_line[i] = self.adc.blocking_read(pc0);
            adc_line[i + IR_SENSORS_COUNT] = self.adc.blocking_read(pc1);
            adc_ir[i] = self.adc.blocking_read(pc2);
        }

        let adc_line = adc_line
            .iter()
            .map(|x| (*x as f32) / ADC_RESOLUTION)
            .collect::<alloc::vec::Vec<_>>();

        adc_ir.iter_mut().for_each(|x| *x = 4096 - *x);
        let adc_ir = adc_ir
            .iter()
            .map(|x| (*x as f32) / ADC_RESOLUTION)
            .collect::<alloc::vec::Vec<_>>();

        let line_vector = calculate_line_vec_with_threshold(
            &adc_line,
            &self.line_sensor.sin_values,
            &self.line_sensor.cos_values,
            0.1, // Default threshold - will be overridden by caller
        );

        let (ir_x, ir_y, _ir_strength) = calculate_adc_vec(
            &adc_ir,
            &self.ir_ball_sensor.sin_values,
            &self.ir_ball_sensor.cos_values,
            1.0,
        );
        let ir_angle = libm::atan2f(ir_y, ir_x);
        let adc_line_max = adc_line.iter().cloned().reduce(f32::max).unwrap_or(0.0);

        SensorReadings {
            line_vector,
            ir_angle,
            ir_position: Vector2::new(ir_x, ir_y),
            adc_have_ball,
            adc_line_max,
            adc_line,
        }
    }

    fn set_multiplexer_pins(&mut self, index: usize) {
        let set_pin_state = |pin: &mut Output<'static>, bit_mask: usize| {
            if index & bit_mask != 0 {
                pin.set_high();
            } else {
                pin.set_low();
            }
        };

        set_pin_state(&mut self.line_sensor.s0, 0b0001);
        set_pin_state(&mut self.line_sensor.s1, 0b0010);
        set_pin_state(&mut self.line_sensor.s2, 0b0100);
        set_pin_state(&mut self.line_sensor.s3, 0b1000);

        set_pin_state(&mut self.ir_ball_sensor.s0, 0b0001);
        set_pin_state(&mut self.ir_ball_sensor.s1, 0b0010);
        set_pin_state(&mut self.ir_ball_sensor.s2, 0b0100);
        set_pin_state(&mut self.ir_ball_sensor.s3, 0b1000);
    }
}

pub struct SensorReadings {
    #[allow(dead_code)]
    pub line_vector: Option<Vector2>,
    pub ir_angle: f32,
    pub ir_position: Vector2,
    pub adc_have_ball: u16,
    pub adc_line_max: f32,
    pub adc_line: alloc::vec::Vec<f32>,
}

pub struct LineProcessor {
    pub state: AdcState,
}

impl LineProcessor {
    pub fn new() -> Self {
        Self {
            state: AdcState::OnGround,
        }
    }

    pub fn process_line(
        &mut self,
        line_vector: Option<Vector2>,
        _threshold: f32,
    ) -> Option<Vector2> {
        match self.state {
            AdcState::OnGround => {
                self.handle_on_ground_state(line_vector)
            }
            AdcState::OnLine(first_angle, first_x, first_y, counter) => {
                self.handle_on_line_state(line_vector, first_angle, first_x, first_y, counter)
            }
            AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter) => {
                self.handle_out_of_line_over_center_state(
                    line_vector, first_angle, first_x, first_y, counter
                )
            }
        }
    }
    

    
    fn handle_on_ground_state(&mut self, line_vector: Option<Vector2>) -> Option<Vector2> {
        if let Some(detected_vector) = line_vector {
            // Line detected - transition to OnLine state
            let detected_angle = detected_vector.angle();
            
            self.state = AdcState::OnLine(detected_angle, detected_vector.x, detected_vector.y, 0);
            Some(-detected_vector) // Return correction vector (opposite direction)
        } else {
            // No line detected - stay on ground
            self.state = AdcState::OnGround;
            None
        }
    }
    
    fn handle_on_line_state(
        &mut self,
        line_vector: Option<Vector2>,
        first_angle: f32,
        first_x: f32,
        first_y: f32,
        counter: u32,
    ) -> Option<Vector2> {
        const COUNTER_TIMEOUT_LIMIT: u32 = 100;
        
        if let Some(current_vector) = line_vector {
            // Line still detected - check if robot has crossed over center
            let current_angle = current_vector.angle();
            
            let angle_tolerance = LINE_OVER_CENTER_THRESHOLD / 2.0;
            let min_allowed_angle = first_angle - angle_tolerance;
            let max_allowed_angle = first_angle + angle_tolerance;
            
            if self.is_angle_outside_range(current_angle, min_allowed_angle, max_allowed_angle) {
                // Robot has crossed over center - transition to OutOfLineOverCenter
                self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, 0);
                Some(-current_vector)
            } else {
                // Still on same side of line - reset counter and continue
                self.state = AdcState::OnLine(first_angle, first_x, first_y, 0);
                Some(-current_vector)
            }
        } else {
            // Line lost - use timeout counter to decide next state
            if counter > COUNTER_TIMEOUT_LIMIT {
                // Timeout exceeded - return to ground
                self.state = AdcState::OnGround;
                None
            } else {
                // Still within timeout - keep using last known position
                let last_known_vector = Vector2::new(first_x, first_y);
                self.state = AdcState::OnLine(first_angle, first_x, first_y, counter + 1);
                Some(-last_known_vector)
            }
        }
    }
    
    fn handle_out_of_line_over_center_state(
        &mut self,
        line_vector: Option<Vector2>,
        first_angle: f32,
        first_x: f32,
        first_y: f32,
        counter: u32,
    ) -> Option<Vector2> {
        const COUNTER_TIMEOUT_LIMIT: u32 = 100;
        let original_vector = Vector2::new(first_x, first_y);
        
        if line_vector.is_some() {
            // Line still detected - increment counter but stay in this state
            self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter + 1);
            Some(-original_vector)
        } else {
            // Line lost - use timeout counter to decide next state
            if counter > COUNTER_TIMEOUT_LIMIT {
                // Timeout exceeded - return to ground
                self.state = AdcState::OnGround;
                None
            } else {
                // Still within timeout - keep using original position
                self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter + 1);
                Some(-original_vector)
            }
        }
    }
    
    fn is_angle_outside_range(&self, angle: f32, min_angle: f32, max_angle: f32) -> bool {
        !is_angle_in_range(angle, min_angle, max_angle)
    }
}

pub fn generate_adc_vec<T>(sin: &mut [T], cos: &mut [T], offset: f32, one_angle: f32, mul: f32)
where
    f32: AsPrimitive<T>,
    T: Num + Copy + 'static,
{
    for i in 0..sin.len() {
        sin[i] = (libm::sinf(i as f32 * one_angle + offset) * mul).as_();
        cos[i] = (libm::cosf(i as f32 * one_angle + offset) * mul).as_();
    }
}

pub fn calculate_adc_vec<T>(adc: &[T], adc_sin: &[T], adc_cos: &[T], _mul: T) -> (f32, f32, T)
where
    T: Num + Copy + 'static + AsPrimitive<f32> + PartialOrd,
    f32: AsPrimitive<T>,
{
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;
    let mut max_adc: T = T::zero();

    for i in 0..adc.len() {
        if adc[i] > max_adc {
            max_adc = adc[i];
        }
        sum_x = sum_x + (adc_cos[i] * adc[i]).as_();
        sum_y = sum_y + (adc_sin[i] * adc[i]).as_();
    }

    let norm = libm::sqrtf(
        libm::powf(sum_x / adc.len() as f32, 2.0) + libm::powf(sum_y / adc.len() as f32, 2.0),
    );

    (
        sum_x / adc.len() as f32 / norm,
        sum_y / adc.len() as f32 / norm,
        max_adc,
    )
}

pub fn calculate_line_vec_with_threshold(
    adc: &[f32],
    adc_sin: &[f32],
    adc_cos: &[f32],
    threshold: f32,
) -> Option<Vector2> {
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;
    let mut over_threshold = false;

    for i in 0..adc.len() {
        if adc[i] > threshold {
            sum_x = sum_x + adc_cos[i];
            sum_y = sum_y + adc_sin[i];
            over_threshold = true;
        }
    }

    if !over_threshold {
        return None;
    }

    let norm = libm::sqrtf(libm::powf(sum_x, 2.0) + libm::powf(sum_y, 2.0));
    Some(Vector2::new(sum_x / norm, sum_y / norm))
}

pub fn is_angle_in_range(angle: f32, a: f32, b: f32) -> bool {
    let normalize = |x: f32| -> f32 {
        let mut x = x;
        while x > PI {
            x -= 2.0 * PI;
        }
        while x <= -PI {
            x += 2.0 * PI;
        }
        x
    };

    let a = normalize(a);
    let b = normalize(b);
    let angle = normalize(angle);

    if libm::fabsf(b - a) <= PI {
        a <= angle && angle <= b
    } else {
        !(b <= angle && angle <= a)
    }
}
