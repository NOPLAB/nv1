use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals;
use embassy_stm32::Peri;

use crate::constants::*;
use crate::types::Vector2;

pub use nv1_hub_core::sensor_math::{
    calculate_adc_vec, calculate_line_vec_with_threshold, generate_adc_vec,
};

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
        adc: Adc<'static, peripherals::ADC1>,
        line_s0: Output<'static>,
        line_s1: Output<'static>,
        line_s2: Output<'static>,
        line_s3: Output<'static>,
        ir_s0: Output<'static>,
        ir_s1: Output<'static>,
        ir_s2: Output<'static>,
        ir_s3: Output<'static>,
    ) -> Self {
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
        pc0: &mut Peri<'static, peripherals::PC0>,
        pc1: &mut Peri<'static, peripherals::PC1>,
        pc2: &mut Peri<'static, peripherals::PC2>,
        pc3: &mut Peri<'static, peripherals::PC3>,
    ) -> SensorReadings {
        use embassy_stm32::adc::SampleTime;
        let sample_time = SampleTime::CYCLES56;

        let mut adc_line_raw = [0u16; LINE_SENSORS_COUNT];
        let mut adc_ir_raw = [0u16; IR_SENSORS_COUNT];
        let adc_have_ball = self.adc.blocking_read(pc3, sample_time);

        for i in 0..IR_SENSORS_COUNT {
            self.set_multiplexer_pins(i);

            adc_line_raw[i] = self.adc.blocking_read(pc0, sample_time);
            adc_line_raw[i + IR_SENSORS_COUNT] = self.adc.blocking_read(pc1, sample_time);
            adc_ir_raw[i] = self.adc.blocking_read(pc2, sample_time);
        }

        let mut adc_line = [0.0f32; LINE_SENSORS_COUNT];
        for (dst, &src) in adc_line.iter_mut().zip(adc_line_raw.iter()) {
            *dst = src as f32 / ADC_RESOLUTION;
        }

        let mut adc_ir = [0.0f32; IR_SENSORS_COUNT];
        for (dst, &src) in adc_ir.iter_mut().zip(adc_ir_raw.iter()) {
            *dst = (4096 - src) as f32 / ADC_RESOLUTION;
        }

        let (ir_x, ir_y, _ir_strength) = calculate_adc_vec(
            &adc_ir,
            &self.ir_ball_sensor.sin_values,
            &self.ir_ball_sensor.cos_values,
            1.0,
        );
        let ir_angle = libm::atan2f(ir_y, ir_x);
        let adc_line_max = adc_line.iter().cloned().reduce(f32::max).unwrap_or(0.0);

        SensorReadings {
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
    pub ir_angle: f32,
    pub ir_position: Vector2,
    pub adc_have_ball: u16,
    pub adc_line_max: f32,
    pub adc_line: [f32; LINE_SENSORS_COUNT],
}
