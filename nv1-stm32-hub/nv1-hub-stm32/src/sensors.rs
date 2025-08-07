use core::f32::consts::PI;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals;
use num_traits::{AsPrimitive, Num};

use crate::constants::*;

pub enum AdcState {
    OnGround,
    OnLine(f32, f32, f32, u32),
    OutOfLineOverCenter(f32, f32, f32, u32),
}

pub struct SensorSystem {
    pub adc: Adc<'static, peripherals::ADC1>,
    pub line_s0: Output<'static>,
    pub line_s1: Output<'static>,
    pub line_s2: Output<'static>,
    pub line_s3: Output<'static>,
    pub ir_s0: Output<'static>,
    pub ir_s1: Output<'static>,
    pub ir_s2: Output<'static>,
    pub ir_s3: Output<'static>,
    pub adc_line_sin: [f32; LINE_SENSORS_COUNT],
    pub adc_line_cos: [f32; LINE_SENSORS_COUNT],
    pub adc_ir_sin: [f32; IR_SENSORS_COUNT],
    pub adc_ir_cos: [f32; IR_SENSORS_COUNT],
}

impl SensorSystem {
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
        adc.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES3);

        let mut adc_line_sin = [0.0_f32; LINE_SENSORS_COUNT];
        let mut adc_line_cos = [0.0_f32; LINE_SENSORS_COUNT];
        generate_adc_vec(
            &mut adc_line_sin,
            &mut adc_line_cos,
            90.0_f32.to_radians(),
            -(360.0_f32 / LINE_SENSORS_COUNT as f32).to_radians(),
            1.0,
        );

        let mut adc_ir_sin = [0.0_f32; IR_SENSORS_COUNT];
        let mut adc_ir_cos = [0.0_f32; IR_SENSORS_COUNT];
        generate_adc_vec(
            &mut adc_ir_sin,
            &mut adc_ir_cos,
            90_f32.to_radians(),
            -(360.0_f32 / IR_SENSORS_COUNT as f32).to_radians(),
            1.0,
        );

        Self {
            adc,
            line_s0,
            line_s1,
            line_s2,
            line_s3,
            ir_s0,
            ir_s1,
            ir_s2,
            ir_s3,
            adc_line_sin,
            adc_line_cos,
            adc_ir_sin,
            adc_ir_cos,
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
            &self.adc_line_sin,
            &self.adc_line_cos,
            0.1, // Default threshold - will be overridden by caller
        );

        let (ir_x, ir_y, _ir_strength) = calculate_adc_vec(&adc_ir, &self.adc_ir_sin, &self.adc_ir_cos, 1.0);
        let ir_angle = libm::atan2f(ir_y, ir_x);
        let adc_line_max = adc_line.iter().cloned().reduce(f32::max).unwrap_or(0.0);

        SensorReadings {
            line_vector,
            ir_angle,
            ir_x,
            ir_y,
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

        set_pin_state(&mut self.line_s0, 0b0001);
        set_pin_state(&mut self.line_s1, 0b0010);
        set_pin_state(&mut self.line_s2, 0b0100);
        set_pin_state(&mut self.line_s3, 0b1000);

        set_pin_state(&mut self.ir_s0, 0b0001);
        set_pin_state(&mut self.ir_s1, 0b0010);
        set_pin_state(&mut self.ir_s2, 0b0100);
        set_pin_state(&mut self.ir_s3, 0b1000);
    }
}

pub struct SensorReadings {
    pub line_vector: Option<(f32, f32)>,
    pub ir_angle: f32,
    pub ir_x: f32,
    pub ir_y: f32,
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

    pub fn process_line(&mut self, line_vector: Option<(f32, f32)>, _threshold: f32) -> Option<(f32, f32)> {
        match self.state {
            AdcState::OnGround => {
                if let Some((x, y)) = line_vector {
                    let now_angle = libm::atan2f(y, x);
                    self.state = AdcState::OnLine(now_angle, x, y, 0);
                    Some((-x, -y))
                } else {
                    self.state = AdcState::OnGround;
                    None
                }
            }
            AdcState::OnLine(first_angle, first_x, first_y, counter) => {
                if let Some((x, y)) = line_vector {
                    let now_angle = libm::atan2f(y, x);

                    if !is_angle_in_range(
                        now_angle,
                        first_angle - LINE_OVER_CENTER_THRESHOLD / 2.0,
                        first_angle + LINE_OVER_CENTER_THRESHOLD / 2.0,
                    ) {
                        self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, 0);
                        Some((-x, -y))
                    } else {
                        self.state = AdcState::OnLine(first_angle, first_x, first_y, 0);
                        Some((-x, -y))
                    }
                } else {
                    if counter > 100 {
                        self.state = AdcState::OnGround;
                        None
                    } else {
                        self.state = AdcState::OnLine(first_angle, first_x, first_y, counter + 1);
                        Some((-first_x, -first_y))
                    }
                }
            }
            AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter) => {
                if line_vector.is_some() {
                    self.state = AdcState::OutOfLineOverCenter(first_angle, first_x, first_y, counter + 1);
                    Some((-first_x, -first_y))
                } else {
                    if counter > 100 {
                        self.state = AdcState::OnGround;
                        None
                    } else {
                        self.state = AdcState::OutOfLineOverCenter(
                            first_angle,
                            first_x,
                            first_y,
                            counter + 1,
                        );
                        Some((-first_x, -first_y))
                    }
                }
            }
        }
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

pub fn calculate_line_vec(
    adc: &[f32],
    adc_sin: &[f32],
    adc_cos: &[f32],
) -> Option<(f32, f32)> {
    let mut sum_x: f32 = 0.0;
    let mut sum_y: f32 = 0.0;

    for i in 0..adc.len() {
        sum_x = sum_x + adc_cos[i];
        sum_y = sum_y + adc_sin[i];
    }

    let norm = libm::sqrtf(libm::powf(sum_x, 2.0) + libm::powf(sum_y, 2.0));
    if norm == 0.0 {
        None
    } else {
        Some((sum_x / norm, sum_y / norm))
    }
}

pub fn calculate_line_vec_with_threshold(
    adc: &[f32],
    adc_sin: &[f32],
    adc_cos: &[f32],
    threshold: f32,
) -> Option<(f32, f32)> {
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
    Some((sum_x / norm, sum_y / norm))
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