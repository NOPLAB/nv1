use core::f32::consts::PI;
use embassy_stm32::{
    peripherals,
    time::Hertz,
    timer::{simple_pwm::SimplePwm, Ch1Dma, GeneralInstance4Channel},
};
use embassy_time::{Duration, Timer};
use rgb::RGB8;

use crate::constants::{LED_COUNT, SPREAD_PATTERN};

const NEO_PIXEL_NUM: usize = 32;

#[derive(Debug, Clone, Copy)]
pub struct NeoPixelData {
    pub jetson_connecting: bool,
    pub pause: bool,
    pub ball_dir: f32,
}

pub struct NeoPixelPwm<T>
where
    T: GeneralInstance4Channel,
{
    pwm: SimplePwm<'static, T>,
    one_duty: u16,
    zero_duty: u16,
    brightness: u8,
}

impl<T> NeoPixelPwm<T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm: SimplePwm<'static, T>, pwm_hz: Hertz) -> Self {
        let max_duty = pwm.max_duty_cycle();

        let one_duty: u16 = max_duty / 2;
        let zero_duty: u16 = (max_duty as f32 * (260.0e-6 * (pwm_hz.0 / 1000) as f32)) as u16;

        Self {
            pwm,
            one_duty,
            zero_duty,
            brightness: 45,
        }
    }

    async fn write(&mut self, dma: &mut impl Ch1Dma<T>, colors: &mut [RGB8]) {
        let mut duty = [0u16; 24 * NEO_PIXEL_NUM];

        for (n, color) in colors.iter_mut().enumerate() {
            for i in 0..8 {
                if color.g & 0b1000_0000 != 0 {
                    duty[n * 24 + i] = self.one_duty;
                } else {
                    duty[n * 24 + i] = self.zero_duty;
                }

                if color.r & 0b1000_0000 != 0 {
                    duty[n * 24 + i + 8] = self.one_duty;
                } else {
                    duty[n * 24 + i + 8] = self.zero_duty;
                }

                if color.b & 0b1000_0000 != 0 {
                    duty[n * 24 + i + 16] = self.one_duty;
                } else {
                    duty[n * 24 + i + 16] = self.zero_duty;
                }

                color.g <<= 1;
                color.r <<= 1;
                color.b <<= 1;
            }
        }

        self.pwm.ch1().enable();
        self.pwm.waveform_ch1(dma, &duty).await;
        self.pwm.ch1().disable();
    }

    pub async fn set_colors(&mut self, dma: &mut impl Ch1Dma<T>, colors: &mut [RGB8]) {
        let angle = 90.0 - self.brightness as f32;
        let angle = angle * core::f32::consts::PI / 180.0;

        let tan_angle = libm::tanf(angle);

        for color in &mut *colors {
            let r = color.r as f32;
            let g = color.g as f32;
            let b = color.b as f32;

            let r = r * tan_angle;
            let g = g * tan_angle;
            let b = b * tan_angle;

            color.r = r as u8;
            color.g = g as u8;
            color.b = b as u8;
        }

        self.write(dma, colors).await;
    }

    #[allow(dead_code)]
    pub fn set_brightness(&mut self, brightness: u8) {
        if brightness > 45 {
            self.brightness = 45;
        } else {
            self.brightness = brightness;
        }
    }
}

#[embassy_executor::task]
pub async fn neo_pixel_task(
    mut neo_pixel: NeoPixelPwm<peripherals::TIM4>,
    dma: &'static mut peripherals::DMA1_CH0,
) {
    let mut neo_pixel_data = [RGB8::default(); LED_COUNT];
    for c in neo_pixel_data.iter_mut() {
        *c = RGB8 { r: 0, g: 0, b: 0 };
    }

    let mut loop_count = 0;
    loop {
        let neo_pixel_info = crate::communication::G_NEO_PIXEL_DATA.lock().await.clone();

        if neo_pixel_info.pause {
            let color = if neo_pixel_info.jetson_connecting {
                RGB8 { r: 0, g: 255, b: 0 }
            } else {
                RGB8 { r: 255, g: 0, b: 0 }
            };

            let base_index = loop_count % LED_COUNT;
            let spread = SPREAD_PATTERN[loop_count % 32];
            for j in 0..3 {
                let offset = spread * (j as isize - 1) as usize;
                let index = (base_index + offset) % LED_COUNT;
                neo_pixel_data[index] = color;
            }

            neo_pixel.set_colors(dma, &mut neo_pixel_data).await;
            Timer::after(Duration::from_millis(30)).await;
        } else {
            let ball_dir = neo_pixel_info.ball_dir;
            let ball_dir = -ball_dir + (PI / 2.0);
            let ball_dir = (ball_dir + 2.0 * PI) % (2.0 * PI);
            let ball_dir = (ball_dir / (2.0 * PI)) * (LED_COUNT as f32);

            for i in 0..LED_COUNT {
                let color = if i == ball_dir as usize {
                    RGB8 {
                        r: 255,
                        g: 255,
                        b: 255,
                    }
                } else {
                    RGB8 { r: 0, g: 0, b: 0 }
                };
                neo_pixel_data[i] = color;
            }

            Timer::after(Duration::from_millis(100)).await;
        }

        loop_count = (loop_count + 1) % LED_COUNT;
    }
}
