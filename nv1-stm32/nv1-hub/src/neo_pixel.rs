use core::f32::consts::PI;
use embassy_stm32::Peri;
use embassy_stm32::{
    peripherals,
    time::Hertz,
    timer::{simple_pwm::SimplePwm, Ch1, Channel, Dma as TimerDma, GeneralInstance4Channel},
};
use embassy_time::{Duration, Timer};
use rgb::RGB8;

use crate::constants::{LED_COUNT, SPREAD_PATTERN};
use nv1_hub_core::neo_pixel::{
    apply_brightness, colors_to_duty_buffer, compute_duties, DutyValues, BITS_PER_LED,
};

pub use nv1_hub_core::neo_pixel::NeoPixelData;

const NEO_PIXEL_NUM: usize = 32;

pub struct NeoPixelPwm<T>
where
    T: GeneralInstance4Channel,
{
    pwm: SimplePwm<'static, T>,
    duties: DutyValues,
    brightness: u8,
}

impl<T> NeoPixelPwm<T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm: SimplePwm<'static, T>, pwm_hz: Hertz) -> Self {
        let max_duty = pwm.max_duty_cycle() as u16;
        let duties = compute_duties(pwm_hz.0, max_duty);
        Self {
            pwm,
            duties,
            brightness: 45,
        }
    }

    async fn write<D: TimerDma<T, Ch1>>(&mut self, dma: &mut Peri<'static, D>, colors: &[RGB8])
    where
        crate::Irqs: embassy_stm32::interrupt::typelevel::Binding<
            D::Interrupt,
            embassy_stm32::dma::InterruptHandler<D>,
        >,
    {
        use crate::Irqs;

        let mut duty = [0u16; BITS_PER_LED * NEO_PIXEL_NUM];
        colors_to_duty_buffer(colors, self.duties, &mut duty);

        self.pwm.ch1().enable();
        self.pwm
            .waveform::<Ch1, _, _>(dma.reborrow(), Irqs, Channel::Ch1, &duty)
            .await;
        // embassy-stm32 0.6's `waveform` leaves CCR1 holding the final duty
        // value, so the PWM keeps emitting that pulse width until the channel
        // is disabled. Forcing CCR1 to 0 here gives a clean low for the LED
        // reset gap (matches embassy-stm32 0.2's `waveform_ch1` behavior).
        self.pwm.ch1().set_duty_cycle(0);
        self.pwm.ch1().disable();
    }

    pub async fn set_colors<D: TimerDma<T, Ch1>>(
        &mut self,
        dma: &mut Peri<'static, D>,
        colors: &mut [RGB8],
    ) where
        crate::Irqs: embassy_stm32::interrupt::typelevel::Binding<
            D::Interrupt,
            embassy_stm32::dma::InterruptHandler<D>,
        >,
    {
        apply_brightness(colors, self.brightness);
        self.write(dma, colors).await;
    }

    #[allow(dead_code)]
    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness.min(45);
    }
}

#[embassy_executor::task]
pub async fn neo_pixel_task(
    mut neo_pixel: NeoPixelPwm<peripherals::TIM4>,
    mut dma: Peri<'static, peripherals::DMA1_CH0>,
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

            neo_pixel.set_colors(&mut dma, &mut neo_pixel_data).await;
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
