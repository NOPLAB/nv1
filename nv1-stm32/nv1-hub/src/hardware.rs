use crate::constants::*;
use embassy_stm32::Peri;
use embassy_stm32::{
    adc::Adc,
    gpio::{Level, Output, Speed},
    i2c::{self, I2c, Master},
    peripherals,
    time::Hertz,
    timer::{low_level::CountingMode, simple_pwm::SimplePwm},
    usart::{self, Uart},
};
use embassy_time::Duration;

pub struct HardwareConfig {
    pub uart_jetson_baudrate: u32,
    pub uart_md_baudrate: u32,
    pub neo_pixel_pwm_hz: Hertz,
}

impl Default for HardwareConfig {
    fn default() -> Self {
        Self {
            uart_jetson_baudrate: UART_JETSON_BAUDRATE,
            uart_md_baudrate: UART_MD_BAUDRATE,
            neo_pixel_pwm_hz: Hertz::khz(500),
        }
    }
}

pub fn configure_clocks() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc;

        config.rcc.hse = Some(rcc::Hse {
            freq: Hertz(20_000_000),
            mode: rcc::HseMode::Oscillator,
        });

        config.rcc.pll_src = rcc::PllSource::HSE;
        config.rcc.pll = Some(rcc::Pll {
            prediv: rcc::PllPreDiv::DIV16,
            mul: rcc::PllMul::MUL288,
            divp: Some(rcc::PllPDiv::DIV2),
            divq: None,
            divr: Some(rcc::PllRDiv::DIV2),
        });

        config.rcc.sys = rcc::Sysclk::PLL1_P;

        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }
    config
}

pub struct UartPins {
    pub uart_jetson: Uart<'static, embassy_stm32::mode::Async>,
    pub uart_md: Uart<'static, embassy_stm32::mode::Async>,
    pub uart_bno: Uart<'static, embassy_stm32::mode::Async>,
}

#[allow(clippy::too_many_arguments)]
pub fn initialize_uarts(
    usart3: Peri<'static, peripherals::USART3>,
    pc5: Peri<'static, peripherals::PC5>,
    pb10: Peri<'static, peripherals::PB10>,
    dma1_ch3: Peri<'static, peripherals::DMA1_CH3>,
    dma1_ch1: Peri<'static, peripherals::DMA1_CH1>,
    uart4: Peri<'static, peripherals::UART4>,
    pc11: Peri<'static, peripherals::PC11>,
    pc10: Peri<'static, peripherals::PC10>,
    dma1_ch4: Peri<'static, peripherals::DMA1_CH4>,
    dma1_ch2: Peri<'static, peripherals::DMA1_CH2>,
    usart6: Peri<'static, peripherals::USART6>,
    pc7: Peri<'static, peripherals::PC7>,
    pc6: Peri<'static, peripherals::PC6>,
    dma2_ch6: Peri<'static, peripherals::DMA2_CH6>,
    dma2_ch1: Peri<'static, peripherals::DMA2_CH1>,
    config: &HardwareConfig,
) -> UartPins {
    use crate::Irqs;

    // UART for Jetson communication
    let mut uart_jetson_config = usart::Config::default();
    uart_jetson_config.baudrate = config.uart_jetson_baudrate;
    let uart_jetson = Uart::new(
        usart3,
        pc5,
        pb10,
        dma1_ch3,
        dma1_ch1,
        Irqs,
        uart_jetson_config,
    )
    .unwrap();

    // UART for Motor Driver communication.
    //
    // Async with DMA1 streams 4 (TX) / 2 (RX). `write().await` is intentionally
    // the main loop's yield point at steady state: the loop's only other awaits
    // are mutex locks that resolve immediately when uncontended, so without an
    // I/O await here the executor never gets to poll the NeoPixel / UI /
    // BNO08x tasks and they starve. See main_loop.rs for the matching call.
    let mut uart_md_config = usart::Config::default();
    uart_md_config.baudrate = config.uart_md_baudrate;
    let uart_md = Uart::new(uart4, pc11, pc10, dma1_ch4, dma1_ch2, Irqs, uart_md_config).unwrap();

    // UART for BNO08x sensor
    let mut uart_bno_config = usart::Config::default();
    uart_bno_config.baudrate = bno08x_rvc::BNO08X_UART_RVC_BAUD_RATE;
    let uart_bno = Uart::new(usart6, pc7, pc6, dma2_ch6, dma2_ch1, Irqs, uart_bno_config).unwrap();

    UartPins {
        uart_jetson,
        uart_md,
        uart_bno,
    }
}

/// Drive the BNO08x nRESET line low for 10ms then release it high, and wait
/// 100ms for the IMU to finish booting before we start reading from its UART.
/// The returned [`Output`] must be kept alive so PA0 stays driven high.
pub async fn reset_bno08x(pa0: Peri<'static, peripherals::PA0>) -> Output<'static> {
    use embassy_time::Timer;

    let mut gpio_reset = Output::new(pa0, Level::High, Speed::Low);
    gpio_reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    gpio_reset.set_high();
    Timer::after(Duration::from_millis(100)).await;
    gpio_reset
}

#[allow(clippy::too_many_arguments)]
pub fn initialize_adc_and_gpio(
    adc1: Peri<'static, peripherals::ADC1>,
    pb12: Peri<'static, peripherals::PB12>,
    pb13: Peri<'static, peripherals::PB13>,
    pb14: Peri<'static, peripherals::PB14>,
    pb15: Peri<'static, peripherals::PB15>,
    pb0: Peri<'static, peripherals::PB0>,
    pb1: Peri<'static, peripherals::PB1>,
    pb4: Peri<'static, peripherals::PB4>,
    pb5: Peri<'static, peripherals::PB5>,
) -> (
    Adc<'static, peripherals::ADC1>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
    Output<'static>,
) {
    let adc = Adc::new(adc1);
    let line_s0 = Output::new(pb12, Level::Low, Speed::VeryHigh);
    let line_s1 = Output::new(pb13, Level::Low, Speed::VeryHigh);
    let line_s2 = Output::new(pb14, Level::Low, Speed::VeryHigh);
    let line_s3 = Output::new(pb15, Level::Low, Speed::VeryHigh);
    let ir_s0 = Output::new(pb0, Level::Low, Speed::VeryHigh);
    let ir_s1 = Output::new(pb1, Level::Low, Speed::VeryHigh);
    let ir_s2 = Output::new(pb4, Level::Low, Speed::VeryHigh);
    let ir_s3 = Output::new(pb5, Level::Low, Speed::VeryHigh);

    (
        adc, line_s0, line_s1, line_s2, line_s3, ir_s0, ir_s1, ir_s2, ir_s3,
    )
}

pub fn initialize_i2c(
    i2c3: Peri<'static, peripherals::I2C3>,
    pa8: Peri<'static, peripherals::PA8>,
    pc9: Peri<'static, peripherals::PC9>,
) -> I2c<'static, embassy_stm32::mode::Blocking, Master> {
    let mut config = i2c::Config::default();
    config.frequency = Hertz::khz(200);
    config.timeout = Duration::from_millis(10);
    // Blocking I2C, not async + DMA, even though it stalls the executor for
    // ~50 ms on each OLED framebuffer flush. The reason: embassy-stm32 v1
    // (STM32F4) async I2C hangs on init (interrupt storm without progress,
    // see embassy issue #2372) so async simply doesn't work on this chip.
    //
    // Using blocking here also frees DMA1 streams 4 and 2 for the MD UART,
    // which *requires* DMA to be the main loop's yield point — see the
    // `uart_md` comment in `initialize_uarts` above.
    I2c::new_blocking(i2c3, pa8, pc9, config)
}

pub fn initialize_neo_pixel_pwm(
    tim4: Peri<'static, peripherals::TIM4>,
    pb6: Peri<'static, peripherals::PB6>,
    config: &HardwareConfig,
) -> SimplePwm<'static, peripherals::TIM4> {
    SimplePwm::new(
        tim4,
        Some(embassy_stm32::timer::simple_pwm::PwmPin::new(
            pb6,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        None,
        None,
        None,
        config.neo_pixel_pwm_hz,
        CountingMode::EdgeAlignedUp,
    )
}
