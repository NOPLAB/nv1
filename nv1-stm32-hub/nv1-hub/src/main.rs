#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod communication;
mod constants;
mod fmt;
mod hardware;
mod main_loop;
mod motor_controller;
mod neo_pixel;
mod omni;
mod sensors;
mod settings;
mod ui_system;

extern crate alloc;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use crate::communication::*;
use crate::constants::*;
use crate::sensors::*;
use crate::settings::*;
use crate::ui_system::UISystem;

use core::mem::MaybeUninit;
use core::{borrow::Borrow, cell::RefCell};

use alloc::rc::Rc;

use defmt::error;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::flash::Flash;

use embassy_stm32::mode;

use embassy_stm32::{
    bind_interrupts,
    gpio::Pull,
    i2c::{self, I2c},
    peripherals,
    usart::{self, Uart},
};
use embassy_time::{with_timeout, Duration, Timer};

use neo_pixel::NeoPixelPwm;
use nv1_hub_ui::Event;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::I2CInterface;
use ssd1306::{size::DisplaySize128x64, I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    USART6 => usart::InterruptHandler<peripherals::USART6>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize static heap
    {
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    // Configure and initialize hardware
    let config = hardware::configure_clocks();
    let mut p = embassy_stm32::init(config);

    let hardware_config = hardware::HardwareConfig::default();
    let uart_pins = hardware::initialize_uarts(
        p.USART3,
        p.PC5,
        p.PB10,
        p.DMA1_CH3,
        p.DMA1_CH1,
        p.UART4,
        p.PC11,
        p.PC10,
        p.DMA1_CH4,
        p.DMA1_CH2,
        p.USART6,
        p.PC7,
        p.PC6,
        p.DMA2_CH6,
        p.DMA2_CH1,
        p.PA0,
        &hardware_config,
    );

    // Reset BNO08x sensor - handle timing that was moved out of hardware init
    {
        use embassy_time::{Duration, Timer};
        Timer::after(Duration::from_millis(10)).await;
        // GPIO reset is handled inside initialize_uarts
        Timer::after(Duration::from_millis(100)).await;
    }

    // Initialize sensor system
    let (adc, line_s0, line_s1, line_s2, line_s3, ir_s0, ir_s1, ir_s2, ir_s3) =
        hardware::initialize_adc_and_gpio(
            p.ADC1, p.PB12, p.PB13, p.PB14, p.PB15, p.PB0, p.PB1, p.PB4, p.PB5,
        );

    let mut sensor_system = SensorSystem::new(
        adc, line_s0, line_s1, line_s2, line_s3, ir_s0, ir_s1, ir_s2, ir_s3,
    );

    // Initialize flash and settings
    let f = Rc::new(RefCell::new(Flash::new_blocking(p.FLASH)));
    let mut settings_data = flash_read(&mut f.clone().borrow_mut()).unwrap_or(Settings::DEFAULT);
    validate_and_fix_settings(&mut settings_data, &mut f.clone().borrow_mut()).unwrap();
    let settings = Rc::new(RefCell::new(settings_data));

    // Initialize UI system
    let gpio_ui_toggle = ExtiInput::new(p.PC12, p.EXTI12, Pull::None);
    let gpio_ui_up = ExtiInput::new(p.PC13, p.EXTI13, Pull::None);
    let gpio_ui_down = ExtiInput::new(p.PC14, p.EXTI14, Pull::None);
    let gpio_ui_enter = ExtiInput::new(p.PC15, p.EXTI15, Pull::None);

    let ssd1306_i2c = hardware::initialize_i2c(p.I2C3, p.PA8, p.PC9);
    let ssd1306_interface = I2CDisplayInterface::new(ssd1306_i2c);
    let ssd1306 = Ssd1306::new(
        ssd1306_interface,
        DisplaySize128x64,
        ssd1306::prelude::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    static SSD1306: StaticCell<
        Ssd1306<
            I2CInterface<I2c<'static, embassy_stm32::mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
    > = StaticCell::new();

    let ssd1306: &'static mut Ssd1306<
        I2CInterface<I2c<'static, embassy_stm32::mode::Blocking>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    > = SSD1306.init(ssd1306);

    let mut ssd1306_init_success = UISystem::try_initialize_display(ssd1306);

    // Create and configure UI elements
    let (mut ui, shutdown, reboot, value_line, value_have_ball) =
        ui_system::UISystem::create_ui_system(ssd1306, settings.clone(), f.clone());

    let display = ui.update(&Event::None);
    if ssd1306_init_success {
        ssd1306_init_success = display.flush().is_ok();
    }

    // Initialize NeoPixel
    let neo_pixel_pwm = hardware::initialize_neo_pixel_pwm(p.TIM4, p.PB6, &hardware_config);
    let neo_pixel = NeoPixelPwm::new(neo_pixel_pwm, hardware_config.neo_pixel_pwm_hz);
    static NEO_PIXEL_DMA: StaticCell<peripherals::DMA1_CH0> = StaticCell::new();
    let neo_pixel_dma: &'static mut peripherals::DMA1_CH0 = NEO_PIXEL_DMA.init(p.DMA1_CH0);

    // Initialize motor controller
    let motor_controller = motor_controller::MotorController::new();

    // Spawn communication tasks
    spawner.must_spawn(bno08x_task(uart_pins.uart_bno));
    spawner.must_spawn(uart_jetson_task(uart_pins.uart_jetson));

    // Spawn UI task if display initialized successfully
    if ssd1306_init_success {
        spawner.must_spawn(ui_system::ui_task(
            ui,
            gpio_ui_up,
            gpio_ui_down,
            gpio_ui_enter,
            shutdown.clone(),
            reboot.clone(),
        ));
    }

    // Spawn NeoPixel task
    spawner.must_spawn(neo_pixel::neo_pixel_task(neo_pixel, neo_pixel_dma));

    // Create main loop context and run
    let mut main_loop_context = main_loop::MainLoopContext::new(
        motor_controller,
        settings,
        value_line,
        value_have_ball,
        gpio_ui_toggle,
    );

    main_loop_context
        .run_main_loop(
            sensor_system,
            uart_pins.uart_md,
            (p.PC0, p.PC1, p.PC2, p.PC3),
        )
        .await;
}

#[embassy_executor::task]
async fn bno08x_task(mut uart: Uart<'static, mode::Async>) {
    let mut bno08x_buf = [0u8; 19];

    let (mut processor, mut parser) = match bno08x_rvc::create(G_BB.borrow()) {
        Ok((proc, pars)) => (proc, pars),
        Err(_e) => {
            error!("Can't create bno08x-rvc");
            loop {}
        }
    };

    loop {
        let _ = uart.read(&mut bno08x_buf).await;
        processor.process_slice(&bno08x_buf).unwrap();
        let _ = parser.worker(|frame| {
            let yaw = embassy_futures::block_on(G_YAW.lock());
            yaw.replace(-(frame.as_pretty_frame().yaw.to_radians()));
        });
    }
}

#[embassy_executor::task]
async fn uart_jetson_task(uart: Uart<'static, mode::Async>) {
    let (mut uart_tx, uart_rx) = uart.split();

    let mut dma_buf = [0u8; 128];
    let mut uart_rx = uart_rx.into_ring_buffered(&mut dma_buf);

    uart_rx.start_uart();

    let mut timeout_cnt = 9999;
    loop {
        let mut byte = [0u8; 1];
        let mut msg_with_cobs = [0u8; 64];
        let mut msg_cnt = 0;
        loop {
            let timeout_res = with_timeout(Duration::from_millis(1), uart_rx.read(&mut byte)).await;
            match timeout_res {
                Ok(receive_res) => match receive_res {
                    Ok(_size) => {
                        msg_with_cobs[msg_cnt] = byte[0];
                        msg_cnt += 1;

                        if byte[0] == 0 {
                            break;
                        }

                        timeout_cnt = 0;
                    }
                    Err(err) => {
                        error!("[UART Jetson] read error: {:?}", err);
                        uart_rx.start_uart();
                    }
                },
                Err(_) => {
                    timeout_cnt += 1;

                    break;
                }
            }
        }
        match postcard::from_bytes_cobs::<nv1_msg::hub::ToHub>(&mut msg_with_cobs) {
            Ok(msg) => {
                // info!("Linear X: {}", msg.vel.x);
                // info!("Linear Y: {}", msg.vel.y);
                // info!("Angular Z: {}", msg.vel.angle);

                G_JETSON_RX.lock().await.replace(msg);

                G_NEO_PIXEL_DATA.lock().await.jetson_connecting = true;
            }
            Err(_) => {
                // error!("[UART Jetson] postcard decode error");
            }
        };

        Timer::after(Duration::from_millis(2)).await;

        let msg = G_JETSON_TX.lock().await.take();
        match postcard::to_vec_cobs::<nv1_msg::hub::ToJetson, 64>(&msg) {
            Ok(msg_with_cobs) => {
                match uart_tx.write(&msg_with_cobs).await {
                    Ok(_) => {
                        // info!("[UART Jetson] sent data, len: {}", msg_with_cobs.len());
                    }
                    Err(e) => {
                        error!("[UART Jetson] write error: {:?}", e);
                    }
                };
            }
            Err(_) => {
                error!("[UART Jetson] postcard encode error");
            }
        }

        if timeout_cnt > 100 {
            // error!("[UART Jetson] timeout");

            G_JETSON_RX.lock().await.replace(nv1_msg::hub::ToHub {
                vel: nv1_msg::hub::Movement {
                    x: 0.0,
                    y: 0.0,
                    angle: 0.0,
                },
                ..Default::default()
            });

            G_NEO_PIXEL_DATA.lock().await.jetson_connecting = false;
        } else {
            G_NEO_PIXEL_DATA.lock().await.jetson_connecting = true;
        }

        Timer::after(Duration::from_millis(2)).await;
    }
}
