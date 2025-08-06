#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

mod constants;
mod settings;
mod sensors;
mod communication;
mod ui_system;
mod fmt;
mod neo_pixel;
mod omni;

extern crate alloc;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use crate::constants::*;
use crate::settings::*;
use crate::sensors::*;
use crate::communication::*;
use crate::ui_system::UISystem;

use core::f32::consts::PI;
use core::{borrow::Borrow, cell::RefCell};
use core::mem::MaybeUninit;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec::Vec;

use defmt::error;
use embassy_executor::Spawner;
use embassy_futures::select::select3;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::flash::Flash;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::mode;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{
    adc::Adc,
    bind_interrupts,
    gpio::{Level, Output, Pull},
    i2c::{self, I2c},
    peripherals,
    time::Hertz,
    usart::{self, Uart},
};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_graphics::prelude::{Point, Size};
use fmt::info;
use neo_pixel::NeoPixelPwm;
use nv1_hub_ui::elements;
use nv1_hub_ui::elements::Element;
use nv1_hub_ui::elements::{Slider, Text, Value};
use nv1_hub_ui::menu::{Menu, RobotStatusMenu, RobotStatusMenuOption};
use nv1_hub_ui::{
    elements::Button,
    menu::{ListMenu, ListMenuOption},
    Event, HubUI,
};
use nv1_hub_ui::{menus, EventKey, HubUIOption};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::I2CInterface;
use ssd1306::{mode::DisplayConfig, size::DisplaySize128x64, I2CDisplayInterface, Ssd1306};
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
    // initialize static heap
    {
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

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

    // initialize peripherals
    let mut p = embassy_stm32::init(config);

    info!("Hello, world!");

    // initialize UARTs
    let mut uart_jetson_config = usart::Config::default();
    uart_jetson_config.baudrate = UART_JETSON_BAUDRATE;
    let uart_jetson = Uart::new(
        p.USART3,
        p.PC5,
        p.PB10,
        Irqs,
        p.DMA1_CH3,
        p.DMA1_CH1,
        uart_jetson_config,
    )
    .unwrap();

    let mut uart_md_config = usart::Config::default();
    uart_md_config.baudrate = UART_MD_BAUDRATE;
    let mut uart_md = Uart::new(
        p.UART4,
        p.PC11,
        p.PC10,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH2,
        uart_md_config,
    )
    .unwrap();

    let mut uart_bno_config = usart::Config::default();
    uart_bno_config.baudrate = bno08x_rvc::BNO08X_UART_RVC_BAUD_RATE;
    let uart_bno = Uart::new(
        p.USART6,
        p.PC7,
        p.PC6,
        Irqs,
        p.DMA2_CH6,
        p.DMA2_CH1,
        uart_bno_config,
    )
    .unwrap();
    // reset bno08x
    let mut gpio_reset = Output::new(p.PA0, Level::High, embassy_stm32::gpio::Speed::Low);
    gpio_reset.set_low();
    Timer::after(Duration::from_millis(10)).await;
    gpio_reset.set_high();
    Timer::after(Duration::from_millis(100)).await;

    // initialize sensor system
    let adc = Adc::new(p.ADC1);
    let line_s0 = Output::new(p.PB12, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let line_s1 = Output::new(p.PB13, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let line_s2 = Output::new(p.PB14, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let line_s3 = Output::new(p.PB15, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let ir_s0 = Output::new(p.PB0, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let ir_s1 = Output::new(p.PB1, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let ir_s2 = Output::new(p.PB4, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);
    let ir_s3 = Output::new(p.PB5, Level::Low, embassy_stm32::gpio::Speed::VeryHigh);

    let mut sensor_system = SensorSystem::new(
        adc, line_s0, line_s1, line_s2, line_s3,
        ir_s0, ir_s1, ir_s2, ir_s3,
    );

    // initialize flash and settings
    let f = Rc::new(RefCell::new(Flash::new_blocking(p.FLASH)));
    let mut settings_data = flash_read(&mut f.clone().borrow_mut()).unwrap_or(Settings::DEFAULT);
    validate_and_fix_settings(&mut settings_data, &mut f.clone().borrow_mut()).unwrap();
    let settings = Rc::new(RefCell::new(settings_data));

    info!("line strength: {}", settings.as_ref().borrow().line_threshold);

    // UI
    let gpio_ui_toggle = ExtiInput::new(p.PC12, p.EXTI12, Pull::None);
    let gpio_ui_up = ExtiInput::new(p.PC13, p.EXTI13, Pull::None);
    let gpio_ui_down = ExtiInput::new(p.PC14, p.EXTI14, Pull::None);
    let gpio_ui_enter = ExtiInput::new(p.PC15, p.EXTI15, Pull::None);

    let mut config = i2c::Config::default();
    config.timeout = Duration::from_millis(10);
    let ssd1306_i2c = I2c::new_blocking(p.I2C3, p.PA8, p.PC9, Hertz::khz(200), config);

    let ssd1306_interface = I2CDisplayInterface::new(ssd1306_i2c);
    let ssd1306 = Ssd1306::new(
        ssd1306_interface,
        DisplaySize128x64,
        ssd1306::prelude::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    static SSD1306: StaticCell<
        Ssd1306<
            I2CInterface<I2c<mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
    > = StaticCell::new();

    let ssd1306: &'static mut Ssd1306<
        I2CInterface<I2c<mode::Blocking>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    > = SSD1306.init(ssd1306);

    let mut ssd1306_init_success = false;
    for _ in 0..10 {
        match ssd1306.init() {
            Ok(_) => {
                ssd1306_init_success = true;
                break;
            }
            Err(_) => {
                error!("Can't initialize ssd1306");
            }
        }
    }

    // Create UI values using the UI system
    let (shutdown, reboot, value_line, value_have_ball) = UISystem::create_ui_and_values();

    // UI view
    let ui_text_interface = Text::new("INTERFACE", embedded_graphics::mono_font::ascii::FONT_6X10);

    let shutdown_clone = shutdown.clone();
    let ui_button_shutdown = Button::new(
        "Shutdown",
        move |pressed| {
            shutdown_clone.replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let reboot_clone = reboot.clone();
    let ui_button_reboot = Button::new(
        "Reboot",
        move |pressed| {
            reboot_clone.replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let ui_value_coat = Value::new(
        "ATK2",
        "",
        move |value| {
            *value = match settings_clone.as_ref().borrow().opp_goal_color {
                GoalColor::Blue => "B",
                GoalColor::Yellow => "Y",
            }
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_button_coat_change = Button::new(
        "SW Coat",
        move |pressed| {
            if pressed {
                let toggle_color = match settings_clone.as_ref().borrow().opp_goal_color {
                    GoalColor::Blue => GoalColor::Yellow,
                    GoalColor::Yellow => GoalColor::Blue,
                };
                settings_clone.borrow_mut().opp_goal_color = toggle_color;
                flash_write(&mut f_clone.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
            }
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let line_value_clone = value_line.clone();
    let ui_value_line = Value::new(
        "L",
        0.0,
        move |value| {
            *value = *line_value_clone.borrow_mut();
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_slider_line_strength = Slider::new(
        settings.as_ref().borrow().line_threshold,
        0.0,
        1.0,
        0.005,
        move |value| {
            settings_clone.borrow_mut().line_threshold = value;
            flash_write(&mut f_clone.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let have_ball_value_clone = value_have_ball.clone();
    let ui_value_have_ball = Value::new(
        "B",
        0,
        move |value| {
            *value = *have_ball_value_clone.borrow_mut();
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_slider_have_ball_threshold = Slider::new(
        settings.as_ref().borrow().have_ball_threshold,
        0,
        2000,
        50,
        move |value| {
            settings_clone.borrow_mut().have_ball_threshold = value;
            flash_write(&mut f_clone.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let ui_text_speed_mul = Text::new("Speed Mul", embedded_graphics::mono_font::ascii::FONT_6X10);

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_slider_robot_speed_multiplier = Slider::new(
        settings.as_ref().borrow().robot_speed_multiplier,
        0.0,
        5.0,
        0.1,
        move |value| {
            settings_clone.borrow_mut().robot_speed_multiplier = value;
            flash_write(&mut f_clone.borrow_mut(), &settings_clone.borrow_mut()).unwrap();
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let settings_clone = settings.clone();
    let f_clone = f.clone();
    let ui_button_settings_reset = Button::new(
        "Reset",
        move |pressed| {
            if pressed {
                flash_write(&mut f_clone.borrow_mut(), &Settings::DEFAULT).unwrap();
                settings_clone.replace(flash_read(&mut f_clone.borrow_mut()).unwrap());
            }
        },
        embedded_graphics::mono_font::ascii::FONT_6X10,
    );

    let elements = elements![
        Ssd1306<
            I2CInterface<I2c<mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
        ui_text_interface,
        ui_button_shutdown,
        ui_button_reboot,
        ui_value_coat,
        ui_button_coat_change,
        ui_value_line,
        ui_slider_line_strength,
        ui_value_have_ball,
        ui_slider_have_ball_threshold,
        ui_text_speed_mul,
        ui_slider_robot_speed_multiplier,
        ui_button_settings_reset
    ];
    let menu = menus![
        Ssd1306<
            I2CInterface<I2c<mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
        ListMenu::new(
            elements,
            ListMenuOption {
                position: Point::new(64, 0),
                size: Size::new(64, 64),
                vertical_num: 4,
                element_margin: 1,
                cursor_line_len: 4,
            },
        ),
        RobotStatusMenu::new(RobotStatusMenuOption {
            position: Point::new(2, 2),
            size: Size::new(56, 56),
        })
    ];

    let ui_option = HubUIOption {};

    let mut ui = HubUI::new(ssd1306, menu, ui_option);
    let display = ui.update(&Event::None);
    if ssd1306_init_success {
        ssd1306_init_success = display.flush().is_ok();
    }

    let shutdown = shutdown.clone();
    let reboot = reboot.clone();

    let neo_pixel_pwm_hz = Hertz::khz(500);
    let neo_pixel_pwm = SimplePwm::new(
        p.TIM4,
        Some(PwmPin::new_ch1(p.PB6, OutputType::PushPull)),
        None,
        None,
        None,
        neo_pixel_pwm_hz,
        CountingMode::EdgeAlignedUp,
    );
    let neo_pixel = NeoPixelPwm::new(neo_pixel_pwm, neo_pixel_pwm_hz);
    static NEO_PIXEL_DMA: StaticCell<peripherals::DMA1_CH0> = StaticCell::new();
    let neo_pixel_dma: &'static mut peripherals::DMA1_CH0 = NEO_PIXEL_DMA.init(p.DMA1_CH0);

    let mut rotation_pid: pid::Pid<f32> = pid::Pid::new(0.0, 100.0);
    rotation_pid.p(7.0, 100.0);

    const WHEEL_R: f32 = 25.0 / 1000.0;
    const THREAD: f32 = 108.0 / 1000.0;
    let wheel_calc1 = omni::OmniWheel::new(45.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc2 = omni::OmniWheel::new(315.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc3 = omni::OmniWheel::new(225.0_f32.to_radians(), WHEEL_R, THREAD);
    let wheel_calc4 = omni::OmniWheel::new(135.0_f32.to_radians(), WHEEL_R, THREAD);

    enum AdcState {
        OnGround,
        OnLine(f32, f32, f32, u32),
        OutOfLineOverCenter(f32, f32, f32, u32),
    }
    let _prev_line_state = AdcState::OnGround;

    // Spawn communication tasks
    spawner.must_spawn(bno08x_task(uart_bno));
    spawner.must_spawn(uart_jetson_task(uart_jetson));
    
    // Spawn UI task if display initialized successfully
    if ssd1306_init_success {
        spawner.must_spawn(ui_task(ui, gpio_ui_up, gpio_ui_down, gpio_ui_enter));
    }
    
    // Spawn NeoPixel task
    spawner.must_spawn(neo_pixel::neo_pixel_task(neo_pixel, neo_pixel_dma));

    info!("[nv1-hub] initialized");

    let mut line_processor = LineProcessor::new();
    let mut prev_time = Instant::now();
    
    loop {
        let settings = settings.as_ref().borrow().clone();
        let yaw = G_YAW.lock().await.clone().take();

        let sensor_readings = sensor_system.read_sensors(&mut p.PC0, &mut p.PC1, &mut p.PC2, &mut p.PC3);
        
        let on_line = calculate_line_vec_with_threshold(
            &sensor_readings.adc_line,
            &sensor_system.adc_line_sin,
            &sensor_system.adc_line_cos,
            settings.line_threshold,
        );

        let calc_line = line_processor.process_line(on_line, settings.line_threshold);


        let received_msg = G_JETSON_RX.lock().await.clone();
        let (vel_x, vel_y) = if let Some((line_x, line_y)) = calc_line {
            (line_x * LINE_SPEED_MULTIPLIER, line_y * LINE_SPEED_MULTIPLIER)
        } else {
            (
                received_msg.borrow().vel.x * settings.robot_speed_multiplier,
                received_msg.borrow().vel.y * settings.robot_speed_multiplier,
            )
        };
        // info!("Vel X: {}, Vel Y: {}", vel_x, vel_y);
        // info!("opp: {}", received_msg.borrow().goal_opp);

        rotation_pid.setpoint(0.0);
        let rotation_pid_result = rotation_pid.next_control_output(yaw);
        let rotation_vel = rotation_pid_result.output;
        let motor1 = wheel_calc1.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor2 = wheel_calc2.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor3 = wheel_calc3.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        let motor4 = wheel_calc4.calculate(vel_x, vel_y, 0.0, rotation_vel) / (2.0 * PI);
        // info!(
        //     "Motor1: {}, Motor2: {}, Motor3: {}, Motor4: {}",
        //     motor1, motor2, motor3, motor4
        // );

        let pause = gpio_ui_toggle.is_high();
        let md_msg = if pause {
            nv1_msg::md::ToMD {
                enable: false,
                m1: 0.0,
                m2: 0.0,
                m3: 0.0,
                m4: 0.0,
            }
        } else {
            nv1_msg::md::ToMD {
                enable: true,
                m1: motor1,
                m2: motor2,
                m3: motor3,
                m4: motor4,
            }
        };

        let md_data = postcard::to_vec_cobs::<nv1_msg::md::ToMD, 64>(&md_msg).unwrap();
        match uart_md.write(&md_data).await {
            Ok(_) => {}
            Err(err) => {
                error!("[UART MD] write error: {:?}", err);
            }
        };

        let opp_color = match settings.opp_goal_color {
            GoalColor::Blue => settings.opencv_goal_blue,
            GoalColor::Yellow => settings.opencv_goal_yellow,
        };

        let own_color = match settings.opp_goal_color {
            GoalColor::Blue => settings.opencv_goal_yellow,
            GoalColor::Yellow => settings.opencv_goal_blue,
        };

        let msg_tx = nv1_msg::hub::ToJetson {
            sys: nv1_msg::hub::System {
                pause,
                shutdown: *shutdown.borrow_mut(),
                reboot: *reboot.borrow_mut(),
            },
            vel: nv1_msg::hub::Movement {
                x: received_msg.borrow().vel.x,
                y: received_msg.borrow().vel.y,
                angle: yaw,
            },
            sensor: nv1_msg::hub::Sensor {
                ir: nv1_msg::hub::Ir {
                    x: sensor_readings.ir_x,
                    y: sensor_readings.ir_y,
                    strength: 0.0,
                },
                on_line: on_line.is_some(),
                have_ball: sensor_readings.adc_have_ball < settings.have_ball_threshold,
            },
            config: nv1_msg::hub::JetsonConfig::OpenCV(nv1_msg::hub::OpenCVConfig {
                opp_color,
                own_color,
            }),
        };
        G_JETSON_TX.lock().await.replace(msg_tx);

        G_NEO_PIXEL_DATA.lock().await.ball_dir = sensor_readings.ir_angle;
        G_NEO_PIXEL_DATA.lock().await.pause = pause;

        // update UI buf
        value_line.replace(sensor_readings.adc_line_max);
        value_have_ball.replace(sensor_readings.adc_have_ball);

        let now_time = Instant::now();
        let elapsed_time = now_time - prev_time;
        // info!("elapsed time: {}", elapsed_time.as_micros());
        if LOOP_US > elapsed_time.as_micros() {
            Timer::after_micros(LOOP_US - elapsed_time.as_micros()).await;
        }
        prev_time = now_time;
    }
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

#[embassy_executor::task]
async fn ui_task(
    mut ui: HubUI<
        'static,
        Ssd1306<
            I2CInterface<I2c<'static, mode::Blocking>>,
            DisplaySize128x64,
            BufferedGraphicsMode<DisplaySize128x64>,
        >,
    >,
    mut gpio_ui_up: ExtiInput<'static>,
    mut gpio_ui_down: ExtiInput<'static>,
    mut gpio_ui_enter: ExtiInput<'static>,
) {
    loop {
        select3(
            gpio_ui_up.wait_for_any_edge(),
            gpio_ui_down.wait_for_any_edge(),
            gpio_ui_enter.wait_for_any_edge(),
        )
        .await;

        let event = if gpio_ui_up.is_high() {
            Event::KeyDown(EventKey::Up)
        } else if gpio_ui_down.is_high() {
            Event::KeyDown(EventKey::Down)
        } else if gpio_ui_enter.is_high() {
            Event::KeyDown(EventKey::Enter)
        } else {
            Event::None
        };

        let display = ui.update(&event);
        let _ = display.flush();
    }
}

