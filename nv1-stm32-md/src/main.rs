#![no_std]
#![no_main]

mod fmt;
mod motor;

extern crate alloc;

use core::ptr::addr_of_mut;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use core::cell::RefCell;

use defmt::error;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use motor::{MotorGroupComplementary, MotorGroupSimple, Motors};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    dma,
    gpio::OutputType,
    mode,
    peripherals,
    timer::{
        Channel,
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        qei::{Qei, QeiMode},
        simple_pwm::{PwmPin, SimplePwm},
    },
    usart::{self, Uart, Config},
    time::Hertz,
};
use embassy_time::{Duration, Timer, with_timeout};

use fmt::info;

use pid::Pid;

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    DMA1_STREAM1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
});

const MOTOR_ENCODER_PLUS: usize = 3 * 4;
const MOTOR_GEAR_RATIO: f32 = 1.0 / 19.225;

static G_HUB_MSG: Mutex<CriticalSectionRawMutex, RefCell<nv1_msg::md::ToMD>> =
    Mutex::new(RefCell::new(nv1_msg::md::ToMD {
        enable: false,
        m1: 0.0,
        m2: 0.0,
        m3: 0.0,
        m4: 0.0,
    }));

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // initialize static heap
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
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

    let p = embassy_stm32::init(config);

    let pwm1_ch1 = PwmPin::new(p.PA8, OutputType::PushPull);
    let pwm1_ch4 = PwmPin::new(p.PA11, OutputType::PushPull);

    let pwm1_ch2n = ComplementaryPwmPin::new(p.PB14, OutputType::PushPull);
    let pwm1_ch3n = ComplementaryPwmPin::new(p.PB15, OutputType::PushPull);

    let mut pwm1 = ComplementaryPwm::new(
        p.TIM1,
        Some(pwm1_ch1),
        None,
        None,
        Some(pwm1_ch2n),
        None,
        Some(pwm1_ch3n),
        Some(pwm1_ch4),
        None,
        Hertz(470),
        Default::default(),
    );

    pwm1.enable(Channel::Ch1);
    pwm1.enable(Channel::Ch2);
    pwm1.enable(Channel::Ch3);
    // TIM1 has no Ch4 complementary output, so we cannot use ComplementaryPwm::enable
    // (it would assert in stm32-metapac). Enable Ch4's regular output directly via PAC.
    embassy_stm32::pac::TIM1.ccer().modify(|w| w.set_cce(3, true));

    let motor_group1 = MotorGroupComplementary::new(
        pwm1,
        Channel::Ch4,
        Channel::Ch1,
        Channel::Ch2,
        Channel::Ch3,
        128,
    );

    let pwm8_ch1 = PwmPin::new(p.PC6, OutputType::PushPull);
    let pwm8_ch2 = PwmPin::new(p.PC7, OutputType::PushPull);
    let pwm8_ch3 = PwmPin::new(p.PC8, OutputType::PushPull);
    let pwm8_ch4 = PwmPin::new(p.PC9, OutputType::PushPull);

    let mut pwm8 = SimplePwm::new(
        p.TIM8,
        Some(pwm8_ch1),
        Some(pwm8_ch2),
        Some(pwm8_ch3),
        Some(pwm8_ch4),
        Hertz(470),
        Default::default(),
    );

    pwm8.channel(Channel::Ch1).enable();
    pwm8.channel(Channel::Ch2).enable();
    pwm8.channel(Channel::Ch3).enable();
    pwm8.channel(Channel::Ch4).enable();

    let motor_group2 = MotorGroupSimple::new(
        pwm8,
        Channel::Ch2,
        Channel::Ch1,
        Channel::Ch4,
        Channel::Ch3,
        128,
    );

    let mut motors = Motors::new(motor_group1, motor_group2);

    let mut config = Config::default();
    config.baudrate = 2_000_000;
    let usart = Uart::new(
        p.USART3, p.PC5, p.PB10, p.DMA1_CH3, p.DMA1_CH1, Irqs, config,
    )
    .unwrap();

    spawner.spawn(uart_task(usart).unwrap());

    let qei_config = embassy_stm32::timer::qei::Config {
        mode: QeiMode::Mode3,
        ..Default::default()
    };

    let qei1 = Qei::new(p.TIM5, p.PA0, p.PA1, qei_config);
    let qei2 = Qei::new(p.TIM3, p.PB4, p.PB5, qei_config);
    let qei3 = Qei::new(p.TIM4, p.PB6, p.PB7, qei_config);
    let qei4 = Qei::new(p.TIM2, p.PA15, p.PB9, qei_config);

    let mut prev1 = qei1.count();
    let mut prev2 = qei2.count();
    let mut prev3 = qei3.count();
    let mut prev4 = qei4.count();

    let mut pid1: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid2: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid3: Pid<f32> = pid::Pid::new(0.0, 100.0);
    let mut pid4: Pid<f32> = pid::Pid::new(0.0, 100.0);

    pid1.p(8.0, 100.0).i(4.0, 50.0).d(0.0, 0.0);
    pid2.p(8.0, 100.0).i(4.0, 50.0).d(0.0, 0.0);
    pid3.p(8.0, 100.0).i(4.0, 50.0).d(0.0, 0.0);
    pid4.p(8.0, 100.0).i(4.0, 50.0).d(0.0, 0.0);

    info!("[MD] Initialized");

    loop {
        let msg = *G_HUB_MSG.lock().await.borrow();

        if msg.enable {
            pid1.setpoint(msg.m1);
            pid2.setpoint(msg.m2);
            pid3.setpoint(msg.m3);
            pid4.setpoint(msg.m4);

            let cnt1 = qei1.count();
            let cnt2 = qei2.count();
            let cnt3 = qei3.count();
            let cnt4 = qei4.count();
            let delta1 = cnt1.wrapping_sub(prev1) as i16 as f32;
            let delta2 = cnt2.wrapping_sub(prev2) as i16 as f32;
            let delta3 = cnt3.wrapping_sub(prev3) as i16 as f32;
            // Motor 4 is mounted with reversed polarity vs the others.
            let delta4 = -(cnt4.wrapping_sub(prev4) as i16 as f32);
            prev1 = cnt1;
            prev2 = cnt2;
            prev3 = cnt3;
            prev4 = cnt4;

            let motor1_rps = delta1 / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor2_rps = delta2 / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor3_rps = delta3 / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;
            let motor4_rps = delta4 / MOTOR_ENCODER_PLUS as f32 * MOTOR_GEAR_RATIO / 0.01;

            info!(
                "rps: {}, {}, {}, {}",
                motor1_rps, motor2_rps, motor3_rps, motor4_rps
            );

            if motor1_rps.is_nan() {
                continue;
            }

            let motor1_output = pid1.next_control_output(motor1_rps);
            let motor2_output = pid2.next_control_output(motor2_rps);
            let motor3_output = pid3.next_control_output(motor3_rps);
            let motor4_output = pid4.next_control_output(motor4_rps);

            motors.set_speed1(motor1_output.output as i16);
            motors.set_speed2(motor2_output.output as i16);
            motors.set_speed3(motor3_output.output as i16);
            motors.set_speed4(motor4_output.output as i16);
        } else {
            pid1.setpoint(0.0);
            pid2.setpoint(0.0);
            pid3.setpoint(0.0);
            pid4.setpoint(0.0);

            motors.stop1();
            motors.stop2();
            motors.stop3();
            motors.stop4();
        }
    }
}

#[embassy_executor::task]
async fn uart_task(usart: Uart<'static, mode::Async>) {
    let (_, uart_rx) = usart.split();

    let mut dma_buf = [0u8; 128];
    let mut uart_rx = uart_rx.into_ring_buffered(&mut dma_buf);

    uart_rx.start_uart();

    loop {
        let mut byte = [0u8; 1];
        let mut msg_with_cobs = [0u8; 64];
        let mut c = 0;
        loop {
            let timeout_res =
                with_timeout(Duration::from_millis(50), uart_rx.read(&mut byte)).await;
            match timeout_res {
                Ok(receive_res) => match receive_res {
                    Ok(_size) => {
                        msg_with_cobs[c] = byte[0];
                        c += 1;

                        if byte[0] == 0 {
                            break;
                        }
                    }
                    Err(_err) => {
                        uart_rx.start_uart();
                    }
                },
                Err(_) => {
                    error!("[UART] timeout");

                    G_HUB_MSG.lock().await.replace(nv1_msg::md::ToMD {
                        enable: false,
                        m1: 0.0,
                        m2: 0.0,
                        m3: 0.0,
                        m4: 0.0,
                    });

                    break;
                }
            }
        }

        match postcard::from_bytes_cobs::<nv1_msg::md::ToMD>(&mut msg_with_cobs) {
            Ok(msg) => {
                G_HUB_MSG.lock().await.replace(msg);
            }
            Err(_) => {
                continue;
            }
        };

        Timer::after_millis(5).await;
    }
}
