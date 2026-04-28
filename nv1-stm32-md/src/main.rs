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

use defmt::{error, warn};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use motor::{MotorGroupComplementary, MotorGroupSimple};

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts, dma,
    gpio::OutputType,
    mode, peripherals,
    time::Hertz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        qei::{Qei, QeiMode},
        simple_pwm::{PwmPin, SimplePwm},
        Channel,
    },
    usart::{self, Config, Uart},
};
use embassy_time::{with_timeout, Duration, Instant, Ticker};

use fmt::info;

use nv1_md_core::{
    control::{SpeedController, SpeedControllerConfig},
    encoder::{EncoderConfig, EncoderTracker},
    failsafe::CommHealth,
    frame::{FrameReader, PushOutcome},
    motor::{pwm_command_from_output, Motors},
};

bind_interrupts!(struct Irqs {
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    DMA1_STREAM1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
});

// ---------------------------------------------------------------------------
// Robot geometry / control tuning
// ---------------------------------------------------------------------------

const MOTOR_ENCODER_COUNTS_PER_REV: u32 = 3 * 4;
const MOTOR_GEAR_RATIO: f32 = 1.0 / 19.225;

/// Period of the closed-loop control update.
///
/// NOTE: PID gains were originally tuned against an unbounded loop period
/// (effectively whatever the embedded executor scheduled). Pinning the period
/// to a fixed value is a behaviour change — gains may need re-tuning on
/// hardware. See README / commit message.
const CONTROL_PERIOD_MS: u64 = 10;
const CONTROL_PERIOD_S: f32 = (CONTROL_PERIOD_MS as f32) / 1000.0;

/// Maximum age of the most recent valid Hub message before we trip the
/// failsafe and stop the motors.
const COMM_TIMEOUT_MS: u64 = 100;

/// Encoder polarity per axis. Motor 4 is mounted with reversed polarity vs
/// the others, so its encoder counts decrease for the rotation direction we
/// treat as positive in software.
const ENCODER_INVERTED: [bool; 4] = [false, false, false, true];

/// PWM command polarity per axis. Currently all axes drive the bridge in the
/// same direction — kept as a per-axis array so polarity can be flipped from
/// configuration without touching the control logic.
const COMMAND_INVERTED: [bool; 4] = [false, false, false, false];

const PID_KP: f32 = 8.0;
const PID_KI: f32 = 4.0;
const PID_KD: f32 = 0.0;
const PID_OUTPUT_LIMIT: f32 = 100.0;
const PID_P_LIMIT: f32 = 100.0;
const PID_I_LIMIT: f32 = 50.0;
const PID_D_LIMIT: f32 = 0.0;

const PWM_FREQ_HZ: u32 = 470;
const PWM_NOMINAL_FULL_SCALE: u16 = 128;

// ---------------------------------------------------------------------------
// Shared state between UART receive task and control loop
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct HubState {
    msg: nv1_msg::md::ToMD,
    health: CommHealth,
}

const INITIAL_HUB_STATE: HubState = HubState {
    msg: nv1_msg::md::ToMD {
        enable: false,
        m1: 0.0,
        m2: 0.0,
        m3: 0.0,
        m4: 0.0,
    },
    health: CommHealth::new(COMM_TIMEOUT_MS),
};

static G_HUB: Mutex<CriticalSectionRawMutex, RefCell<HubState>> =
    Mutex::new(RefCell::new(INITIAL_HUB_STATE));

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
        Hertz(PWM_FREQ_HZ),
        Default::default(),
    );

    pwm1.enable(Channel::Ch1);
    pwm1.enable(Channel::Ch2);
    pwm1.enable(Channel::Ch3);
    // TIM1 has no Ch4 complementary output, so we cannot use ComplementaryPwm::enable
    // (it would assert in stm32-metapac). Enable Ch4's regular output directly via PAC.
    embassy_stm32::pac::TIM1
        .ccer()
        .modify(|w| w.set_cce(3, true));

    let motor_group1 = MotorGroupComplementary::new(
        pwm1,
        Channel::Ch4,
        Channel::Ch1,
        Channel::Ch2,
        Channel::Ch3,
        PWM_NOMINAL_FULL_SCALE,
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
        Hertz(PWM_FREQ_HZ),
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
        PWM_NOMINAL_FULL_SCALE,
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

    let encoder_cfg = |inverted: bool| EncoderConfig {
        counts_per_motor_rev: MOTOR_ENCODER_COUNTS_PER_REV,
        gear_ratio: MOTOR_GEAR_RATIO,
        inverted,
    };

    let mut trackers = [
        EncoderTracker::new(qei1.count().into(), encoder_cfg(ENCODER_INVERTED[0])),
        EncoderTracker::new(qei2.count().into(), encoder_cfg(ENCODER_INVERTED[1])),
        EncoderTracker::new(qei3.count().into(), encoder_cfg(ENCODER_INVERTED[2])),
        EncoderTracker::new(qei4.count().into(), encoder_cfg(ENCODER_INVERTED[3])),
    ];

    let pid_cfg = SpeedControllerConfig {
        kp: PID_KP,
        ki: PID_KI,
        kd: PID_KD,
        p_limit: PID_P_LIMIT,
        i_limit: PID_I_LIMIT,
        d_limit: PID_D_LIMIT,
        output_limit: PID_OUTPUT_LIMIT,
    };

    let mut controllers = [
        SpeedController::new(pid_cfg),
        SpeedController::new(pid_cfg),
        SpeedController::new(pid_cfg),
        SpeedController::new(pid_cfg),
    ];

    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_PERIOD_MS));

    info!("[MD] Initialized");

    let mut prev_alive = false;

    loop {
        let now_ms = Instant::now().as_millis();

        let state = *G_HUB.lock().await.borrow();
        let alive = state.health.is_alive(now_ms);

        if alive != prev_alive {
            if alive {
                info!("[MD] hub link alive");
            } else {
                warn!("[MD] hub link lost — entering failsafe");
            }
            prev_alive = alive;
        }

        if alive && state.msg.enable {
            let setpoints = [state.msg.m1, state.msg.m2, state.msg.m3, state.msg.m4];
            let counts: [u32; 4] = [
                qei1.count().into(),
                qei2.count().into(),
                qei3.count().into(),
                qei4.count().into(),
            ];

            for i in 0..4 {
                controllers[i].setpoint(setpoints[i]);
                let rps = trackers[i].update(counts[i], CONTROL_PERIOD_S);

                match controllers[i].next(rps) {
                    Some(out) => {
                        let cmd = pwm_command_from_output(out, COMMAND_INVERTED[i]);
                        motors.set_speed(i, cmd);
                    }
                    None => motors.stop(i),
                }
            }
        } else {
            // Failsafe: comm lost OR hub disabled drive. Stop motors and reset
            // the integrators so we don't kick on resume.
            for c in controllers.iter_mut() {
                c.setpoint(0.0);
                c.reset();
            }
            motors.stop_all();
        }

        ticker.next().await;
    }
}

// ---------------------------------------------------------------------------
// UART receive task
// ---------------------------------------------------------------------------

/// Maximum size of one COBS-framed `ToMD` message. Empirically the encoded
/// length is well under 32 bytes; 64 leaves comfortable headroom and matches
/// the buffer used by the Hub side serializer.
const FRAME_BUF: usize = 64;

/// Per-read timeout. The Hub publishes at ~100 Hz; tripping after 50 ms is
/// long enough to ride out one missed frame but short enough to surface a
/// disconnected cable within a control period.
const READ_TIMEOUT_MS: u64 = 50;

#[embassy_executor::task]
async fn uart_task(usart: Uart<'static, mode::Async>) {
    let (_, uart_rx) = usart.split();

    let mut dma_buf = [0u8; 256];
    let mut uart_rx = uart_rx.into_ring_buffered(&mut dma_buf);
    uart_rx.start_uart();

    let mut reader: FrameReader<FRAME_BUF> = FrameReader::new();
    let mut chunk = [0u8; 64];

    loop {
        match with_timeout(
            Duration::from_millis(READ_TIMEOUT_MS),
            uart_rx.read(&mut chunk),
        )
        .await
        {
            Ok(Ok(n)) => {
                for &b in &chunk[..n] {
                    process_byte(&mut reader, b).await;
                }
            }
            Ok(Err(_e)) => {
                // Bus-level error (framing, overrun, parity). Reset our
                // assembler, restart UART, and trip the failsafe.
                reader.reset();
                uart_rx.start_uart();
                invalidate_health().await;
                error!("[MD/UART] bus error");
            }
            Err(_) => {
                // No bytes received within READ_TIMEOUT_MS — Hub link dead.
                reader.reset();
                invalidate_health().await;
            }
        }
    }
}

async fn process_byte(reader: &mut FrameReader<FRAME_BUF>, b: u8) {
    match reader.push(b) {
        PushOutcome::Pending => {}
        PushOutcome::Frame(frame) => match postcard::from_bytes_cobs::<nv1_msg::md::ToMD>(frame) {
            Ok(msg) => {
                let now_ms = Instant::now().as_millis();
                let g = G_HUB.lock().await;
                let mut state = g.borrow_mut();
                state.msg = msg;
                state.health.mark_received(now_ms);
            }
            Err(_) => {
                invalidate_health().await;
                warn!("[MD/UART] parse error");
            }
        },
        PushOutcome::Overflow => {
            invalidate_health().await;
            warn!("[MD/UART] frame overflow");
        }
    }
}

async fn invalidate_health() {
    let g = G_HUB.lock().await;
    g.borrow_mut().health.invalidate();
}
