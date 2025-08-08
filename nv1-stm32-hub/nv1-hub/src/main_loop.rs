use crate::{
    communication::{G_JETSON_RX, G_JETSON_TX, G_NEO_PIXEL_DATA, G_YAW},
    constants::*,
    motor_controller::MotorController,
    sensors::{calculate_line_vec_with_threshold, AdcSensor},
    settings::Settings,
};
use alloc::rc::Rc;
use core::cell::RefCell;
use defmt::info;
use embassy_stm32::{exti::ExtiInput, peripherals, usart::Uart};
use embassy_time::{Instant, Timer};
use nv1_msg;
use postcard;

pub struct MainLoopContext {
    pub motor_controller: MotorController,
    pub settings: Rc<RefCell<Settings>>,
    pub value_line: Rc<RefCell<f32>>,
    pub value_have_ball: Rc<RefCell<u32>>,
    pub gpio_ui_toggle: ExtiInput<'static>,
}

impl MainLoopContext {
    pub fn new(
        motor_controller: MotorController,
        settings: Rc<RefCell<Settings>>,
        value_line: Rc<RefCell<f32>>,
        value_have_ball: Rc<RefCell<u32>>,
        gpio_ui_toggle: ExtiInput<'static>,
    ) -> Self {
        Self {
            motor_controller,
            settings,
            value_line,
            value_have_ball,
            gpio_ui_toggle,
        }
    }

    pub async fn run_main_loop(
        &mut self,
        mut adc_sensor: AdcSensor,
        mut uart_md: Uart<'static, embassy_stm32::mode::Async>,
        mut adc_pins: (
            peripherals::PC0,
            peripherals::PC1,
            peripherals::PC2,
            peripherals::PC3,
        ),
    ) -> ! {
        let mut prev_time = Instant::now();

        info!("[nv1-hub] Start main loop...");

        loop {
            let settings = self.settings.as_ref().borrow().clone();
            let yaw = G_YAW.lock().await.clone().take();

            let sensor_readings = adc_sensor.read_sensors(
                &mut adc_pins.0,
                &mut adc_pins.1,
                &mut adc_pins.2,
                &mut adc_pins.3,
            );

            let on_line = calculate_line_vec_with_threshold(
                &sensor_readings.adc_line,
                &adc_sensor.line_sensor.sin_values,
                &adc_sensor.line_sensor.cos_values,
                settings.line_threshold,
            );

            let calc_line = self
                .motor_controller
                .process_line(on_line, settings.line_threshold);

            let received_msg = G_JETSON_RX.lock().await.clone();
            let (vel_x, vel_y) = if let Some(line_vector) = calc_line {
                (
                    line_vector.x * LINE_SPEED_MULTIPLIER,
                    line_vector.y * LINE_SPEED_MULTIPLIER,
                )
            } else {
                (
                    received_msg.borrow().vel.x * settings.robot_speed_multiplier,
                    received_msg.borrow().vel.y * settings.robot_speed_multiplier,
                )
            };

            let (motor1, motor2, motor3, motor4) = self
                .motor_controller
                .calculate_motor_values(vel_x, vel_y, yaw);

            let pause = self.gpio_ui_toggle.is_high();
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
                    defmt::error!("[nv1-hub] UART MD Write error: {:?}", err);
                }
            };

            let opp_color = settings.get_opp_color();

            let own_color = settings.get_own_color();

            let msg_tx = nv1_msg::hub::ToJetson {
                sys: nv1_msg::hub::System {
                    pause,
                    shutdown: false, // These will be handled by UI system
                    reboot: false,   // These will be handled by UI system
                },
                vel: nv1_msg::hub::Movement {
                    x: received_msg.borrow().vel.x,
                    y: received_msg.borrow().vel.y,
                    angle: yaw,
                },
                sensor: nv1_msg::hub::Sensor {
                    ir: nv1_msg::hub::Ir {
                        x: sensor_readings.ir_position.x,
                        y: sensor_readings.ir_position.y,
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

            // Update UI values
            self.value_line.replace(sensor_readings.adc_line_max);
            self.value_have_ball
                .replace(sensor_readings.adc_have_ball.into());

            let now_time = Instant::now();
            let elapsed_time = now_time - prev_time;
            if LOOP_US > elapsed_time.as_micros() {
                Timer::after_micros(LOOP_US - elapsed_time.as_micros()).await;
            }
            prev_time = now_time;
        }
    }
}
