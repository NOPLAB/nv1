use bbqueue::BBBuffer;
use core::borrow::Borrow;
use core::cell::RefCell;
use defmt::error;
use embassy_stm32::{mode, usart::Uart};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{with_timeout, Duration, Timer};

pub static G_BB: BBBuffer<{ bno08x_rvc::BUFFER_SIZE }> = BBBuffer::new();
pub static G_YAW: Mutex<CriticalSectionRawMutex, RefCell<f32>> = Mutex::new(RefCell::new(0.0));
pub static G_JETSON_RX: Mutex<CriticalSectionRawMutex, RefCell<nv1_msg::hub::ToHub>> =
    Mutex::new(RefCell::new(nv1_msg::hub::ToHub {
        vel: nv1_msg::hub::Movement {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
        },
        kick: false,
        goal_opp: None,
        goal_own: None,
    }));
pub static G_JETSON_TX: Mutex<CriticalSectionRawMutex, RefCell<nv1_msg::hub::ToJetson>> =
    Mutex::new(RefCell::new(nv1_msg::hub::ToJetson {
        sys: nv1_msg::hub::System {
            pause: false,
            shutdown: false,
            reboot: false,
        },
        vel: nv1_msg::hub::Movement {
            x: 0.0,
            y: 0.0,
            angle: 0.0,
        },
        sensor: nv1_msg::hub::Sensor {
            ir: nv1_msg::hub::Ir {
                x: 0.0,
                y: 0.0,
                strength: 0.0,
            },
            on_line: false,
            have_ball: false,
        },
        config: nv1_msg::hub::JetsonConfig::None,
    }));

pub struct CommunicationSystem {
    pub uart_jetson: Uart<'static, mode::Async>,
    pub uart_md: Uart<'static, mode::Async>,
    pub uart_bno: Uart<'static, mode::Async>,
}

impl CommunicationSystem {
    pub fn new(
        uart_jetson: Uart<'static, mode::Async>,
        uart_md: Uart<'static, mode::Async>,
        uart_bno: Uart<'static, mode::Async>,
    ) -> Self {
        Self {
            uart_jetson,
            uart_md,
            uart_bno,
        }
    }

    // This method is not used in the current implementation
    // Tasks are spawned directly in main.rs

    pub async fn send_md_message(
        &mut self,
        msg: &nv1_msg::md::ToMD,
    ) -> Result<(), embassy_stm32::usart::Error> {
        let md_data = postcard::to_vec_cobs::<nv1_msg::md::ToMD, 64>(msg)
            .map_err(|_| embassy_stm32::usart::Error::Framing)?;

        match self.uart_md.write(&md_data).await {
            Ok(_) => Ok(()),
            Err(err) => {
                error!("[UART MD] write error: {:?}", err);
                Err(err)
            }
        }
    }
}

#[embassy_executor::task]
pub async fn bno08x_task(mut uart: Uart<'static, mode::Async>) {
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
pub async fn uart_jetson_task(uart: Uart<'static, mode::Async>) {
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

pub async fn send_system_command(shutdown: bool, reboot: bool) {
    let jetson_tx = G_JETSON_TX.lock().await;
    let mut msg = jetson_tx.take();
    msg.sys.shutdown = shutdown;
    msg.sys.reboot = reboot;
    jetson_tx.replace(msg);
}

use crate::neo_pixel::NeoPixelData;

pub static G_NEO_PIXEL_DATA: Mutex<CriticalSectionRawMutex, NeoPixelData> =
    Mutex::new(NeoPixelData {
        jetson_connecting: false,
        pause: false,
        ball_dir: 0.0,
    });
