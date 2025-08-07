use alloc::rc::Rc;
use core::cell::RefCell;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::{i2c::I2c, mode};
use embassy_futures::select::select3;
use nv1_hub_ui::{Event, EventKey, HubUI};
use ssd1306::mode::{BufferedGraphicsMode, DisplayConfig};
use ssd1306::prelude::I2CInterface;
use ssd1306::{size::DisplaySize128x64, Ssd1306};
use defmt::error;

pub type DisplayType = Ssd1306<
    I2CInterface<I2c<'static, mode::Blocking>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

pub struct UISystem {
    pub display: &'static mut DisplayType,
    pub gpio_ui_up: ExtiInput<'static>,
    pub gpio_ui_down: ExtiInput<'static>,
    pub gpio_ui_enter: ExtiInput<'static>,
}

impl UISystem {
    pub fn new(
        display: &'static mut DisplayType,
        gpio_ui_up: ExtiInput<'static>,
        gpio_ui_down: ExtiInput<'static>,
        gpio_ui_enter: ExtiInput<'static>,
    ) -> Self {
        Self {
            display,
            gpio_ui_up,
            gpio_ui_down,
            gpio_ui_enter,
        }
    }

    pub fn try_initialize_display(display: &mut DisplayType) -> bool {
        for _ in 0..10 {
            match display.init() {
                Ok(_) => return true,
                Err(_) => {
                    error!("Can't initialize ssd1306");
                }
            }
        }
        false
    }

    pub fn create_ui_and_values() -> (
        Rc<RefCell<bool>>,
        Rc<RefCell<bool>>,
        Rc<RefCell<f32>>,
        Rc<RefCell<u16>>,
    ) {
        let shutdown = Rc::new(RefCell::new(false));
        let reboot = Rc::new(RefCell::new(false));
        let value_line = Rc::new(RefCell::new(0.0));
        let value_have_ball = Rc::new(RefCell::new(0));
        
        (shutdown, reboot, value_line, value_have_ball)
    }
}

#[embassy_executor::task]
pub async fn ui_task(
    mut ui: HubUI<
        'static,
        DisplayType,
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

// This function is not used in the current implementation
// Display initialization is handled directly in main.rs