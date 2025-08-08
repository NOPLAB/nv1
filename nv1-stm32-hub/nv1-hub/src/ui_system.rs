use defmt::error;

use embassy_stm32::exti::ExtiInput;
use embassy_stm32::i2c::I2c;
use nv1_hub_ui::{Event, EventKey, HubUI};
use ssd1306::mode::{BufferedGraphicsMode, DisplayConfig};
use ssd1306::prelude::I2CInterface;
use ssd1306::{size::DisplaySize128x64, Ssd1306};

pub type DisplayType = Ssd1306<
    I2CInterface<I2c<'static, embassy_stm32::mode::Blocking>>,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

#[allow(dead_code)]
pub struct UISystem {
    pub display: &'static mut DisplayType,
    pub gpio_ui_up: ExtiInput<'static>,
    pub gpio_ui_down: ExtiInput<'static>,
    pub gpio_ui_enter: ExtiInput<'static>,
}

#[allow(dead_code)]
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

    pub fn create_ui_system(
        ssd1306: &'static mut DisplayType,
        settings: alloc::rc::Rc<core::cell::RefCell<crate::settings::Settings>>,
        f: alloc::rc::Rc<
            core::cell::RefCell<
                embassy_stm32::flash::Flash<'static, embassy_stm32::flash::Blocking>,
            >,
        >,
    ) -> (
        nv1_hub_ui::HubUI<'static, DisplayType>,
        alloc::rc::Rc<core::cell::RefCell<bool>>,
        alloc::rc::Rc<core::cell::RefCell<bool>>,
        alloc::rc::Rc<core::cell::RefCell<f32>>,
        alloc::rc::Rc<core::cell::RefCell<u32>>,
    ) {
        let shutdown = alloc::rc::Rc::new(core::cell::RefCell::new(false));
        let reboot = alloc::rc::Rc::new(core::cell::RefCell::new(false));
        let value_line = alloc::rc::Rc::new(core::cell::RefCell::new(0.0));
        let value_have_ball = alloc::rc::Rc::new(core::cell::RefCell::new(0u32));

        use crate::settings::{flash_write, GoalColor};
        use alloc::{boxed::Box, vec::Vec};
        use embedded_graphics::{geometry::Point, geometry::Size, mono_font::ascii::FONT_6X10};
        use nv1_hub_ui::{
            elements,
            elements::{Button, Element, Slider, Text, Value},
            menu::{ListMenu, ListMenuOption, Menu},
            menus,
        };

        // Create UI elements
        let ui_text_interface = Text::new("INTERFACE", FONT_6X10);

        let shutdown_clone = shutdown.clone();
        let ui_button_shutdown = Button::new(
            "Shutdown",
            move |pressed| {
                shutdown_clone.replace(pressed);
            },
            FONT_6X10,
        );

        let reboot_clone = reboot.clone();
        let ui_button_reboot = Button::new(
            "Reboot",
            move |pressed| {
                reboot_clone.replace(pressed);
            },
            FONT_6X10,
        );

        let settings_clone = settings.clone();
        let ui_value_coat = Value::new(
            "ATK",
            "",
            move |value| {
                *value = match settings_clone.as_ref().borrow().get_opp_goal_color() {
                    GoalColor::Blue => "B",
                    GoalColor::Yellow => "Y",
                }
            },
            FONT_6X10,
        );

        let settings_clone = settings.clone();
        let f_clone = f.clone();
        let ui_button_coat_change = Button::new(
            "SW Coat",
            move |pressed| {
                if pressed {
                    settings_clone.borrow_mut().toggle_goal_color();
                    {
                        let mut flash_ref = f_clone.borrow_mut();
                        let settings_ref = settings_clone.borrow();
                        flash_write(&mut *flash_ref, &*settings_ref).unwrap();
                    }
                }
            },
            FONT_6X10,
        );

        let line_value_clone = value_line.clone();
        let ui_value_line = Value::new(
            "L",
            0.0,
            move |value| {
                *value = *line_value_clone.borrow_mut();
            },
            FONT_6X10,
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
            FONT_6X10,
        );

        let have_ball_value_clone = value_have_ball.clone();
        let ui_value_have_ball = Value::new(
            "B",
            0,
            move |value| {
                *value = *have_ball_value_clone.borrow_mut();
            },
            FONT_6X10,
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
            FONT_6X10,
        );

        let ui_text_speed_mul = Text::new("Speed", FONT_6X10);

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
            FONT_6X10,
        );

        let settings_clone = settings.clone();
        let f_clone = f.clone();
        let ui_button_settings_reset = Button::new(
            "Reset",
            move |pressed| {
                if pressed {
                    {
                        let mut flash_ref = f_clone.borrow_mut();
                        flash_write(&mut *flash_ref, &crate::settings::Settings::DEFAULT).unwrap();
                        let new_settings = crate::settings::flash_read(&mut *flash_ref).unwrap();
                        settings_clone.replace(new_settings);
                    }
                }
            },
            FONT_6X10,
        );

        let elements = elements![
            DisplayType,
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
            DisplayType,
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
            nv1_hub_ui::menu::RobotStatusMenu::new(nv1_hub_ui::menu::RobotStatusMenuOption {
                position: Point::new(2, 2),
                size: Size::new(56, 56),
            })
        ];

        let ui_option = nv1_hub_ui::HubUIOption {};

        let ui = nv1_hub_ui::HubUI::new(ssd1306, menu, ui_option);
        (ui, shutdown, reboot, value_line, value_have_ball)
    }
}

#[embassy_executor::task]
pub async fn ui_task(
    mut ui: HubUI<'static, DisplayType>,
    mut gpio_ui_up: ExtiInput<'static>,
    mut gpio_ui_down: ExtiInput<'static>,
    mut gpio_ui_enter: ExtiInput<'static>,
    shutdown: alloc::rc::Rc<core::cell::RefCell<bool>>,
    reboot: alloc::rc::Rc<core::cell::RefCell<bool>>,
) {
    use embassy_futures::select::{select3, Either3};
    use embassy_time::{Duration, Timer};

    loop {
        let ui_event = select3(
            gpio_ui_up.wait_for_any_edge(),
            gpio_ui_down.wait_for_any_edge(),
            gpio_ui_enter.wait_for_any_edge(),
        );

        let timer_future = Timer::after(Duration::from_millis(500));

        match embassy_futures::select::select(ui_event, timer_future).await {
            embassy_futures::select::Either::First(result) => {
                let event = match result {
                    Either3::First(_) if gpio_ui_up.is_high() => Event::KeyDown(EventKey::Up),
                    Either3::Second(_) if gpio_ui_down.is_high() => Event::KeyDown(EventKey::Down),
                    Either3::Third(_) if gpio_ui_enter.is_high() => Event::KeyDown(EventKey::Enter),
                    _ => Event::None,
                };

                let display = ui.update(&event);
                let _ = display.flush();

                if *shutdown.borrow() {
                    shutdown.replace(false);
                    crate::communication::send_system_command(true, false).await;
                }

                if *reboot.borrow() {
                    reboot.replace(false);
                    crate::communication::send_system_command(false, true).await;
                }
            }
            embassy_futures::select::Either::Second(_) => {
                let display = ui.update(&Event::None);
                let _ = display.flush();
            }
        }
    }
}
