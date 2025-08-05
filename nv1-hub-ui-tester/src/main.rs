#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[cfg_attr(target_os = "none", panic_handler)]
#[cfg(target_os = "none")]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(not(target_os = "none"))]
use core::cell::RefCell;

#[cfg(not(target_os = "none"))]
static G_SHUTDOWN: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[cfg(not(target_os = "none"))]
static G_REBOOT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[cfg(not(target_os = "none"))]
fn main() -> anyhow::Result<()> {
    use std::sync::Mutex;
    use embedded_graphics::{pixelcolor::BinaryColor, prelude::*};
    use embedded_graphics_simulator::{
        sdl2::Keycode, BinaryColorTheme, OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent,
        Window,
    };use core::cell::RefCell;
    use nv1_hub_ui::menu::Menu;
    use nv1_hub_ui::menu::RobotStatusMenu;
    use nv1_hub_ui::menu::RobotStatusMenuOption;
    use nv1_hub_ui::{elements, menus};
    use nv1_hub_ui::{
        elements::{Button, Element, Text, Value},
        menu::{ListMenu, ListMenuOption},
        Event, EventKey, HubUI, HubUIOption,
    };

    println!("Hello, world!");

    let mut display = SimulatorDisplay::<BinaryColor>::new(Size::new(128, 64));
    let output_settings = OutputSettingsBuilder::new()
        .theme(BinaryColorTheme::OledWhite)
        .build();
    let mut window = Window::new("SSD1306", &output_settings);

    let ui_option = HubUIOption {};

    let ui_text = Text::new("Interface", embedded_graphics::mono_font::ascii::FONT_5X8);

    let ui_shutdown = Button::new(
        "Shutdown",
        |pressed| {
            G_SHUTDOWN.lock().as_mut().unwrap().replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_5X8,
    );

    let ui_reboot = Button::new(
        "Reboot",
        |pressed| {
            G_REBOOT.lock().as_mut().unwrap().replace(pressed);
        },
        embedded_graphics::mono_font::ascii::FONT_5X8,
    );

    let ui_line = Value::new(
        "L",
        0.0,
        |value| {
            *value += 0.1;
        },
        embedded_graphics::mono_font::ascii::FONT_5X8,
    );

    let elements = elements![
        SimulatorDisplay<BinaryColor>,
        ui_text,
        ui_shutdown,
        ui_reboot,
        ui_line
    ];

    let menu = menus![
        SimulatorDisplay<BinaryColor>,
        ListMenu::new(
            elements,
            ListMenuOption {
                position: Point::new(64, 0),
                size: Size::new(64, 64),
                vertical_num: 5,
                element_margin: 1,
                cursor_line_len: 4,
            },
        ),
        RobotStatusMenu::new(RobotStatusMenuOption {
            position: Point::new(8, 8),
            size: Size::new(48, 48),
        })
    ];
    let mut ui = HubUI::new(&mut display, menu, ui_option);

    let mut ui_event = Event::None;

    'running: loop {
        let mut display = ui.update(&ui_event);
        window.update(&mut display);

        ui_event = Event::None;
        for event in window.events() {
            match event {
                SimulatorEvent::Quit => break 'running,
                SimulatorEvent::KeyDown { keycode, .. } => {
                    println!("Key pressed: {:?}", keycode);

                    ui_event = match keycode {
                        Keycode::Up => Event::KeyDown(EventKey::Up),
                        Keycode::Down => Event::KeyDown(EventKey::Down),
                        Keycode::Return => Event::KeyDown(EventKey::Enter),
                        _ => Event::None,
                    };
                }
                SimulatorEvent::KeyUp { keycode, .. } => {
                    println!("Key released: {:?}", keycode);

                    ui_event = match keycode {
                        Keycode::Up => Event::KeyUp(EventKey::Up),
                        Keycode::Down => Event::KeyUp(EventKey::Down),
                        Keycode::Return => Event::KeyUp(EventKey::Enter),
                        _ => Event::None,
                    };
                }
                _ => {
                    ui_event = Event::None;
                }
            }
        }
    }

    Ok(())
}
