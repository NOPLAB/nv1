use crate::fmt::info;
use defmt::error;
use embassy_stm32::flash::{Blocking, Flash};
use nv1_msg::hub::HSV;
use serde::{Deserialize, Serialize};

use crate::constants::*;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub enum GoalColor {
    #[default]
    Blue,
    Yellow,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, Default)]
pub struct Settings {
    pub line_threshold: f32,
    pub have_ball_threshold: u16,
    pub opp_goal_color: GoalColor,
    pub opencv_goal_blue: HSV,
    pub opencv_goal_yellow: HSV,
    pub robot_speed_multiplier: f32,
}

impl Settings {
    pub const DEFAULT: Self = Settings {
        line_threshold: DEFAULT_LINE_THRESHOLD,
        have_ball_threshold: DEFAULT_HAVE_BALL_THRESHOLD,
        robot_speed_multiplier: DEFAULT_ROBOT_SPEED_MULTIPLIER,
        opp_goal_color: GoalColor::Blue,
        opencv_goal_blue: DEFAULT_OPENCV_GOAL_BLUE,
        opencv_goal_yellow: DEFAULT_OPENCV_GOAL_YELLOW,
    };

    pub fn get_opp_color(&self) -> HSV {
        match self.opp_goal_color {
            GoalColor::Blue => self.opencv_goal_blue,
            GoalColor::Yellow => self.opencv_goal_yellow,
        }
    }

    pub fn get_own_color(&self) -> HSV {
        match self.opp_goal_color {
            GoalColor::Blue => self.opencv_goal_yellow,
            GoalColor::Yellow => self.opencv_goal_blue,
        }
    }

    pub fn toggle_goal_color(&mut self) {
        self.opp_goal_color = match self.opp_goal_color {
            GoalColor::Blue => GoalColor::Yellow,
            GoalColor::Yellow => GoalColor::Blue,
        };
    }
}

pub fn flash_read(f: &mut Flash<'_, Blocking>) -> Result<Settings, embassy_stm32::flash::Error> {
    let mut buf = [0u8; SETTINGS_BUFFER_SIZE];
    f.blocking_read(FLASH_SETTINGS_ADDRESS, &mut buf)?;

    info!("flash read: {:?}", buf);

    let decoded = match postcard::from_bytes(&buf) {
        Ok(d) => d,
        Err(_) => {
            error!("flash read error");
            Settings::DEFAULT
        }
    };

    Ok(decoded)
}

pub fn flash_write(
    f: &mut Flash<'_, Blocking>,
    settings: &Settings,
) -> Result<(), embassy_stm32::flash::Error> {
    let mut buf = [0u8; SETTINGS_BUFFER_SIZE];
    postcard::to_slice(settings, &mut buf).unwrap();

    f.blocking_erase(
        FLASH_SETTINGS_ADDRESS,
        FLASH_SETTINGS_ADDRESS + FLASH_SETTINGS_SIZE,
    )?;
    f.blocking_write(FLASH_SETTINGS_ADDRESS, &buf)?;

    Ok(())
}

pub fn validate_and_fix_settings(
    settings: &mut Settings,
    flash: &mut Flash<'_, Blocking>,
) -> Result<(), embassy_stm32::flash::Error> {
    let mut needs_save = false;

    if settings.line_threshold.is_nan() {
        settings.line_threshold = DEFAULT_LINE_THRESHOLD;
        needs_save = true;
    }

    if needs_save {
        flash_write(flash, settings)?;
    }

    Ok(())
}
