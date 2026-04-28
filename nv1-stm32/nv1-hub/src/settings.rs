use crate::fmt::info;
use defmt::error;
use embassy_stm32::flash::{Blocking, Flash};

use crate::constants::*;

pub use nv1_hub_core::settings::{GoalColor, Settings};

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
    if settings.sanitize() {
        flash_write(flash, settings)?;
    }
    Ok(())
}
