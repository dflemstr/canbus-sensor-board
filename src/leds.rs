use crate::config;
use embassy_stm32::gpio;
use embassy_sync::blocking_mutex::raw;
use embassy_sync::mutex;

pub type Led = mutex::Mutex<raw::ThreadModeRawMutex, gpio::Output<'static>>;

/// Convenience wrapper around the available LEDs.
#[derive(Clone, Copy)]
pub struct Leds<'a> {
    led: &'a Led,
}

impl<'a> Leds<'a> {
    pub fn new(led: &'a Led) -> Self {
        Self { led }
    }
}

impl Leds<'static> {
    pub fn show_error_code(&self, spawner: embassy_executor::Spawner, blinks: u32) {
        defmt::trace!("show_error_code blinks={=u32}", blinks);
        if let Err(err) = spawner.spawn(show_error_code(*self, blinks)) {
            defmt::warn!("could not spawn show_error_code: {}", err);
        }
    }

    pub async fn blink_watchdog(&self, level: bool) {
        defmt::trace!("blink_watchdog level={=bool}", level);
        let mut led = self.led.lock().await;
        if level {
            led.set_level(gpio::Level::High);
        } else {
            led.set_level(gpio::Level::Low);
        }
    }
}

#[embassy_executor::task(pool_size = 4)]
async fn show_error_code(leds: Leds<'static>, blinks: u32) {
    // Lock LED so that other concurrent LED patterns don't mess up our error code.
    let mut led = leds.led.lock().await;
    for _ in 0..config::ERROR_NUM_BLINK_GROUPS {
        for _ in 0..blinks {
            led.set_level(gpio::Level::High);
            embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINKS).await;
            led.set_level(gpio::Level::Low);
            embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINKS).await;
        }
        embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINK_GROUPS).await;
    }
}
