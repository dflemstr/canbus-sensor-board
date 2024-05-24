use crate::config;
use embassy_stm32::gpio;
use embassy_sync::blocking_mutex::raw;
use embassy_sync::mutex;

pub type Led = mutex::Mutex<raw::ThreadModeRawMutex, gpio::Output<'static>>;

/// Convenience wrapper around the available LEDs.
#[derive(Clone, Copy)]
pub struct Leds<'a> {
    g_led: &'a Led,
    y_led: &'a Led,
}

impl<'a> Leds<'a> {
    pub fn new(
        g_led: &'a Led,
        y_led: &'a Led,) -> Self {
        Self {
            g_led,
            y_led,
        }
    }
}

impl Leds<'static> {
    pub fn show_error_code(&self, spawner: embassy_executor::Spawner, blinks: u32) {
        if let Err(err) = spawner.spawn(show_error_code(*self, blinks)) {
            defmt::warn!("could not spawn show_error_code: {}", err);
        }
    }

    pub async fn blink_watchdog(&self) {
        let mut g_led = self.g_led.lock().await;
        g_led.set_level(gpio::Level::High);
        embassy_time::Timer::after(config::WATCHDOG_DELAY_BETWEEN_BLINKS).await;
        g_led.set_level(gpio::Level::Low);
        embassy_time::Timer::after(config::WATCHDOG_DELAY_BETWEEN_BLINKS).await;
    }
}

#[embassy_executor::task(pool_size = 4)]
async fn show_error_code(leds: Leds<'static>, blinks: u32) {
    // Lock LED so that other concurrent LED patterns don't mess up our error code.
    let mut y_led = leds.y_led.lock().await;
    for _ in 0..config::ERROR_NUM_BLINK_GROUPS {
        for _ in 0..blinks {
            y_led.set_level(gpio::Level::High);
            embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINKS).await;
            y_led.set_level(gpio::Level::Low);
            embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINKS).await;
        }
        embassy_time::Timer::after(config::ERROR_DELAY_BETWEEN_BLINK_GROUPS).await;
    }
}
