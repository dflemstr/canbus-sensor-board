use crate::leds;

pub trait Signaller {
    async fn signal_error(&self, error_kind: ErrorKind);
}

#[repr(u32)]
pub enum ErrorKind {
    Can = 2,
    Sensors = 3,
}

#[derive(Clone, Copy)]
pub struct LedsSignaller {
    leds: leds::Leds<'static>,
    spawner: embassy_executor::Spawner,
}

impl LedsSignaller {
    pub fn new(leds: leds::Leds<'static>, spawner: embassy_executor::Spawner) -> Self {
        Self { leds, spawner }
    }
}

impl Signaller for LedsSignaller {
    async fn signal_error(&self, error_kind: ErrorKind) {
        self.leds.show_error_code(self.spawner, error_kind as u32);
    }
}
