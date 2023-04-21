use crate::config::Config;
use crate::tasks::mqtt::MqttFormat;
use embassy_stm32::can::bxcan::Frame;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as _Mutex;
use embassy_sync::{channel::Channel, mutex::Mutex, signal::Signal};
// use embassy_time::Instant;

pub type InverterChannelRx = Channel<_Mutex, Frame, 2>;
pub type InverterChannelTx = Channel<_Mutex, Frame, 20>;
pub type BmsChannelRx = Channel<_Mutex, Frame, 20>;
pub type BmsChannelTx = Channel<_Mutex, Frame, 20>;
// pub type Elapsed = Mutex<_Mutex, Instant>;
pub type MutexBool = Mutex<_Mutex, bool>;
pub type MqttFmtMutex = embassy_sync::mutex::Mutex<_Mutex, MqttFormat>;
pub type ConfigType = embassy_sync::mutex::Mutex<_Mutex, Config>;
pub type Status = Signal<_Mutex, bool>;
