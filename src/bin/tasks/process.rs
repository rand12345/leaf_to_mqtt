use crate::statics::{
    CAR_CAN_RX, CAR_CAN_TX, EV_CAN_RX, EV_CAN_TX, LAST_READING_TIMEOUT_SECS, MQTTFMT, SEND_MQTT,
};

use defmt::Format;
use embassy_futures::select::{select3, Either3::*};
use embassy_stm32::can::bxcan::{Frame, Id, StandardId};
use embassy_time::{Duration, Instant, Timer};

#[embassy_executor::task]
pub async fn parse() {
    let _tx_car_can = CAR_CAN_TX.sender();
    let tx_ev_can = EV_CAN_TX.sender();
    let rx_car_can = CAR_CAN_RX.receiver();
    let rx_ev_can = EV_CAN_RX.receiver();
    defmt::warn!("Starting parser");
    let canid = |frame: &Frame| -> u16 {
        match frame.id() {
            embassy_stm32::can::bxcan::Id::Standard(id) => id.as_raw(),
            embassy_stm32::can::bxcan::Id::Extended(_) => 0,
        }
    };
    //02
    // 21 (= read block command?)
    // "01: ??? (6 lines)
    // 02: cellpair data (29 lines)
    // 03: Vmin, Max, ??? (5 lines)
    // 04: Temperature (3 lines)
    // 05: ??? (11 lines)
    // 06: balancing shunts"

    let init_frame = |thismode: Mode| {
        let m = thismode as u8;
        Frame::new_data(
            Id::Standard(StandardId::new(0x79b).unwrap()),
            [0x02, 0x21, m, 0xff, 0xff, 0xff, 0xff, 0xff],
        )
    };
    let continue_frame = || {
        Frame::new_data(
            Id::Standard(StandardId::new(0x79b).unwrap()),
            [0x30, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff],
        )
    };
    let mut mode = Mode::IdleCheck;
    let mut isotp_bytes: heapless::Vec<u8, 255> = heapless::Vec::new();
    let mut cell_mv: heapless::Vec<u16, 96> = heapless::Vec::new();
    let mut tick = embassy_time::Ticker::every(embassy_time::Duration::from_secs(
        LAST_READING_TIMEOUT_SECS / 5,
    ));
    let mut last_rx = Instant::now();
    let mut amps = 0.0;
    loop {
        let (frame, can) = match select3(rx_car_can.recv(), rx_ev_can.recv(), tick.next()).await {
            First(frame) => (frame, Can::Car),
            Second(frame) => (frame, Can::Ev),
            Third(_) => {
                mode = match mode {
                    Group1 => CellPairs,
                    CellPairs => Temps,
                    Temps => Bal,
                    Bal => IdleCheck,
                    IdleCheck => Group1,
                };

                if matches!(mode, IdleCheck) {
                    if last_rx.elapsed().as_secs() < LAST_READING_TIMEOUT_SECS {
                        let mut mqtt = MQTTFMT.lock().await;
                        mqtt.sleeping = false;
                        SEND_MQTT.signal(true);
                        continue;
                    }
                    Timer::after(Duration::from_secs(LAST_READING_TIMEOUT_SECS)).await;

                    let mut mqtt = MQTTFMT.lock().await;
                    if !mqtt.sleeping {
                        mqtt.sleeping = true;
                        SEND_MQTT.signal(true);
                    }
                    mode = Bal; // shutdown all tx, loop back to awaiting an rx
                    continue;
                }
                if let Err(_) = tx_ev_can.try_send(init_frame(mode)) {
                    defmt::error!("EV CAN tx queue error")
                };
                continue;
            }
        };
        last_rx = Instant::now();
        let id = canid(&frame);
        let data = frame.data().unwrap();
        if id == 0x5b9 {
            defmt::info!("0x5b9 {}: {:x}", can, data);
        }

        if id == 0x1db {
            let d = &data[..2];
            let mut nl_battery_current = ((d[0] as u16) << 3) | ((d[1] & 0xe0) >> 5) as u16;
            if nl_battery_current & 0x0400 != 0 {
                // negative so extend the sign bit
                nl_battery_current |= 0xf800;
            }
            // sign updated to match standard metric definition where battery output is positive
            amps = (nl_battery_current as i16 as f32) / 2.0;
            // defmt::info!("Amps: {}", amps)
        }
        if matches!(can, Can::Car) {
            continue;
        }
        // process ISO-TP from EV can only
        if id != 0x7bb {
            continue;
        };
        use Mode::*;
        if data[0] == 0x10 {
            isotp_bytes.clear(); // clear byte buffer
        };

        let bytes = match mode {
            IdleCheck => continue, //panic!("Attempted to process can data with sleep mode active"),
            Temps | CellPairs | Bal => {
                if data[0] == 0x10 {
                    Some(&data[4..8])
                } else {
                    Some(&data[1..8])
                }
            }
            Group1 => match data[0] {
                0x23 => {
                    if let Some(array) = data[3..5].try_into().ok() {
                        let acc = u16::from_be_bytes(array) as f32 / 1024.0;
                        MQTTFMT.lock().await.acc = acc;
                    };
                    None
                }
                0x24 => {
                    if let Some(array) = data[4..8].try_into().ok() {
                        let soc = u32::from_be_bytes(array) as f32 / 10000.0;
                        MQTTFMT.lock().await.soc = soc;
                    };
                    None
                }
                _ => None,
            },
        };

        if let Err(_) = tx_ev_can.try_send(continue_frame()) {
            defmt::error!("EV tx2 error")
        }

        if bytes.is_none() {
            continue;
        };

        let bytes = bytes.unwrap();

        let _ = isotp_bytes.extend_from_slice(bytes);

        if matches!(mode, CellPairs) {
            if isotp_bytes.len() == 200 {
                cell_mv.clear();
                // defmt::warn!("{} {:x}", isotp_bytes.len(), Debug2Format(&isotp_bytes));
                isotp_bytes
                    .chunks(2)
                    .take(96)
                    .into_iter()
                    .filter(|l| l.len() == 2)
                    .map(|v| u16::from_be_bytes([v[0], v[1]]))
                    .for_each(|mv| cell_mv.push(mv).unwrap());
                // defmt::warn!("#{} {:?}", cell_mv.len(), defmt::Debug2Format(&cell_mv));

                let mut mqtt = MQTTFMT.lock().await;
                mqtt.cell_mv_high = *cell_mv.iter().min().unwrap() as f32;
                mqtt.cell_mv_low = *cell_mv.iter().max().unwrap() as f32;
                mqtt.volts = u16::from_be_bytes([isotp_bytes[192], isotp_bytes[193]])
                    .saturating_div(100) as f32;
                mqtt.amps = amps;
                defmt::info!(
                    "Read CellPairs: High {}mV Low {}mV Pack {}V Amps {}A",
                    mqtt.cell_mv_high,
                    mqtt.cell_mv_low,
                    mqtt.volts,
                    mqtt.amps
                )
            }
            continue;
        }
        if matches!(mode, Temps) {
            // [2, 16, 18, 2, 13, 19, 255, 255, 255, 2, 34, 17, 17, 0, 255, 255, 255, 255]
            if isotp_bytes.len() == 18 {
                let mut mqtt = MQTTFMT.lock().await;
                mqtt.cell_temp_high = isotp_bytes[2] as i8 as f32;
                mqtt.cell_temp_low = isotp_bytes[5] as i8 as f32;
                defmt::info!(
                    "Read Temps: High {}ºC Low {}ºC",
                    mqtt.cell_temp_high,
                    mqtt.cell_temp_low
                );
            }
            continue;
        }
        if matches!(mode, Bal) {
            if isotp_bytes.len() == 25 {
                let mut bal_cells = [false; 96];
                isotp_bytes
                    .iter()
                    .flat_map(|v| (0..4u8).map(move |i| v & (1 << i) == 0))
                    .take(96)
                    .enumerate()
                    .for_each(|(idx, bit)| bal_cells[idx] = bit);
                let bal = bal_cells.iter().filter(|b| **b).count();
                let mut mqtt = MQTTFMT.lock().await;
                mqtt.bal = bal as u8;
                defmt::info!("Read Bal cells: {} balancing", bal)
            };
            continue;
        }
    }
}

#[derive(Copy, Clone, Format)]
enum Can {
    Ev,
    Car,
}
#[derive(Format, Debug, Copy, Clone)]
enum Mode {
    Group1 = 1,
    CellPairs = 2,
    // MinMax = 3,
    Temps = 4,
    Bal = 6,
    IdleCheck = 15, //minutes
}
