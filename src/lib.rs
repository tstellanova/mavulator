use std::sync::{Arc, RwLock};

use std::time::{Duration, SystemTime};

use uorb_codec::{self, UorbHeader, UorbMsgMeta};
use uorb_codec::common::*;


pub mod connection;
pub mod simulator;

use connection::UorbConnection;

//physical type definitions
pub type WholeMillimeters = i32;
pub type MetersPerSecond = f32;

pub struct VehicleState {
    boot_time: SystemTime,
    simulated_usecs: u64,

    altitude: WholeMillimeters,
    speed: MetersPerSecond,

}

pub fn initial_vehicle_state() ->  Arc<RwLock<VehicleState>> {
    Arc::new( RwLock::new (VehicleState {
        boot_time: SystemTime::now(),
        simulated_usecs: 0,
        altitude: 0,
        speed: 0.0
    }))
}

pub fn micros_from_duration(duration: &Duration) -> u64 {
    (duration.as_secs() * 1000000) + (duration.subsec_micros() as u64)
}

pub fn increment_simulated_time(state: &mut VehicleState) {
    //TODO allow simulated time to decouple from real time
    let new_real_time = state.boot_time.elapsed().unwrap();
    state.simulated_usecs = micros_from_duration(&new_real_time);
}


//TODO move this wrapping into uorb-codec itself
pub fn gen_wrapped_gps_position_msg(state: &VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = simulator::gen_gps_msg_data(state);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: VehicleGpsPositionData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: VehicleGpsPositionData::ENCODED_LEN
    };
    let msg =  UorbMessage::VehicleGpsPosition(msg_data);
    (hdr, msg)
}

pub fn gen_wrapped_battery_status(state: &VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = simulator::gen_battery_status_data(state);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: BatteryStatusData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: BatteryStatusData::ENCODED_LEN
    };
    let msg =  UorbMessage::BatteryStatus(msg_data);
    (hdr, msg)
}

pub fn gen_wrapped_sensor_gyro(state: &VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = simulator::gen_sensor_gyro_data(state, 2293768);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: SensorGyroData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: SensorGyroData::ENCODED_LEN
    };
    let msg =  UorbMessage::SensorGyro(msg_data);
    (hdr, msg)
}


pub fn send_slow_cadence_sensors(conn: &UorbConnection , state: &VehicleState) {
    let (hdr, msg) = gen_wrapped_gps_position_msg(state);
    let res = conn.send(&hdr, &msg);
    if !res.is_ok() {
        println!("gps send failed: {:?}", res);
    }

    let (hdr, msg) = gen_wrapped_battery_status(state);
    let res = conn.send(&hdr, &msg);
    if !res.is_ok() {
        println!("batt send failed: {:?}", res);
    }

    //print!("sent slow");
}

pub fn send_fast_cadence_sensors(conn: &UorbConnection , state: &VehicleState) {
    let (hdr, msg) = gen_wrapped_sensor_gyro(&state);
    let res = conn.send(&hdr, &msg);
    if !res.is_ok() {
        println!("gyro send failed: {:?}", res);
    }
    //print!("sent fast");

}