
use std::sync::Arc;
use std::thread;
use std::time::{Duration, SystemTime};

use mavulator::connection;
use uorb_codec::{self, UorbHeader, UorbMsgMeta};
use uorb_codec::common::*;

pub struct VehicleState {
    boot_time: SystemTime,
}

fn micros_from_duration(duration: &Duration) -> u64 {
    (duration.as_secs() * 1000000) + (duration.subsec_micros() as u64)
}

fn get_simulated_usec(state: &VehicleState) -> u64 {
    let new_real_time = state.boot_time.elapsed().unwrap();
    micros_from_duration(&new_real_time)
}

fn gen_gps_msg_data(state: &VehicleState) -> VehicleGpsPositionData {
    let timestamp = get_simulated_usec(state);
    VehicleGpsPositionData {
        timestamp: timestamp,
        lat: 0,
        lon: 0,
        alt: 100,
        alt_ellipsoid: 100,
        s_variance_m_s: 3.0,
        c_variance_rad: 2.0,
        fix_type: 3,
        eph: 1.5,
        epv: 1.5,
        hdop: 1.0,
        vdop: 1.0,
        noise_per_ms: 10,
        jamming_indicator: 0,
        vel_m_s: 0.0,
        vel_n_m_s: 0.0,
        vel_e_m_s: 0.0,
        vel_d_m_s: 0.0,
        cog_rad: 0.1,
        vel_ned_valid: true,
        timestamp_time_relative: 0,
        time_utc_usec: timestamp,
        satellites_used: 11,
        heading: 0.001,
        heading_offset: 0.0,
    }
}

//TODO move this wrapping into uorb-codec itself
fn gen_wrapped_gps_position_msg(state: &VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_gps_msg_data(state);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: VehicleGpsPositionData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: VehicleGpsPositionData::ENCODED_LEN
    };
    let msg =  UorbMessage::VehicleGpsPosition(msg_data);
    (hdr, msg)
}

fn main() {
    println!("starting");


    let shared_vehicle_state = Arc::new(
        VehicleState {
            boot_time: SystemTime::now(),
    });

    let selector = "tcpout:127.0.0.1:4560";
    let conn = connection::select_protocol(selector).unwrap();
    let vehicle = Arc::new(conn);

    thread::spawn({
        let vehicle = vehicle.clone();
        let shared_vehicle_state = shared_vehicle_state.clone();
        move || {
            loop {
                let (hdr, msg) = gen_wrapped_gps_position_msg(&shared_vehicle_state);
                let res = vehicle.send(&hdr, &msg);
                if res.is_ok() {
                    thread::sleep(Duration::from_secs(1));
                }
                else {
                    println!("send failed: {:?}", res);
                }
            }
        }
    });

    loop {
        match vehicle.recv() {
            Ok((_header, msg)) => {
                println!("received: {:?}", msg);
            },
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::WouldBlock => {
                        //no messages currently available to receive -- wait a while
                        thread::sleep(Duration::from_secs(1));
                        continue;
                    },
                    _ => {
                        println ! ("recv error: {:?}", e);
                        break;
                    }
                }
            }
        }
    }
}
