#[macro_use]
extern crate criterion;

use criterion::Criterion;

use std::thread;
use uorb_codec::common::*;
use uorb_codec::{UorbMsgMeta};
use std::time::{Duration};

fn get_vehicle_status() -> uorb_codec::common::VehicleStatusData {
    uorb_codec::common::VehicleStatusData {
        timestamp: 83838333,
        nav_state: 14,
        arming_state: 15,
        hil_state: 16,
        failsafe: false,
        system_type: 99,
        system_id: 51,
        component_id: 49,
        is_rotary_wing: true,
        is_vtol: true,
        vtol_fw_permanent_stab: true,
        in_transition_mode: false,
        in_transition_to_fw: false,
        rc_signal_lost: false,
        rc_input_mode: 12,
        data_link_lost: false,
        high_latency_data_link_active: false,
        data_link_lost_counter: 0,
        engine_failure: false,
        mission_failure: false,
        failure_detector_status: 0,
        onboard_control_sensors_present: 32,
        onboard_control_sensors_enabled: 2097167,
        onboard_control_sensors_health: 32,
    }
}

pub fn run_tcp_loopback() {
    const RECEIVE_CHECK_COUNT: i32 = 5;

    let server_thread = thread::spawn( {
        move || {
            let server = mavulator::connection::select_protocol("tcpin:0.0.0.0:14570")
                .expect("Couldn't create server");

            let mut recv_count = 0;
            for _i in 0..RECEIVE_CHECK_COUNT {
                match server.recv() {
                    Ok((_header, msg)) => {
                        match msg {
                            uorb_codec::common::UorbMessage::VehicleStatus(_vehicle_status_msg) => {
                                recv_count += 1;

                            },
                            _ => {
                                // one message parse failure fails the test
                                break;
                            }
                        }
                    }
                    Err(..) => {
                        // one message read failure fails the test
                        break;
                    }
                }
            }
            assert_eq!(recv_count, RECEIVE_CHECK_COUNT);
        }
    });

    thread::sleep(Duration::from_millis(10));

    // have the client send a few hearbeats
    thread::spawn({
        move || {
            let msg_data = get_vehicle_status();
            let msg = uorb_codec::common::UorbMessage::VehicleStatus( msg_data.clone() );
            let header = uorb_codec::UorbHeader {
                version: uorb_codec::UORB_MAGIC_V1,
                hash: VehicleStatusData::MSG_HASH_CODE,
                timestamp: 666,
                instance_id: 17,
                payload_len: VehicleStatusData::ENCODED_LEN,
            };
            let client = mavulator::connection::select_protocol("tcpout:127.0.0.1:14570")
                .expect("Couldn't create client");
            for _i in 0..RECEIVE_CHECK_COUNT {
                client.send(&header,&msg).ok();
            }
        }
    });

    server_thread.join().unwrap();
}


fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("mavu_loopback", |b| b.iter(|| run_tcp_loopback()));
}


criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);