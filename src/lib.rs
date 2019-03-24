
use std::io::Error;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration};
use uorb_codec::{UorbHeader, UorbMessage};

pub mod connection;
//pub mod simulator;
pub mod sim_wrapper;

use connection::UorbConnection;
//use crate::simulator::VehicleState;
use flighty::simulato::Simulato;


pub fn send_all_messages( conn: &UorbConnection,  msg_list: Vec<(UorbHeader, UorbMessage)>) -> Result<(), Error> {
    for (hdr, msg) in msg_list {
        conn.send(&hdr, &msg)?;
    }
    Ok(())
}

/// Gyro rate should be 400 Hz
/// Accel rate should be 400 Hz
/// Mag rate should be 100 Hz
/// Baro rate should be 100 Hz
/// Airspeed should be 100 Hz

//pub fn simulator_loop(vehicle_state:Arc<RwLock<VehicleState>> , conn:Arc<Box<UorbConnection+Send+Sync>>) {
//    {
//        let mut state_w = vehicle_state.write().unwrap();
//        //send a first message to establish a time base (abs time offset)
//        let (hdr, msg) = simulator::gen_wrapped_timesync_status(&mut state_w);
//        let first_msg = vec![(hdr, msg)];
//        let res = send_all_messages(&**conn, first_msg);
//        if res.is_err() {
//            println!("first send failed: {:?}", res);
//            return;
//        }
//    }
//
//    let mut last_slow_cadence_send: u64 = 0; //1Hz sensors
//    let mut last_med_cadence_send: u64 = 0; //100Hz sensors
//    let mut last_fast_cadence_send: u64 = 0; //400Hz sensors
//    loop {
//        // TODO modfy this when we decouple from realtime clock
//        //thread::yield_now();
//        thread::sleep(Duration::from_micros(100));
//        let mut msg_list: Vec<(UorbHeader, UorbMessage)> = vec![];
//        {
//            let mut state_w = vehicle_state.write().unwrap();
//            simulator::increment_simulated_time(&mut state_w);
//
//            //Fast cadence: 400Hz approx
//            if (0 ==  last_fast_cadence_send) ||
//                (state_w.elapsed_since(last_fast_cadence_send) > 2500) {
//                let msgs = simulator::gen_fast_cadence_sensors(&mut state_w);
//                msg_list.extend(msgs);
//                last_fast_cadence_send = state_w.get_simulated_usecs();
//            }
//
//            // Medium cadence: about 100Hz  10000 usec
//            if (0 ==  last_med_cadence_send) ||
//                (state_w.elapsed_since(last_med_cadence_send) > 10000) {
//                let msgs = simulator::collect_med_cadence_sensors(&mut state_w);
//                msg_list.extend(msgs);
//                last_med_cadence_send = state_w.get_simulated_usecs();
//            }
//
//            //Slow cadence: 1Hz approx
//            if (0 ==  last_slow_cadence_send) ||
//                (state_w.elapsed_since(last_slow_cadence_send) > 100000) {
//                let msgs = simulator::collect_slow_cadence_sensors(&mut state_w);
//                msg_list.extend(msgs);
//                last_slow_cadence_send = state_w.get_simulated_usecs();
//            }
//        }
//
//        if msg_list.len() > 0 {
//            //send all messages
//            let res = send_all_messages(&**conn, msg_list);
//            if res.is_err() {
//                println!("sending failed: {:?}", res);
//                return;
//            }
//        }
//
//    }
//}



pub fn simulato_loop(sim:Arc<RwLock<Simulato>> , conn:Arc<Box<UorbConnection+Send+Sync>>) {
    {
        let mut state_w = sim.write().unwrap();
        //send a first message to establish a time base (abs time offset)
        let (hdr, msg) = sim_wrapper::gen_wrapped_timesync_status(&mut state_w);
        let first_msg = vec![(hdr, msg)];
        let res = send_all_messages(&**conn, first_msg);
        if res.is_err() {
            println!("first send failed: {:?}", res);
            return;
        }
    }

    let mut last_slow_cadence_send: u64 = 0; //1Hz sensors
    let mut last_med_cadence_send: u64 = 0; //100Hz sensors
    let mut last_fast_cadence_send: u64 = 0; //400Hz sensors
    loop {
        // TODO modfy this when we decouple from realtime clock
        //thread::yield_now();
        thread::sleep(Duration::from_micros(100));
        let mut msg_list: Vec<(UorbHeader, UorbMessage)> = vec![];
        {
            let mut state_w = sim.write().unwrap();
            sim_wrapper::increment_simulated_time(&mut state_w);

            //Fast cadence: 400Hz approx
            if (0 ==  last_fast_cadence_send) ||
                (state_w.elapsed_since(last_fast_cadence_send) > 2500) {
                let msgs = sim_wrapper::collect_fast_cadence_sensors(&mut state_w);
                msg_list.extend(msgs);
                last_fast_cadence_send = state_w.get_simulated_time();
            }

            // Medium cadence: about 100Hz  10000 usec
            if (0 ==  last_med_cadence_send) ||
                (state_w.elapsed_since(last_med_cadence_send) > 10000) {
                let msgs = sim_wrapper::collect_med_cadence_sensors(&mut state_w);
                msg_list.extend(msgs);
                last_med_cadence_send = state_w.get_simulated_time();
            }

            //Slow cadence: 1Hz approx
            if (0 ==  last_slow_cadence_send) ||
                (state_w.elapsed_since(last_slow_cadence_send) > 100000) {
                let msgs = sim_wrapper::collect_slow_cadence_sensors(&mut state_w);
                msg_list.extend(msgs);
                last_slow_cadence_send = state_w.get_simulated_time();
            }
        }

        if msg_list.len() > 0 {
            //send all messages
            let res = send_all_messages(&**conn, msg_list);
            if res.is_err() {
                println!("sending failed: {:?}", res);
                return;
            }
        }

    }
}



