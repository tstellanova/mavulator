
use std::io::Error;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration};


pub mod connection;
pub mod simulator;

use connection::UorbConnection;
use crate::simulator::VehicleState;



pub fn send_slow_cadence_sensors(state: &mut VehicleState, conn: &UorbConnection ) -> Result<(), Error> {
    let msg_list = simulator::gen_slow_cadence_sensors(state);
    for (hdr, msg) in msg_list {
        conn.send(&hdr, &msg)?;
    }
    Ok(())
}

pub fn send_fast_cadence_sensors( state: &mut VehicleState, conn: &UorbConnection) -> Result<(), Error> {
    let msg_list = simulator::gen_fast_cadence_sensors(state);
    for (hdr, msg) in msg_list {
        conn.send(&hdr, &msg)?;
    }
    Ok(())
}

pub fn simulator_loop(vehicle_state:Arc<RwLock<VehicleState>> , conn:Arc<Box<UorbConnection+Send+Sync>>) {
    let mut last_slow_cadence_send: u64 = 0;
    loop {
        //Fast cadence loop, 1KHz approx
        for _i in 0..500 {
            let mut state_w = vehicle_state.write().unwrap();
            let res = send_fast_cadence_sensors(  &mut state_w, &**conn );
            if res.is_err() {
                println!("send_fast_cadence_sensors failed: {:?}",res);
                return;
            }

            if state_w.elapsed_since(last_slow_cadence_send) > 1000000 {
                last_slow_cadence_send = state_w.get_simulated_usecs();
                let res = send_slow_cadence_sensors(&mut state_w, &**conn);
                if res.is_err()  {
                    println!("send_fast_cadence_sensors failed: {:?}",res);
                    return;
                }
            }
        }
        thread::yield_now();
//        thread::sleep(Duration::from_millis(5));
    }
}



