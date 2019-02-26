
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration};

use mavulator::*;

use uorb_codec::common::*;

fn main() {
    println!("starting");

    let shared_vehicle_state = Arc::new(RwLock::new(simulator::initial_vehicle_state()));
    let selector = "tcpout:127.0.0.1:4560";
    let conn = connection::select_protocol(selector);
    if !conn.is_ok() {
        println!("Couldn't connect...terminating");
        return;
    }

    let vehicle_conn = Arc::new(conn.unwrap());
    println!("connected");

    thread::spawn({
        let conn = vehicle_conn.clone();
        move || {
            let mut last_slow_cadence_send: u64 = 0;
            let vehicle_state = shared_vehicle_state.clone();
            loop {
                //Fast cadence loop, 1KHz approx
                for _i in 0..1000 {
                    let mut state_w = vehicle_state.write().unwrap();
                    let res = send_fast_cadence_sensors(  &**conn, &mut state_w );
                    if res.is_err() {
                        println!("send_fast_cadence_sensors failed: {:?}",res);
                        return;
                    }

                    if state_w.elapsed_since(last_slow_cadence_send) > 1000000 {
                        last_slow_cadence_send = state_w.get_simulated_usecs();
                        let res = send_slow_cadence_sensors(&**conn, &mut state_w);
                        if res.is_err()  {
                            println!("send_fast_cadence_sensors failed: {:?}",res);
                            return;
                        }
                    }
                }
                //thread::yield_now();
                thread::sleep(Duration::from_millis(10));
            }
        }
    });

    loop {
        match vehicle_conn.recv() {
            Ok((_header, msg)) => {
                match msg {
                    UorbMessage::ActuatorOutputs(_m) => {
                        //println!("time: {}", m.timestamp);
                    },
                    UorbMessage::VehicleStatus(_m) => {

                    },
                    _ => {
                        println!("recv: {:?}", msg);
                    }
                }
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
