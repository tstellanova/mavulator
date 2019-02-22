
use std::sync::Arc;
use std::thread;
use std::time::{Duration};

use mavulator::*;



fn main() {
    println!("starting");

    let shared_vehicle_state = initial_vehicle_state();
    let selector = "tcpout:127.0.0.1:4560";
    let conn = connection::select_protocol(selector).unwrap();
    let vehicle_conn = Arc::new(conn);
    println!("connected");

    thread::spawn({
        let conn = vehicle_conn.clone();
        move || {
            let vehicle_state = shared_vehicle_state.clone();
            loop {
                {
                    let mut state_w = vehicle_state.write().unwrap();
                    //println!("> got write ");
                    increment_simulated_time(&mut state_w);
                    send_slow_cadence_sensors(&**conn, &state_w);
                }

                //High cadence loop, 4KHz approx
                for _i in 0..1000 {
                    let mut state_w = vehicle_state.write().unwrap();
                    //println!("> got write ");
                    increment_simulated_time(&mut state_w);
                    send_fast_cadence_sensors(
                        &**conn,
                        &state_w
                    );
                }

                thread::yield_now();
                //thread::sleep(Duration::from_millis(125));
            }
        }
    });

    loop {
        match vehicle_conn.recv() {
            Ok((_header, msg)) => {
//                println!("received: {:?}", msg);
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
