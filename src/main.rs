
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration};

use mavulator::*;

use uorb_codec::common::*;
use connection::UorbConnection;

use flighty::simulato::Simulato;

fn main() {
    println!("starting");

    let selector = "tcpout:127.0.0.1:4560";
    //let selector = "tcpout:rock64-04.local:4560";
    let conn = connection::select_protocol(selector);
    if !conn.is_ok() {
        println!("Couldn't connect...terminating");
        return;
    }

    let vehicle_conn = Arc::new(conn.unwrap());
    println!("connected");

    //don't create the shared state object until after we've connected
    let shared_simulato:Arc<RwLock<Simulato>> = Arc::new(RwLock::new(Simulato::new()));

    // start a thread reporting the physical state of the simulated vehicle
    thread::spawn({
        let conn:Arc<Box<UorbConnection+Send+Sync>> = vehicle_conn.clone();
        let simulato_state = shared_simulato.clone();
        move || {
            reporting_loop(simulato_state, conn);
        }
    });

    // loop receiving messages from the mav firmware itself
    loop {
        match vehicle_conn.recv() {
            Ok((header, msg)) => {
                match msg {
                    UorbMessage::ActuatorOutputs(m) => {
                        let controls = normalize_actuator_outputs(&m.output, 4);
                        let mut state_w = shared_simulato.write().unwrap();
                        //println!("msg time: {}", header.timestamp);
                        state_w.update(header.timestamp, &controls);
                    },
                    UorbMessage::VehicleStatus(_m) => {
                        //TODO provide to simulato?
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
                        println!("recv error: {:?}", e);
                        break;
                    }
                }
            }
        }
    }
}

