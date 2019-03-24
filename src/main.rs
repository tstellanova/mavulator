
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration};

use mavulator::*;

use uorb_codec::common::*;
use connection::UorbConnection;
//use crate::simulator::VehicleState;

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
    let mut simulato = Simulato::new();
    let shared_simulato = Arc::new(RwLock::new(simulato));

    //don't create the shared state object until after we've connected
    //let shared_vehicle_state = Arc::new(RwLock::new(simulator::initial_vehicle_state()));

    thread::spawn({
        let conn:Arc<Box<UorbConnection+Send+Sync>> = vehicle_conn.clone();
        let simulato_state = shared_simulato.clone();
        move || {
            simulato_loop(simulato_state, conn);
        }
//        let vehicle_state:Arc<RwLock<VehicleState>> = shared_vehicle_state.clone();
//        move || {
//            simulator_loop(simulato_state, conn);
//        }
    });

    loop {
        match vehicle_conn.recv() {
            Ok((_header, msg)) => {
                match msg {
                    UorbMessage::ActuatorOutputs(_m) => {
                        //TODO provide actuator to simulato
                        //println!("time: {}", m.timestamp);
                    },
                    UorbMessage::VehicleStatus(_m) => {
                        //TODO provide actuator to simulato?
                        //println!("time: {}", m.timestamp);
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

