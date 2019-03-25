
use std::sync::{Arc, RwLock};
use std::thread;

use mavulator::*;

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
    let shared_sim:Arc<RwLock<Simulato>> = Arc::new(RwLock::new(Simulato::new()));

    // start a thread reporting the vehicle's simulated physical state back to the mav firmware
    thread::spawn({
        let conn:Arc<Box<UorbConnection+Send+Sync>> = vehicle_conn.clone();
        let sim = shared_sim.clone();
        move || {
            mav_writer::reporting_loop(sim, conn);
        }
    });

    // this thread loops forever receiving state messages from the mav firmware and
    // forwarding to the physical simulator
    mav_reader::feedback_loop(shared_sim, vehicle_conn);

}

