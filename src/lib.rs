
use std::io::Error;


pub mod connection;
pub mod simulator;

use connection::UorbConnection;
use crate::simulator::VehicleState;



pub fn send_slow_cadence_sensors(conn: &UorbConnection , state: &mut VehicleState) -> Result<(), Error> {
    let msg_list = simulator::gen_slow_cadence_sensors(state);
    for (hdr, msg) in msg_list {
        conn.send(&hdr, &msg)?;
    }
    Ok(())
}

pub fn send_fast_cadence_sensors(conn: &UorbConnection , state: &mut VehicleState) -> Result<(), Error> {
    let msg_list = simulator::gen_fast_cadence_sensors(state);
    for (hdr, msg) in msg_list {
        conn.send(&hdr, &msg)?;
    }
    Ok(())
}