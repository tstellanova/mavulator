
use std::io::Error;


pub mod connection;
pub mod simulator;

use connection::UorbConnection;
use crate::simulator::VehicleState;



pub fn send_slow_cadence_sensors(conn: &UorbConnection , state: &mut VehicleState) -> Result<(), Error> {
    let (hdr, msg) = simulator::gen_wrapped_gps_position_msg(state);
    conn.send(&hdr, &msg)?;

    let (hdr, msg) = simulator::gen_wrapped_battery_status(state);
    conn.send(&hdr, &msg)?;

    Ok(())
}

pub fn send_fast_cadence_sensors(conn: &UorbConnection , state: &mut VehicleState) -> Result<(), Error> {
    let (hdr, msg) = simulator::gen_wrapped_sensor_gyro(state);
    conn.send(&hdr, &msg)?;

    let (hdr, msg) = simulator::gen_wrapped_sensor_accel(state);
    conn.send(&hdr, &msg)?;

    let (hdr, msg) = simulator::gen_wrapped_sensor_mag(state);
    conn.send(&hdr, &msg)?;

    let (hdr, msg) = simulator::gen_wrapped_sensor_baro(state);
    conn.send(&hdr, &msg)?;

    Ok(())
}