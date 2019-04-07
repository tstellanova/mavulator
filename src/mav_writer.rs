
use std::f32::NAN;

use std::sync::{Arc, RwLock};
use std::thread;
use std::io::Error;
use std::time::{Duration};

use uorb_codec::common::*;
use uorb_codec::{self, UorbHeader, UorbMessage, UorbMsgMeta};

use crate::connection::UorbConnection;

use flighty::simulato::Simulato;
use flighty::physical_types::*;

/// multiplier for converting LatLonUnits into whole numbers
const WHOLE_DEGREE_MULT: LatLonUnits = 1E7;

/// The minimum GPS ground velocity that can be considered valid
const GPS_HVEL_MINIMUM_VALID: SpeedUnits = 1.0;

/// Timing for fast cadence sensor cadence: 400Hz approx (or greater)
const FAST_CADENCE_MICROSECONDS: TimeBaseUnits = 2500;
/// Timing for medium cadence sensors: 100Hz approx
const MEDIUM_CADENCE_MICROSECONDS: TimeBaseUnits = 10000;
/// Timing for slow cadence sensors: 5Hz approximately
const SLOW_CADENCE_MICROSECONDS: TimeBaseUnits = 200000;


//TODO collect_messages shouldn't really be pub , but is required for benchmarking?
pub fn collect_messages(sim: &Arc<RwLock<Simulato>>,
                    last_slow_cadence_send: &mut TimeBaseUnits,
                    last_med_cadence_send: &mut TimeBaseUnits,
                    last_fast_cadence_send: &mut TimeBaseUnits

) -> Vec<(UorbHeader, UorbMessage)> {

    let mut msg_list: Vec<(UorbHeader, UorbMessage)> = vec![];
    {
        let time_check: TimeBaseUnits;
        {
            //clock is driven by the physical simulator
            let mut state_w = sim.write().unwrap();
            state_w.increment_simulated_time();
            time_check = state_w.get_simulated_time();
        }

        let state_r = sim.read().unwrap();
        //let sensed_z = state_r.sensed.accel.get_val()[2];
        //println!("time {}  sensed accel_z {}", time_check, sensed_z );

        if (0 ==  *last_fast_cadence_send) ||
            (state_r.elapsed_since(*last_fast_cadence_send) > FAST_CADENCE_MICROSECONDS) {
            let msgs = collect_fast_cadence_sensors(&state_r);
            msg_list.extend(msgs);
            *last_fast_cadence_send = time_check;
        }

        // Medium cadence: about 100Hz  10000 usec
        if (0 ==  *last_med_cadence_send) ||
            (state_r.elapsed_since(*last_med_cadence_send) > MEDIUM_CADENCE_MICROSECONDS) {
            let msgs = collect_med_cadence_sensors(&state_r);
            msg_list.extend(msgs);
            *last_med_cadence_send = time_check;
        }

        //Slow cadence: 1Hz approx
        if (0 ==  *last_slow_cadence_send) ||
            (state_r.elapsed_since(*last_slow_cadence_send) > SLOW_CADENCE_MICROSECONDS) {
            let msgs = collect_slow_cadence_sensors(&state_r);
            msg_list.extend(msgs);
            *last_slow_cadence_send = time_check;
        }
    }
    msg_list
}

/// Report sensor data from the vehicle
///
/// - Gyro rate should be 400 Hz
/// - Accel rate should be 400 Hz
/// - Mag rate should be 100 Hz
/// - Baro rate should be 100 Hz
/// - Airspeed should be 100 Hz
///
pub fn reporting_loop(sim:Arc<RwLock<Simulato>>, conn:Arc<Box<UorbConnection+Send+Sync>>) {
    {
        //send a first message to establish a time base (abs time offset)
        let state_r = sim.read().unwrap();
        let (hdr, msg) = gen_wrapped_timesync_status(&state_r);
        let first_msg = vec![(hdr, msg)];
        let res = send_all_messages(&**conn, first_msg);
        if res.is_err() {
            println!("first send failed: {:?}", res);
            return;
        }
    }

    let mut last_slow_cadence_send:TimeBaseUnits = 0; //1Hz sensors
    let mut last_med_cadence_send:TimeBaseUnits = 0; //100Hz sensors
    let mut last_fast_cadence_send:TimeBaseUnits = 0; //400Hz sensors

    loop {
        //thread::sleep(Duration::from_micros(100));
        thread::yield_now();
        let msg_list = collect_messages(&sim,
            &mut last_slow_cadence_send,
            &mut last_med_cadence_send,
            &mut last_fast_cadence_send
        );
        if msg_list.len() > 0 {
            //send all messages
            //let start = SystemTime::now();
            let res = send_all_messages(&**conn, msg_list);
            if res.is_err() {
                println!("sending failed: {:?}", res);
                return;
            }
            //let elapsed = start.elapsed().unwrap();
            //println!("send_all_messages time: {:?}", elapsed);
        }
    }
}


fn send_all_messages( conn: &UorbConnection,  msg_list: Vec<(UorbHeader, UorbMessage)>) -> Result<(), Error> {
    for (hdr, msg) in msg_list {
        conn.send(&hdr, &msg)?;
    }
    Ok(())
}


fn gen_wrapped_vehicle_attitude(state: &Simulato)-> (UorbHeader, UorbMessage) {
    let msg_data = gen_wrapped_vehicle_attitude_data(state);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

fn gen_wrapped_vehicle_attitude_data(state: &Simulato)-> VehicleAttitudeData {
    VehicleAttitudeData {
        timestamp: state.get_simulated_time(),
        rollspeed: state.vehicle_state.kinematic.body_angular_velocity[0],
        pitchspeed: state.vehicle_state.kinematic.body_angular_velocity[1],
        yawspeed: state.vehicle_state.kinematic.body_angular_velocity[2],
        q: [0.0, 0.0, 0.0, 1.0], //TODO upright quaternion always
        delta_q_reset: [0.0, 0.0, 0.0, 0.0],
        quat_reset_counter: 0,
    }
}


fn gen_wrapped_battery_status(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_battery_status_data(state);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

fn gen_battery_status_data(state: &Simulato) -> BatteryStatusData {
    BatteryStatusData {
        timestamp: state.get_simulated_time(),
        voltage_v: 16.0,
        voltage_filtered_v: 16.0,
        current_a: 0.0,
        current_filtered_a: 0.0,
        average_current_a: 0.0,
        discharged_mah: 0.0,
        remaining: 10500.0,
        scale: 1.0,
        temperature: state.vehicle_state.base_temperature,
        cell_count: 4,
        connected: true,
        system_source: true,
        priority: 1,
        capacity: 12000,
        cycle_count: 5,
        run_time_to_empty: 2350,
        average_time_to_empty: 2350,
        serial_number: 12345,
        voltage_cell_v: [3.2, 3.2, 3.2, 3.2],
        max_cell_voltage_delta: 3.2,
        is_powering_off: false,
        warning: BatteryStatusData::BATTERY_WARNING_NONE,
    }
}

fn gen_wrapped_gps_position_msg(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_gps_msg_data(state);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

fn gen_gps_msg_data(state: &Simulato) -> VehicleGpsPositionData {
    //TODO ensure we use the same altitude that baro has already generated
    let pos = state.sensed.gps.get_global_pos();
    let vel = state.sensed.gps.get_velocity();
    let alt_mm = (pos.alt_wgs84 * 1E3) as i32;

    let vel_ned_ground_valid = vel[3] > GPS_HVEL_MINIMUM_VALID;

    VehicleGpsPositionData {
        timestamp: state.get_simulated_time(),
        time_utc_usec: 0,

        lat: (pos.lat * WHOLE_DEGREE_MULT) as i32,
        lon: (pos.lon * WHOLE_DEGREE_MULT) as i32,
        alt: alt_mm, //TODO not strictly accurate-- AMSL not the same as above-ellipsoid
        alt_ellipsoid: alt_mm, //accurate

        s_variance_m_s: 0.0,
        c_variance_rad: 0.0,
        fix_type: 3, //3D
        eph: 0.01,
        epv: 0.01,
        hdop: 0.0,
        vdop: 0.0,

        noise_per_ms: 0,
        jamming_indicator: 0,

        vel_n_m_s: if vel_ned_ground_valid { vel[0] } else {0.0},
        vel_e_m_s: if vel_ned_ground_valid { vel[1] } else {0.0},
        vel_d_m_s: vel[2],
        vel_m_s:   if vel_ned_ground_valid { vel[3] } else {0.0},
        vel_ned_valid: vel_ned_ground_valid,

        //course over ground (cod) = atan2(y,x)
        cog_rad: if vel_ned_ground_valid { vel[1].atan2(vel[0]) } else { 0.0  },

        timestamp_time_relative: 0,
        satellites_used: 11,
        heading: NAN,
        heading_offset: NAN,
    }
}


const SIM_GYRO0_DEVICE_ID: u32 = 2293768;
//const SIM_GYRO1_DEVICE_ID: u32 = 3141593;

fn gen_wrapped_sensor_gyro0(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_gyro_data(state, SIM_GYRO0_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}
//fn gen_wrapped_sensor_gyro1(state: &Simulato) -> (UorbHeader, UorbMessage) {
//    let msg_data = gen_sensor_gyro_data(state, SIM_GYRO1_DEVICE_ID);
//    msg_data.gen_ready_pair(1, state.get_simulated_time())
//}

const GYRO_REBASE_FACTOR:f32 =  8388463.696;

fn gen_sensor_gyro_data(state: &Simulato, device_id: u32) -> SensorGyroData {
    let gyro_bucket = state.sensed.gyro.get_val();
    let xgyro = gyro_bucket[0];
    let ygyro = gyro_bucket[1];
    let zgyro = gyro_bucket[2];

    SensorGyroData {
        device_id,
        timestamp: state.get_simulated_time(),
        error_count: 0,
        x: xgyro,
        y: ygyro,
        z: zgyro,
        integral_dt: 0 , //4000,
        x_integral: 0.0, //6.511943e-06,
        y_integral: 0.0, //9.221726e-06,
        z_integral: 0.0, //2.8863593e-05,
        temperature: state.vehicle_state.base_temperature,
        scaling: 1.1920929e-07,
        x_raw: (xgyro * GYRO_REBASE_FACTOR) as i16,
        y_raw: (ygyro * GYRO_REBASE_FACTOR) as i16,
        z_raw: (zgyro * GYRO_REBASE_FACTOR) as i16,
    }
}



const SIM_ACCEL0_DEVICE_ID:u32 = 1376264;
//const SIM_ACCEL1_DEVICE_ID:u32 = 1310728;

fn gen_wrapped_sensor_accel0(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_accel_data(state, SIM_ACCEL0_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

//fn gen_wrapped_sensor_accel1(state: &Simulato) -> (UorbHeader, UorbMessage) {
//    let msg_data = gen_sensor_accel_data(state, SIM_ACCEL1_DEVICE_ID);
//    msg_data.gen_ready_pair(1, state.get_simulated_time())
//}

//const ACCEL_REBASE_FACTOR:f32 = (ACCEL_ONE_G / 1E3);

fn gen_sensor_accel_data(state: &Simulato, device_id: u32) -> SensorAccelData {
    let accel_bucket = state.sensed.accel.get_val();
    let xacc = accel_bucket[0];
    let yacc = accel_bucket[1];
    let zacc = accel_bucket[2];

    SensorAccelData {
        timestamp: state.get_simulated_time(),
        device_id: device_id,
        error_count: 0,
        x: xacc,
        y: yacc,
        z: zacc,
        integral_dt: 0, //4000, //TODO maybe FAST_CADENCE_MICROSECONDS
        x_integral: 0.0, //3.46E-05,
        y_integral: 0.0, //1.85E-05,
        z_integral: 0.0, //3.92E-02,
        temperature: state.vehicle_state.base_temperature,
        scaling: 1.19E-07,
        x_raw: 32767, //(xacc / ACCEL_REBASE_FACTOR) as i16,
        y_raw: 32767, //(yacc / ACCEL_REBASE_FACTOR) as i16,
        z_raw: 32767, //(zacc / ACCEL_REBASE_FACTOR) as i16,
    }
}


const SIM_MAG_DEVCE_ID: u32 = 196616;

fn gen_wrapped_sensor_mag(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_mag_data(state, SIM_MAG_DEVCE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}


const MAG_REBASE_FACTOR : f32 = 8220444.151;

fn gen_sensor_mag_data(state: &Simulato, device_id: u32) -> SensorMagData {
    let mag_bucket = state.sensed.mag.get_val();
    let xmag = mag_bucket[0];
    let ymag = mag_bucket[1];
    let zmag = mag_bucket[2];

    SensorMagData {
        timestamp: state.get_simulated_time(),
        device_id: device_id,
        error_count: 0,
        x: xmag,
        y: ymag,
        z: zmag,
        temperature: state.vehicle_state.base_temperature,
        scaling: 0.0,
        x_raw: (xmag * MAG_REBASE_FACTOR) as i16,
        y_raw: (ymag * MAG_REBASE_FACTOR) as i16,
        z_raw: (zmag * MAG_REBASE_FACTOR) as i16,
        is_external: false,
    }
}

const SIM_BARO_DEVICE_ID: u32 = 478459;

fn gen_wrapped_sensor_baro(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_baro_data(state, SIM_BARO_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

fn gen_sensor_baro_data(state: &Simulato, device_id: u32) -> SensorBaroData {
    SensorBaroData {
        timestamp: state.get_simulated_time(),
        device_id,
        error_count: 0,
        pressure: state.sensed.baro.get_val() ,
        temperature: state.vehicle_state.base_temperature,
    }
}


const SIM_DIFF_PRESS_DEVICE_ID: u32 = 0;

fn gen_wrapped_differential_pressure(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_differential_pressure_data(state, SIM_DIFF_PRESS_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

fn gen_differential_pressure_data(state: &Simulato, device_id: u32) -> DifferentialPressureData {
    let speed_pressure: f32 = state.sensed.airspeed.get_val();

    DifferentialPressureData {
        timestamp: state.get_simulated_time(),
        device_id,
        error_count: 0,
        differential_pressure_raw_pa:speed_pressure,
        differential_pressure_filtered_pa: speed_pressure,
        temperature: state.vehicle_state.base_temperature,
    }
}


/// We use timsync_status to set the initial px4_timestart_monotonic
fn gen_wrapped_timesync_status(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_timesync_status_data(state);
    msg_data.gen_ready_pair(0, state.abstime_offset)
}

fn gen_timesync_status_data(state: &Simulato) -> TimesyncStatusData {
    TimesyncStatusData {
        timestamp: state.abstime_offset,
        remote_timestamp: state.abstime_offset,
        observed_offset: state.abstime_offset as i64,
        estimated_offset: state.abstime_offset as i64,
        round_trip_time: 2, //micros
    }
}

//const DURA_UNTIL_ACCEL0_FAILURE:Duration = Duration::from_secs(300);
//const DURA_UNTIL_ACCEL1_FAILURE:Duration = Duration::from_secs(120);

/// Gyro rate should be 400 Hz
/// Accel rate should be 400 Hz
fn collect_fast_cadence_sensors(state: &Simulato) -> Vec<(UorbHeader, UorbMessage)> {

    let mut msg_list = vec![];
    //if state.elapsed() < DURA_UNTIL_ACCEL0_FAILURE {
        msg_list.push(gen_wrapped_sensor_accel0(state));
        //msg_list.push( gen_wrapped_sensor_accel1(state) );
    //}

   msg_list.push( gen_wrapped_sensor_gyro0(state) );
   // msg_list.push( gen_wrapped_sensor_gyro1(state) );

//    let gyrobes = state.sensed.gyro.get_val();
//    let accelbes = state.sensed.accel.get_val();
//
//    println!("{} {} {}  {} {} {}",
//        gyrobes[0],
//        gyrobes[1],
//        gyrobes[2],
//        accelbes[0],
//        accelbes[1],
//        accelbes[2]);


    msg_list
}


/// Mag rate should be 100 Hz
/// Baro rate should be 100 Hz
/// Airspeed should be 100 Hz
fn collect_med_cadence_sensors(state: &Simulato) -> Vec<(UorbHeader, UorbMessage)> {
    let mut msg_list = vec![];
    msg_list.push( gen_wrapped_sensor_mag(state) );
    msg_list.push( gen_wrapped_sensor_baro(state) );
    msg_list.push( gen_wrapped_differential_pressure(state) );
    //TODO for now we force the attitude to upright
    msg_list.push( gen_wrapped_vehicle_attitude(state) );
    msg_list
}

/// Gps rate should be about 1 Hz
/// Battery status rate should be about 1 Hz
fn collect_slow_cadence_sensors(state: &Simulato) -> Vec<(UorbHeader, UorbMessage)> {
    let mut msg_list = vec![];
    msg_list.push(gen_wrapped_gps_position_msg(state));
    msg_list.push(gen_wrapped_battery_status(state));
    msg_list
}



