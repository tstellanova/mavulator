
use std::f32::NAN;

use uorb_codec::common::*;
use uorb_codec::{self, UorbHeader, UorbMsgMeta};
use std::time::{Duration};

use flighty::simulato::Simulato;
use flighty::physical_types::*;


pub const WHOLE_DEGREE_MULT: LatLonUnits = 1E7;


pub fn micros_from_duration(duration: &Duration) -> u64 {
    (duration.as_secs() * 1000000) + (duration.subsec_micros() as u64)
}

pub fn increment_simulated_time(state: &mut Simulato) {
    //TODO allow simulated time to decouple from real time
    let new_real_time = state.elapsed();
    state.set_simulated_time(micros_from_duration(&new_real_time));
}



pub fn gen_wrapped_battery_status(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_battery_status_data(state);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

pub fn gen_battery_status_data(state: &Simulato) -> BatteryStatusData {
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


pub fn gen_wrapped_gps_position_msg(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_gps_msg_data(state);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

pub fn gen_gps_msg_data(state: &Simulato) -> VehicleGpsPositionData {
    //TODO ensure we use the same altitude that baro has already generated
    let pos = state.sensed.gps.get_val();
    let alt_mm = (pos.alt * 1E3) as i32;

    VehicleGpsPositionData {
        timestamp: state.get_simulated_time(),
        time_utc_usec: 0,

        lat: (pos.lat * WHOLE_DEGREE_MULT) as i32,
        lon: (pos.lon * WHOLE_DEGREE_MULT) as i32,
        alt: alt_mm,
        alt_ellipsoid: 0,

        s_variance_m_s: 0.0,
        c_variance_rad: 0.0,
        fix_type: 3, //3d
        eph: 0.01,
        epv: 0.01,
        hdop: 0.0,
        vdop: 0.0,

        noise_per_ms: 0,
        jamming_indicator: 0,

        vel_m_s: 0.0,
        vel_n_m_s: 0.0,
        vel_e_m_s: 0.0,
        vel_d_m_s: 0.0,

        cog_rad: 0.0,
        vel_ned_valid: false,

        timestamp_time_relative: 0,
        satellites_used: 11,
        heading: NAN,
        heading_offset: NAN,
    }
}


const SIM_GYRO0_DEVICE_ID: u32 = 2293768;
const SIM_GYRO1_DEVICE_ID: u32 = 3141593;

pub fn gen_wrapped_sensor_gyro0(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_gyro_data(state, SIM_GYRO0_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}
pub fn gen_wrapped_sensor_gyro1(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_gyro_data(state, SIM_GYRO1_DEVICE_ID);
    msg_data.gen_ready_pair(1, state.get_simulated_time())
}

const GYRO_REBASE_FACTOR:f32 =  8388463.696;

pub fn gen_sensor_gyro_data(state: &Simulato, device_id: u32) -> SensorGyroData {
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
        temperature: state.sensed.temperature,
        scaling: 1.1920929e-07,
        x_raw: (xgyro * GYRO_REBASE_FACTOR) as i16,
        y_raw: (ygyro * GYRO_REBASE_FACTOR) as i16,
        z_raw: (zgyro * GYRO_REBASE_FACTOR) as i16,
    }
}



const SIM_ACCEL0_DEVICE_ID:u32 = 1376264;
const SIM_ACCEL1_DEVICE_ID:u32 = 1310728;

pub fn gen_wrapped_sensor_accel0(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_accel_data(state, SIM_ACCEL0_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

pub fn gen_wrapped_sensor_accel1(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_accel_data(state, SIM_ACCEL1_DEVICE_ID);
    msg_data.gen_ready_pair(1, state.get_simulated_time())
}

//const ACCEL_REBASE_FACTOR:f32 = (ACCEL_ONE_G / 1E3);

pub fn gen_sensor_accel_data(state: &Simulato, device_id: u32) -> SensorAccelData {
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
        integral_dt: 0, //4000,
        x_integral: 0.0, //3.46E-05,
        y_integral: 0.0, //1.85E-05,
        z_integral: 0.0, //3.92E-02,
        temperature: state.sensed.temperature,
        scaling: 1.19E-07,
        x_raw: 32767, //(xacc / ACCEL_REBASE_FACTOR) as i16,
        y_raw: 32767, //(yacc / ACCEL_REBASE_FACTOR) as i16,
        z_raw: 32767, //(zacc / ACCEL_REBASE_FACTOR) as i16,
    }
}


const SIM_MAG_DEVCE_ID: u32 = 196616;

pub fn gen_wrapped_sensor_mag(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_mag_data(state, SIM_MAG_DEVCE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}


//const MAG_REBASE_FACTOR : f32 = 112657.0/0.0015089384;
const MAG_REBASE_FACTOR : f32 = 8220444.151;

pub fn gen_sensor_mag_data(state: &Simulato, device_id: u32) -> SensorMagData {
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
        temperature: state.sensed.temperature,
        scaling: 0.0,
        x_raw: (xmag * MAG_REBASE_FACTOR) as i16,
        y_raw: (ymag * MAG_REBASE_FACTOR) as i16,
        z_raw: (zmag * MAG_REBASE_FACTOR) as i16,
        is_external: false,
    }
}

const SIM_BARO_DEVICE_ID: u32 = 478459;

pub fn gen_wrapped_sensor_baro(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_baro_data(state, SIM_BARO_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

pub fn gen_sensor_baro_data(state: &Simulato, device_id: u32) -> SensorBaroData {
    SensorBaroData {
        timestamp: state.get_simulated_time(),
        device_id: device_id,
        error_count: 0,
        pressure: 0.0, //TODO
        //altitude_to_baro_pressure(state.alt.measure()),
        temperature: state.sensed.temperature,
    }
}


const SIM_DIFF_PRESS_DEVICE_ID: u32 = 0;

pub fn gen_wrapped_differential_pressure(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_differential_pressure_data(state, SIM_DIFF_PRESS_DEVICE_ID);
    msg_data.gen_ready_pair(0, state.get_simulated_time())
}

pub fn gen_differential_pressure_data(state: &Simulato, device_id: u32) -> DifferentialPressureData {
    let speed_pressure: f32 = state.sensed.airspeed.peek();

    DifferentialPressureData {
        timestamp: state.get_simulated_time(),
        device_id,
        error_count: 0,
        differential_pressure_raw_pa:speed_pressure,
        differential_pressure_filtered_pa: speed_pressure,
        temperature: state.sensed.temperature,
    }
}


/// We use timsync_status to set the initial px4_timestart_monotonic
pub fn gen_wrapped_timesync_status(state: &Simulato) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_timesync_status_data(state);
    msg_data.gen_ready_pair(0, state.abstime_offset)
}

pub fn gen_timesync_status_data(state: &Simulato) -> TimesyncStatusData {
    TimesyncStatusData {
        timestamp: state.abstime_offset,
        remote_timestamp: state.abstime_offset,
        observed_offset: state.abstime_offset as i64,
        estimated_offset: state.abstime_offset as i64,
        round_trip_time: 2, //micros
    }
}

const DURA_UNTIL_ACCEL0_FAILURE:Duration = Duration::from_secs(60);
//const DURA_UNTIL_ACCEL1_FAILURE:Duration = Duration::from_secs(120);

/// Gyro rate should be 400 Hz
/// Accel rate should be 400 Hz
pub fn collect_fast_cadence_sensors(state: &Simulato) -> Vec<(UorbHeader, UorbMessage)> {

    let mut msg_list = vec![];
    if state.elapsed() < DURA_UNTIL_ACCEL0_FAILURE {
        msg_list.push(gen_wrapped_sensor_accel0(state));
        msg_list.push( gen_wrapped_sensor_accel1(state) );
    }
//    if state.elapsed() < DURA_UNTIL_ACCEL1_FAILURE {
//        msg_list.push( gen_wrapped_sensor_accel1(state) );
//    }

    msg_list.push( gen_wrapped_sensor_gyro0(state) );
    msg_list.push( gen_wrapped_sensor_gyro1(state) );
    msg_list
}


/// Mag rate should be 100 Hz
/// Baro rate should be 100 Hz
/// Airspeed should be 100 Hz
pub fn collect_med_cadence_sensors(state: &Simulato) -> Vec<(UorbHeader, UorbMessage)> {
    let mut msg_list = vec![];
    msg_list.push( gen_wrapped_sensor_mag(state) );
    msg_list.push( gen_wrapped_sensor_baro(state) );
    msg_list.push( gen_wrapped_differential_pressure(state) );
    msg_list
}

/// Gps rate should be about 1 Hz
/// Battery status rate should be about 1 Hz
pub fn collect_slow_cadence_sensors(state: &Simulato) -> Vec<(UorbHeader, UorbMessage)> {
    let mut msg_list = vec![];
    msg_list.push(gen_wrapped_gps_position_msg(state));
    msg_list.push(gen_wrapped_battery_status(state));
    msg_list
}



