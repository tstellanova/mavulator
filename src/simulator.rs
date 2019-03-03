
use std::f32::NAN;

use uorb_codec::common::*;
use uorb_codec::{self, UorbHeader, UorbMsgMeta};
use std::time::{Duration, SystemTime};
use sensulator::Sensulator;

pub const WHOLE_DEGREE_MULT: f32 = 1E7;

pub const STD_PRESS: f64 = 101325.0;  // static pressure at sea level (Pa)
pub const STD_TEMP: f64 = 288.15;    // standard temperature at sea level (K)
pub const LAPSE_RATE: f64 = -0.0065;   // standard temp altitude lapse rate (K/m)
pub const MOL_MASS : f64 = 0.0289644;  // molar mass of Earth's air (kg/mol)
pub const ACCEL_G : f64 = 9.80665;    // gravity acceleration (m/s^2)
pub const GAS_CONSTANT_R : f64 = 8.31432;    // universal gas constant, R


//physical type definitions

type Meters = f32;
type Pascals = f32;
//type MetersPerSecond = f32;
type MetersPerSecondPerSecond = f32;
//type RadiansPerSecond = f32;
//type RadiansPerSecondPerSecond = f32;
//
pub type WGS84Degrees = f32;

//pub type WholeMillimeters = i32;


const MIN_SENSOR_REL_ERR:f32  = 1E-7;

const GYRO_ABS_ERR : f32 = 0.0000266;
const GYRO_REL_ERR : f32 = MIN_SENSOR_REL_ERR;

const ACCEL_ABS_ERR : f32 = 0.0000267;
const ACCEL_REL_ERR : f32 = MIN_SENSOR_REL_ERR;

const MAG_ABS_ERR : f32 = 0.0000265;
const MAG_REL_ERR : f32 = MIN_SENSOR_REL_ERR;

const GPS_DEGREES_ABS_ERR: WGS84Degrees = 1E-6;
const GPS_DEGREES_REL_ERR: WGS84Degrees = MIN_SENSOR_REL_ERR;

// this range appears to allow EKF fusion to begin
const ALT_ABS_ERR: Meters = 1.5;
const ALT_REL_ERR: Meters = MIN_SENSOR_REL_ERR;

//const ACCEL_ONE_G: MetersPerSecondPerSecond = 9.80665;

/// Fake home coordinates
const HOME_LAT: WGS84Degrees =  37.8001024;
const HOME_LON: WGS84Degrees =  -122.1997184;
const HOME_ALT: Meters = 500.0;

//const HOME_MAG:[f32; 3] = [ 22535E-5, 5384E-5, 42217E-5 ];
//const HOME_MAG:[f32; 3] = [ 0.00144767145, 0.004156071878, 0.003152540706 ];
const HOME_MAG:[f32; 3] = [ 0.001613762305, 0.003411047039, 0.002402817653 ];

const LANDED_ACCEL_VALS:[MetersPerSecondPerSecond; 3] = [0.008651008647, 0.004627180345, 9.801405599];

const LANDED_GYRO_VALS:[f32; 3] = [0.0020351426,0.0028725443,0.009010567];

pub struct VehicleState {
    boot_time: SystemTime,
    simulated_usecs: u64,
    temperature: f32,

    ///--- Data arriving directly from sensors:
    /// GPS
    lat: Sensulator,
    lon: Sensulator,
    alt: Sensulator,

    /// Gyro
    xgyro: Sensulator,
    ygyro: Sensulator,
    zgyro: Sensulator,

    /// Accelerometer
    xacc: Sensulator,
    yacc: Sensulator,
    zacc: Sensulator,

    /// Magnetometer
    xmag: Sensulator,
    ymag: Sensulator,
    zmag: Sensulator,

    /// Airspeed (differential pressure)
    diff_press: Sensulator,

}

impl VehicleState {
    pub fn elapsed_since(&self, old: u64) -> u64 {
        let mut diff: i64 = (self.simulated_usecs - old) as i64;
        if diff < 0 {
            diff = 0;
        }
        diff as u64
    }

    pub fn get_simulated_usecs(&self) -> u64 {
        self.simulated_usecs
    }
}


pub fn initial_vehicle_state() ->  VehicleState {
    VehicleState {
        boot_time: SystemTime::now(),
        simulated_usecs: 0,
        temperature: 25.0,

        lat: Sensulator::new(HOME_LAT , GPS_DEGREES_ABS_ERR, GPS_DEGREES_REL_ERR),
        lon: Sensulator::new(HOME_LON , GPS_DEGREES_ABS_ERR, GPS_DEGREES_REL_ERR),
        // this range appears to allow EKF fusion to begin
        alt: Sensulator::new(HOME_ALT , ALT_ABS_ERR, ALT_REL_ERR),

        xgyro: Sensulator::new(LANDED_GYRO_VALS[0], GYRO_ABS_ERR, GYRO_REL_ERR),
        ygyro: Sensulator::new(LANDED_GYRO_VALS[1], GYRO_ABS_ERR, GYRO_REL_ERR),
        zgyro: Sensulator::new(LANDED_GYRO_VALS[2], GYRO_ABS_ERR, GYRO_REL_ERR),

        xacc:  Sensulator::new(LANDED_ACCEL_VALS[0], ACCEL_ABS_ERR, ACCEL_REL_ERR),
        yacc:  Sensulator::new(LANDED_ACCEL_VALS[1], ACCEL_ABS_ERR, ACCEL_REL_ERR),
        zacc:  Sensulator::new(LANDED_ACCEL_VALS[2], ACCEL_ABS_ERR, ACCEL_REL_ERR),

        xmag: Sensulator::new(HOME_MAG[0], MAG_ABS_ERR, MAG_REL_ERR),
        ymag: Sensulator::new(HOME_MAG[1], MAG_ABS_ERR, MAG_REL_ERR),
        zmag: Sensulator::new(HOME_MAG[2], MAG_ABS_ERR, MAG_REL_ERR),

        //diff press requires some error in order to pass preflight checks
        diff_press: Sensulator::new(0.0, 1E-3, MIN_SENSOR_REL_ERR),
    }
}

/// Convert altitude (meters) to standard barometric pressure (Pascals)
/// Note: this formula is likely only useful under 10k feet
pub fn altitude_to_baro_pressure(alt: Meters) -> Pascals {
    let big_alt: f64 = alt.into();
    let base = STD_TEMP / (STD_TEMP + (LAPSE_RATE * big_alt));
    let exp = (ACCEL_G * MOL_MASS) / (GAS_CONSTANT_R * LAPSE_RATE);
    let val: f64 = STD_PRESS * base.powf(exp);
    (val as Pascals)
}


pub fn micros_from_duration(duration: &Duration) -> u64 {
    (duration.as_secs() * 1000000) + (duration.subsec_micros() as u64)
}

pub fn increment_simulated_time(state: &mut VehicleState) {
    //TODO allow simulated time to decouple from real time
    let new_real_time = state.boot_time.elapsed().unwrap();
    state.simulated_usecs = micros_from_duration(&new_real_time);
}

pub fn gen_wrapped_battery_status(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_battery_status_data(state);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}

pub fn gen_battery_status_data(state: &VehicleState) -> BatteryStatusData {
    BatteryStatusData {
        timestamp: state.simulated_usecs,
        voltage_v: 16.0,
        voltage_filtered_v: 16.0,
        current_a: 0.0,
        current_filtered_a: 0.0,
        average_current_a: 0.0,
        discharged_mah: 0.0,
        remaining: 10500.0,
        scale: 1.0,
        temperature: state.temperature,
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


pub fn gen_wrapped_gps_position_msg(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_gps_msg_data(state);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}

pub fn gen_gps_msg_data(state: &mut VehicleState) -> VehicleGpsPositionData {
    let alt = state.alt.peek();//use the same altitude that baro has already generated
    let alt_mm = (alt * 1E3) as i32;

    VehicleGpsPositionData {
        timestamp: state.simulated_usecs,
        time_utc_usec: 0,

        lat: (state.lat.measure() * WHOLE_DEGREE_MULT) as i32,
        lon: (state.lon.measure() * WHOLE_DEGREE_MULT) as i32,
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


const SIM_GYRO_DEVICE_ID: u32 = 2293768;

pub fn gen_wrapped_sensor_gyro(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_gyro_data(state, SIM_GYRO_DEVICE_ID);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}


const GYRO_REBASE_FACTOR:f32 =  8388463.696; //17072.0/0.0020351426;

pub fn gen_sensor_gyro_data(state: &mut VehicleState, device_id: u32) -> SensorGyroData {

    let xgyro = state.xgyro.measure();
    let ygyro = state.ygyro.measure();
    let zgyro = state.zgyro.measure();

    SensorGyroData {
        device_id: device_id,
        timestamp: state.simulated_usecs,
        error_count: 0,
        x: xgyro,
        y: ygyro,
        z: zgyro,
        integral_dt:4000,
        x_integral: 6.511943e-06,
        y_integral: 9.221726e-06,
        z_integral: 2.8863593e-05,
        temperature: state.temperature,
        scaling: 1.1920929e-07,
        x_raw: (xgyro * GYRO_REBASE_FACTOR) as i16,
        y_raw: (ygyro * GYRO_REBASE_FACTOR) as i16,
        z_raw: (zgyro * GYRO_REBASE_FACTOR) as i16,
    }
}


const SIM_ACCEL_DEVICE_ID:u32 = 1376264;

pub fn gen_wrapped_sensor_accel(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_accel_data(state, SIM_ACCEL_DEVICE_ID);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}

//const ACCEL_REBASE_FACTOR:f32 = (ACCEL_ONE_G / 1E3);

pub fn gen_sensor_accel_data(state: &mut VehicleState, device_id: u32) -> SensorAccelData {
    let xacc = state.xacc.measure();
    let yacc = state.yacc.measure();
    let zacc = state.zacc.measure();

    SensorAccelData {
        timestamp: state.simulated_usecs,
        device_id: device_id,
        error_count: 0,
        x: xacc,
        y: yacc,
        z: zacc,
        integral_dt: 4000,
        x_integral: 3.46E-05,
        y_integral: 1.85E-05,
        z_integral: 3.92E-02,
        temperature: state.temperature,
        scaling: 1.19E-07,
        x_raw: 32767, //(xacc / ACCEL_REBASE_FACTOR) as i16,
        y_raw: 32767, //(yacc / ACCEL_REBASE_FACTOR) as i16,
        z_raw: 32767, //(zacc / ACCEL_REBASE_FACTOR) as i16,
    }
}


const SIM_MAG_DEVCE_ID: u32 = 196616;

pub fn gen_wrapped_sensor_mag(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_mag_data(state, SIM_MAG_DEVCE_ID);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}


//const MAG_REBASE_FACTOR : f32 = 112657.0/0.0015089384;
const MAG_REBASE_FACTOR : f32 = 8220444.151;

pub fn gen_sensor_mag_data(state: &mut VehicleState, device_id: u32) -> SensorMagData {
    let xmag = state.xmag.measure();
    let ymag = state.ymag.measure();
    let zmag = state.zmag.measure();

    SensorMagData {
        timestamp: state.simulated_usecs,
        device_id: device_id,
        error_count: 0,
        x: xmag,
        y: ymag,
        z: zmag,
        temperature: state.temperature,
        scaling: 0.0,
        x_raw: (xmag * MAG_REBASE_FACTOR) as i16,
        y_raw: (ymag * MAG_REBASE_FACTOR) as i16,
        z_raw: (zmag * MAG_REBASE_FACTOR) as i16,
        is_external: false,
    }
}

const SIM_BARO_DEVICE_ID: u32 = 478459;

pub fn gen_wrapped_sensor_baro(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_baro_data(state, SIM_BARO_DEVICE_ID);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}

pub fn gen_sensor_baro_data(state: &mut VehicleState, device_id: u32) -> SensorBaroData {
    SensorBaroData {
        timestamp: state.simulated_usecs,
        device_id: device_id,
        error_count: 0,
        pressure: altitude_to_baro_pressure(state.alt.measure()),
        temperature: state.temperature,
    }
}


const SIM_DIFF_PRESS_DEVICE_ID: u32 = 0;

pub fn gen_wrapped_differential_pressure(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_differential_pressure_data(state, SIM_DIFF_PRESS_DEVICE_ID);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}

pub fn gen_differential_pressure_data(state: &mut VehicleState, device_id: u32) -> DifferentialPressureData {
    let speed_pressure: f32 = state.diff_press.measure();

    DifferentialPressureData {
        timestamp: state.simulated_usecs,
        device_id: device_id,
        error_count: 0,
        differential_pressure_raw_pa:speed_pressure,
        differential_pressure_filtered_pa: speed_pressure,
        temperature: state.temperature,
    }
}



pub fn gen_wrapped_timesync_status(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_timesync_status_data(state);
    msg_data.gen_ready_pair(0,state.simulated_usecs)
}

pub fn gen_timesync_status_data(state: &mut VehicleState) -> TimesyncStatusData {
    TimesyncStatusData {
        timestamp: state.simulated_usecs,
        remote_timestamp: state.simulated_usecs,
        observed_offset: 0,
        estimated_offset: 0,
        round_trip_time: 2, //micros
    }
}


/// Gyro rate should be 400 Hz
/// Accel rate should be 400 Hz
pub fn gen_fast_cadence_sensors(state: &mut VehicleState) -> Vec<(UorbHeader, UorbMessage)> {
    let mut msg_list = vec![];
    msg_list.push( gen_wrapped_timesync_status(state) );
    msg_list.push( gen_wrapped_sensor_accel(state) );
    msg_list.push( gen_wrapped_sensor_gyro(state) );
    msg_list
}


/// Mag rate should be 100 Hz
/// Baro rate should be 100 Hz
/// Airspeed should be 100 Hz
pub fn gen_med_cadence_sensors(state: &mut VehicleState) -> Vec<(UorbHeader, UorbMessage)> {
    let mut msg_list = vec![];
    msg_list.push( gen_wrapped_sensor_mag(state) );
    msg_list.push( gen_wrapped_sensor_baro(state) );
    msg_list.push( gen_wrapped_differential_pressure(state) );
    msg_list
}

/// Gps rate should be about 1 Hz
/// Battery status rate should be about 1 Hz
pub fn gen_slow_cadence_sensors(state: &mut VehicleState) -> Vec<(UorbHeader, UorbMessage)> {
    let mut msg_list = vec![];
    msg_list.push(gen_wrapped_gps_position_msg(state));
    msg_list.push(gen_wrapped_battery_status(state));
    msg_list
}



