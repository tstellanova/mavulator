

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
//type MetersPerSecond = f32;
type MetersPerSecondPerSecond = f32;
//type RadiansPerSecond = f32;
//type RadiansPerSecondPerSecond = f32;
//
pub type WGS84Degrees = f32;

//pub type WholeMillimeters = i32;


/// Some guesses as to accuracy of a fake accelerometer
const ACCEL_ABS_ERR : MetersPerSecondPerSecond = 5e-2;
const ACCEL_REL_ERR : MetersPerSecondPerSecond = 1e-4;

const GYRO_ABS_ERR : f32 = 1e-2;
const GYRO_REL_ERR : f32 = 1e-4;

const MAG_ABS_ERR : f32 = 5e-3;
const MAG_REL_ERR : f32 = 1e-4;

const GPS_DEGREES_ABS_ERR: WGS84Degrees = 1e-6;
const GPS_DEGREES_REL_ERR: WGS84Degrees = 1e-8;

const ALT_ABS_ERR: Meters = 1e-1;
const ALT_REL_ERR: Meters = 1e-4;

const ACCEL_ONE_G: MetersPerSecondPerSecond = 9.80665;

/// Fake home coordinates
const HOME_LAT: WGS84Degrees = 37.8;
const HOME_LON: WGS84Degrees = -122.2;
const HOME_ALT: Meters = 500.0;
const HOME_MAG:[f32; 3] = [ 22535E-5, 5384E-5, 42217E-5 ];
const LANDED_ACCEL_VAL: MetersPerSecondPerSecond = 1E-3;
const LANDED_GYRO_VAL: f32 = 1E-4;

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

        xgyro: Sensulator::new(LANDED_GYRO_VAL, GYRO_ABS_ERR, GYRO_REL_ERR),
        ygyro: Sensulator::new(LANDED_GYRO_VAL, GYRO_ABS_ERR, GYRO_REL_ERR),
        zgyro: Sensulator::new(LANDED_GYRO_VAL, GYRO_ABS_ERR, GYRO_REL_ERR),

        xacc:  Sensulator::new(LANDED_ACCEL_VAL, ACCEL_ABS_ERR, ACCEL_REL_ERR),
        yacc:  Sensulator::new(LANDED_ACCEL_VAL, ACCEL_ABS_ERR, ACCEL_REL_ERR),
        zacc:  Sensulator::new(ACCEL_ONE_G, ACCEL_ABS_ERR, ACCEL_REL_ERR),

        xmag: Sensulator::new(HOME_MAG[0], MAG_ABS_ERR, MAG_REL_ERR),
        ymag: Sensulator::new(HOME_MAG[1], MAG_ABS_ERR, MAG_REL_ERR),
        zmag: Sensulator::new(HOME_MAG[2], MAG_ABS_ERR, MAG_REL_ERR),
    }
}

/// Convert altitude (meters) to standard barometric pressure (Pascals)
/// Note: this formula is likely only useful under 10k feet
pub fn altitude_to_baro_pressure(alt: f32) -> f32 {
    let big_alt: f64 = alt.into();
    let base = STD_TEMP / (STD_TEMP + (LAPSE_RATE * big_alt));
    let exp = (ACCEL_G * MOL_MASS) / (GAS_CONSTANT_R * LAPSE_RATE);
    let val: f64 = STD_PRESS * base.powf(exp);
    (val as f32)
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
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: BatteryStatusData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: BatteryStatusData::ENCODED_LEN
    };
    let msg =  UorbMessage::BatteryStatus(msg_data);
    (hdr, msg)
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


//TODO move this wrapping into uorb-codec itself
pub fn gen_wrapped_gps_position_msg(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_gps_msg_data(state);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: VehicleGpsPositionData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: VehicleGpsPositionData::ENCODED_LEN
    };
    let msg =  UorbMessage::VehicleGpsPosition(msg_data);
    (hdr, msg)
}

pub fn gen_gps_msg_data(state: &mut VehicleState) -> VehicleGpsPositionData {
    let alt = state.alt.read();
    VehicleGpsPositionData {
        timestamp: state.simulated_usecs,
        lat: (state.lat.read() * WHOLE_DEGREE_MULT) as i32,
        lon: (state.lon.read() * WHOLE_DEGREE_MULT) as i32,
        alt: (alt * 1E3) as i32,
        alt_ellipsoid: (alt * 1E3) as i32,
        s_variance_m_s: 0.0,
        c_variance_rad: 0.0,
        fix_type: 3,
        eph: 1.0,
        epv: 1.0,
        hdop: 0.0,
        vdop: 0.0,
        noise_per_ms: 0,
        jamming_indicator: 0,
        vel_m_s: 0.0,
        vel_n_m_s: 0.0,
        vel_e_m_s: 0.0,
        vel_d_m_s: 0.0,
        cog_rad: 0.1,
        vel_ned_valid: true,
        timestamp_time_relative: 0,
        time_utc_usec: state.simulated_usecs,
        satellites_used: 11,
        heading: 0.001,
        heading_offset: 0.0,
    }
}


const SIM_GYRO_DEVICE_ID: u32 = 2293768;

pub fn gen_wrapped_sensor_gyro(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_gyro_data(state, SIM_GYRO_DEVICE_ID);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: SensorGyroData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: SensorGyroData::ENCODED_LEN
    };
    let msg =  UorbMessage::SensorGyro(msg_data);
    (hdr, msg)
}

const GYRO_REBASE_FACTOR:f32 =  1E3;

pub fn gen_sensor_gyro_data(state: &mut VehicleState, device_id: u32) -> SensorGyroData {
//    println!("gyro time: {:x} {:x}",
//             ((state.simulated_usecs >> 32) & 0xFFFFFFFF) as u32,
//             (state.simulated_usecs & 0xFFFFFFFF) as u32
//    );

    let xgyro = state.xgyro.read();
    let ygyro = state.ygyro.read();
    let zgyro = state.zgyro.read();

    SensorGyroData {
        device_id: device_id,
        timestamp: state.simulated_usecs,
        error_count: 0,
        x: xgyro,
        y: ygyro,
        z: zgyro,
        integral_dt: 5 * 1000000,
        x_integral: 1.0,
        y_integral: 1.0,
        z_integral: 1.0,
        temperature: state.temperature,
        scaling: 0.0,
        x_raw: (xgyro * GYRO_REBASE_FACTOR) as i16,
        y_raw: (ygyro * GYRO_REBASE_FACTOR) as i16,
        z_raw: (zgyro * GYRO_REBASE_FACTOR) as i16,
    }
}


const SIM_ACCEL_DEVICE_ID:u32 = 1376264;

pub fn gen_wrapped_sensor_accel(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_accel_data(state, SIM_ACCEL_DEVICE_ID);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: SensorAccelData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: SensorAccelData::ENCODED_LEN
    };
    let msg =  UorbMessage::SensorAccel(msg_data);
    (hdr, msg)
}

const ACCEL_REBASE_FACTOR:f32 = (ACCEL_ONE_G / 1E3);

pub fn gen_sensor_accel_data(state: &mut VehicleState, device_id: u32) -> SensorAccelData {
    let xacc = state.xacc.read();
    let yacc = state.yacc.read();
    let zacc = state.zacc.read();

    SensorAccelData {
        timestamp: state.simulated_usecs,
        device_id: device_id,
        error_count: 0,
        x: xacc,
        y: yacc,
        z: zacc,
        integral_dt: 0,
        x_integral: 0.0,
        y_integral: 0.0,
        z_integral: 0.0,
        temperature: state.temperature,
        scaling: 0.0,
        x_raw: (xacc / ACCEL_REBASE_FACTOR) as i16,
        y_raw: (yacc / ACCEL_REBASE_FACTOR) as i16,
        z_raw: (zacc / ACCEL_REBASE_FACTOR) as i16,
    }
}


const SIM_MAG_DEVCE_ID: u32 = 196616;

pub fn gen_wrapped_sensor_mag(state: &mut VehicleState) -> (UorbHeader, UorbMessage) {
    let msg_data = gen_sensor_mag_data(state, SIM_MAG_DEVCE_ID);
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: SensorMagData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: SensorMagData::ENCODED_LEN
    };
    let msg =  UorbMessage::SensorMag(msg_data);
    (hdr, msg)
}

const MAG_REBASE_FACTOR : f32 = 1E3;

pub fn gen_sensor_mag_data(state: &mut VehicleState, device_id: u32) -> SensorMagData {
    let xmag = state.xmag.read();
    let ymag = state.ymag.read();
    let zmag = state.zmag.read();

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
    let hdr:UorbHeader = UorbHeader {
        version: uorb_codec::UORB_MAGIC_V1,
        hash: SensorBaroData::MSG_HASH_CODE,
        instance_id: 0,
        payload_len: SensorBaroData::ENCODED_LEN
    };
    let msg =  UorbMessage::SensorBaro(msg_data);
    (hdr, msg)
}

pub fn gen_sensor_baro_data(state: &mut VehicleState, device_id: u32) -> SensorBaroData {
    SensorBaroData {
        timestamp: state.simulated_usecs,
        device_id: device_id,
        error_count: 0,
        pressure: altitude_to_baro_pressure(state.alt.read()),
        temperature: state.temperature,
    }
}