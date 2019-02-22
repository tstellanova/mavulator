

use crate::{VehicleState};
use uorb_codec::common::*;

pub const STD_PRESS: f64 = 101325.0;  // static pressure at sea level (Pa)
pub const STD_TEMP: f64 = 288.15;    // standard temperature at sea level (K)
pub const LAPSE_RATE: f64 = -0.0065;   // standard temp altitude lapse rate (K/m)
pub const MOL_MASS : f64 = 0.0289644;  // molar mass of Earth's air (kg/mol)
pub const ACCEL_G : f64 = 9.80665;    // gravity acceleration (m/s^2)
pub const GAS_CONSTANT_R : f64 = 8.31432;    // universal gas constant, R


/// Convert altitude (meters) to standard barometric pressure (Pascals)
/// Note: this formula is likely only useful under 10k feet
pub fn altitude_to_baro_pressure(alt: f32) -> f32 {
    let big_alt: f64 = alt.into();
    let base = STD_TEMP / (STD_TEMP + (LAPSE_RATE * big_alt));
    let exp = (ACCEL_G * MOL_MASS) / (GAS_CONSTANT_R * LAPSE_RATE);
    let val: f64 = STD_PRESS * base.powf(exp);
    (val as f32)
}


pub fn gen_battery_status_data(state: &VehicleState) -> BatteryStatusData {
    let timestamp = state.simulated_usecs;
    BatteryStatusData {
        timestamp: timestamp,
        voltage_v: 16.0,
        voltage_filtered_v: 16.0,
        current_a: 0.0,
        current_filtered_a: 0.0,
        average_current_a: 0.0,
        discharged_mah: 0.0,
        remaining: 10500.0,
        scale: 1.0,
        temperature: 25.0,
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
        warning: 0,
    }
}

pub fn gen_gps_msg_data(state: &VehicleState) -> VehicleGpsPositionData {
    let timestamp = state.simulated_usecs;
    VehicleGpsPositionData {
        timestamp: timestamp,
        lat: 0,
        lon: 0,
        alt: 100,
        alt_ellipsoid: 100,
        s_variance_m_s: 3.0,
        c_variance_rad: 2.0,
        fix_type: 3,
        eph: 1.5,
        epv: 1.5,
        hdop: 1.0,
        vdop: 1.0,
        noise_per_ms: 10,
        jamming_indicator: 0,
        vel_m_s: 0.0,
        vel_n_m_s: 0.0,
        vel_e_m_s: 0.0,
        vel_d_m_s: 0.0,
        cog_rad: 0.1,
        vel_ned_valid: true,
        timestamp_time_relative: 0,
        time_utc_usec: timestamp,
        satellites_used: 11,
        heading: 0.001,
        heading_offset: 0.0,
    }
}

pub fn gen_sensor_gyro_data(state: &VehicleState, device_id: u32) -> SensorGyroData {
    let timestamp = state.simulated_usecs;
    SensorGyroData {
        device_id: device_id,
        timestamp: timestamp,
        error_count: 0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        integral_dt: 0,
        x_integral: 0.0,
        y_integral: 0.0,
        z_integral: 0.0,
        temperature: 25.0,
        scaling: 1.0,
        x_raw: 0,
        y_raw: 0,
        z_raw: 0,
    }
}