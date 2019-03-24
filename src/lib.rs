
use std::io::Error;
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration};

use uorb_codec::{UorbHeader, UorbMessage};

pub mod connection;
pub mod sim_reporter;

use connection::UorbConnection;
use flighty::simulato::Simulato;
use flighty::models::ActuatorControls;


///  Default minimum PWM in microseconds
const PWM_DEFAULT_MIN: f32 = 1000.0;
const PWM_DEFAULT_MIN_HALF: f32 = PWM_DEFAULT_MIN / 2.0;

/// Default maximum PWM in microseconds
const PWM_DEFAULT_MAX: f32 = 2000.0;

const PWM_DEFAULT_RANGE: f32 = (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
const PWM_DEFAULT_RANGE_HALF: f32 = PWM_DEFAULT_RANGE / 2.0;

const PWM_DEFAULT_CENTER: f32  = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2.0;


pub fn normalize_actuator_outputs(pwm: &[f32; 16], nrotors: usize) -> ActuatorControls {
    let mut ctrls:ActuatorControls = [0.0; 16];
    //println!("pwm: {:?}",pwm);
    for i in 0..16 {
        let pwm_val = pwm[i];
        if !pwm_val.is_nan() &&  pwm_val > PWM_DEFAULT_MIN_HALF {
            if i < nrotors {
                // scale  to 0..1 for rotors
                //TODO what about reversible rotors??
                ctrls[i] = (pwm_val - PWM_DEFAULT_MIN) / PWM_DEFAULT_RANGE;
                if ctrls[i] < 0.0 {
                    ctrls[i] = 0.0;
                }
            }
            else {
                // scale to -1..1 for non-rotor channels
                ctrls[i] = (pwm_val - PWM_DEFAULT_CENTER) / PWM_DEFAULT_RANGE_HALF;
            }
        }
    }
    //println!("ctrls {:?}", ctrls);
    ctrls
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
        let (hdr, msg) = sim_reporter::gen_wrapped_timesync_status(&state_r);
        let first_msg = vec![(hdr, msg)];
        let res = send_all_messages(&**conn, first_msg);
        if res.is_err() {
            println!("first send failed: {:?}", res);
            return;
        }
    }

    let mut last_slow_cadence_send: u64 = 0; //1Hz sensors
    let mut last_med_cadence_send: u64 = 0; //100Hz sensors
    let mut last_fast_cadence_send: u64 = 0; //400Hz sensors
    loop {
        // TODO modfy this when we decouple from realtime clock
        thread::sleep(Duration::from_micros(100));
        let mut msg_list: Vec<(UorbHeader, UorbMessage)> = vec![];
        {
            {
                //clock is driven by the simulator
                let mut state_w = sim.write().unwrap();
                state_w.increment_simulated_time();
            }

            let state_r = sim.read().unwrap();
            //Fast cadence: 400Hz approx
            if (0 ==  last_fast_cadence_send) ||
                (state_r.elapsed_since(last_fast_cadence_send) > 2500) {
                let msgs = sim_reporter::collect_fast_cadence_sensors(&state_r);
                msg_list.extend(msgs);
                last_fast_cadence_send = state_r.get_simulated_time();
            }

            // Medium cadence: about 100Hz  10000 usec
            if (0 ==  last_med_cadence_send) ||
                (state_r.elapsed_since(last_med_cadence_send) > 10000) {
                let msgs = sim_reporter::collect_med_cadence_sensors(&state_r);
                msg_list.extend(msgs);
                last_med_cadence_send = state_r.get_simulated_time();
            }

            //Slow cadence: 1Hz approx
            if (0 ==  last_slow_cadence_send) ||
                (state_r.elapsed_since(last_slow_cadence_send) > 100000) {
                let msgs = sim_reporter::collect_slow_cadence_sensors(&state_r);
                msg_list.extend(msgs);
                last_slow_cadence_send = state_r.get_simulated_time();
            }
        }

        if msg_list.len() > 0 {
            //send all messages
            let res = send_all_messages(&**conn, msg_list);
            if res.is_err() {
                println!("sending failed: {:?}", res);
                return;
            }
        }

    }
}


fn send_all_messages( conn: &UorbConnection,  msg_list: Vec<(UorbHeader, UorbMessage)>) -> Result<(), Error> {
    for (hdr, msg) in msg_list {
        conn.send(&hdr, &msg)?;
    }
    Ok(())
}
