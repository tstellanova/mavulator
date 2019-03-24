use uorb_codec::{self, UorbHeader, UorbMessage};
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::{Duration};

use flighty::models::ActuatorControls;
use flighty::simulato::Simulato;

//use uorb_codec::common::ActuatorOutputsData;
use uorb_codec::common::*;

use crate::connection::UorbConnection;



pub fn handle_actuator_outputs(shared_simulato:Arc<RwLock<Simulato>>,
                               header: &UorbHeader,
                               data: &ActuatorOutputsData
) {
    let controls = normalize_actuator_outputs(&data.output, 4);
    let mut state_w = shared_simulato.write().unwrap();
    //println!("msg time: {}", header.timestamp);
    state_w.update(header.timestamp, &controls);
}

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


pub fn feedback_loop(shared_simulato:Arc<RwLock<Simulato>>, vehicle_conn:Arc<Box<UorbConnection+Send+Sync>>) {
    // loop receiving messages from the mav firmware itself
    loop {
        match vehicle_conn.recv() {
            Ok((header, msg)) => {
                match msg {
                    UorbMessage::ActuatorOutputs(m) => {
                        let controls = normalize_actuator_outputs(&m.output, 4);
                        let mut state_w = shared_simulato.write().unwrap();
                        //println!("msg time: {}", header.timestamp);
                        state_w.update(header.timestamp, &controls);
                    },
                    UorbMessage::VehicleStatus(_m) => {
                        //TODO provide to simulato?
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
                        println!("recv error: {:?}", e);
                        break;
                    }
                }
            }
        }
    }
}