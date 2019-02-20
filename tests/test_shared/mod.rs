extern crate uorb_codec;


pub fn get_vehicle_status() -> uorb_codec::common::VehicleStatusData {
    uorb_codec::common::VehicleStatusData {
        timestamp: 83838333,
        nav_state: 14,
        arming_state: 15,
        hil_state: 16,
        failsafe: false,
        system_type: 99,
        system_id: 51,
        component_id: 49,
        is_rotary_wing: true,
        is_vtol: true,
        vtol_fw_permanent_stab: true,
        in_transition_mode: false,
        in_transition_to_fw: false,
        rc_signal_lost: false,
        rc_input_mode: 12,
        data_link_lost: false,
        high_latency_data_link_active: false,
        data_link_lost_counter: 0,
        engine_failure: false,
        mission_failure: false,
        failure_detector_status: 0,
        onboard_control_sensors_present: 12345,
        onboard_control_sensors_enabled: 12345,
        onboard_control_sensors_health: 12345,
    }
}

pub fn get_actuator_controls() -> uorb_codec::common::ActuatorControlsData {
    uorb_codec::common::ActuatorControlsData {
        timestamp: 19,
        timestamp_sample: 21,
        control: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0],
    }
}