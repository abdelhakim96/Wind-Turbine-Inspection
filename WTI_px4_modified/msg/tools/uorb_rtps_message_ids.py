
# Message identification constants


msg_id_map = {
    'actuator_armed': 1,
    'actuator_controls': 2,
    'actuator_direct': 3,
    'actuator_outputs': 4,
    'adc_report': 5,
    'airspeed': 6,
    'att_pos_mocap': 7,
    'battery_status': 8,
    'camera_capture': 9,
    'camera_trigger': 10,
    'collision_report': 11,
    'commander_state': 12,

    'cpuload': 14,
    'debug_key_value': 15,
    'differential_pressure': 16,
    'distance_sensor': 17,
    'ekf2_innovations': 18,

    'ekf2_timestamps': 20,
    'esc_report': 21,
    'esc_status': 22,
    'estimator_status': 23,
    'fence': 24,
    'fence_vertex': 25,

    'follow_target': 27,
    'fw_pos_ctrl_status': 28,
    'geofence_result': 29,
    'gps_dump': 30,
    'gps_inject_data': 31,

    'home_position': 33,
    'input_rc': 34,
    'led_control': 35,
    'log_message': 36,
    'manual_control_setpoint': 37,
    'mavlink_log': 38,
    'mc_att_ctrl_status': 39,
    'mission': 40,
    'mission_result': 41,
    'mount_orientation': 42,
    'multirotor_motor_limits': 43,
    'offboard_control_mode': 44,
    'optical_flow': 45,

    'parameter_update': 47,
    'position_setpoint': 48,
    'position_setpoint_triplet': 49,
    'pwm_input': 50,
    'qshell_req': 51,
    'rc_channels': 52,
    'rc_parameter_map': 53,
    'safety': 54,
    'satellite_info': 55,
    'sensor_accel': 56,
    'sensor_baro': 57,
    'sensor_combined': 58,
    'sensor_correction': 59,
    'sensor_gyro': 60,
    'sensor_mag': 61,
    'sensor_preflight': 62,
    'sensor_selection': 63,
    'servorail_status': 64,
    'subsystem_info': 65,
    'system_power': 66,
    'task_stack_info': 67,
    'tecs_status': 68,
    'telemetry_status': 69,
    'test_motor': 70,
    'time_offset': 71,
    'transponder_report': 72,
    'uavcan_parameter_request': 73,
    'uavcan_parameter_value': 74,
    'ulog_stream_ack': 75,
    'ulog_stream': 76,
    'vehicle_attitude': 77,
    'vehicle_attitude_setpoint': 78,
    'vehicle_command_ack': 79,
    'vehicle_command': 80,
    'vehicle_control_mode': 81,

    'vehicle_global_position': 83,

    'vehicle_gps_position': 85,
    'vehicle_land_detected': 86,
    'vehicle_local_position': 87,
    'vehicle_local_position_setpoint': 88,
    'vehicle_rates_setpoint': 89,
    'vehicle_roi': 90,
    'vehicle_status_flags': 91,
    'vehicle_status': 92,
    'vtol_vehicle_status': 93,
    'wind_estimate': 94,
}

def message_id(message):
    """
    Get id of a message
    """
    if message in msg_id_map:
        return msg_id_map[message]
    return 0
