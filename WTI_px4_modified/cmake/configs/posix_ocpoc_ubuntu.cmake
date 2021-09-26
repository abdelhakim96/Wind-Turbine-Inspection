#set toolchain
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-gcc-arm-linux-gnueabihf.cmake)

set(CMAKE_PROGRAM_PATH
	${CMAKE_PROGRAM_PATH}
)


add_definitions(
  -D__PX4_POSIX_OCPOC
  -D__DF_LINUX # For DriverFramework
  -D__DF_OCPOC # For DriverFramework
  -D__PX4_POSIX
)

set(config_module_list
	#
	# Board support modules
	#
	drivers/distance_sensor
	modules/sensors
	platforms/posix/drivers/df_mpu9250_wrapper
	platforms/posix/drivers/df_ms5611_wrapper
	platforms/posix/drivers/df_hmc5883_wrapper

	#
	# System commands
	#
	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/perf

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/local_position_estimator
	modules/landing_target_estimator
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control
	modules/fw_att_control
	modules/fw_pos_control_l1

	#
	# Library modules
	#
	modules/sdlog2
	modules/logger
	modules/commander
	modules/dataman
	modules/land_detector
	modules/navigator
	modules/mavlink

	#
	# PX4 drivers
	#
	drivers/gps
	drivers/ocpoc_adc
	drivers/linux_sbus
	drivers/linux_pwm_out
	drivers/rgbled
	
	examples/px4_simple_app
)

#
# DriverFramework driver
#
set(config_df_driver_list
	mpu9250
	ms5611
	hmc5883
)
