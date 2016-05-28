if( APPLE )
  link_directories(/opt/local/lib)
endif( APPLE )

#########################################################
# Base headers
#
set(argos2_common_control_interface_HEADERS_base
  ci_actuator.h
  ci_camera_sensor.h
  ci_clock_sensor.h
  ci_controller.h
  ci_robot.h
  ci_sensor.h
  ci_wifi_actuator.h
  ci_wifi_sensor.h
)


#########################################################
# Swarmanoid robot headers
#
set(argos2_common_control_interface_HEADERS_swarmanoid
  swarmanoid/ci_battery_sensor.h
  swarmanoid/ci_range_and_bearing_actuator.h
  swarmanoid/ci_range_and_bearing_sensor.h
)

set(argos2_common_control_interface_HEADERS_swarmanoid_footbot
  swarmanoid/footbot/ci_footbot_base_ground_sensor.h
  swarmanoid/footbot/ci_footbot_beacon_actuator.h
  swarmanoid/footbot/ci_footbot_omnidirectional_camera_sensor.h
  swarmanoid/footbot/ci_footbot_ceiling_camera_sensor.h
  swarmanoid/footbot/ci_footbot_distance_scanner_actuator.h
  swarmanoid/footbot/ci_footbot_distance_scanner_sensor.h
  swarmanoid/footbot/ci_footbot_encoder_sensor.h
  swarmanoid/footbot/ci_footbot_gripper_actuator.h
  swarmanoid/footbot/ci_footbot_gripper_sensor.h
  swarmanoid/footbot/ci_footbot_leds_actuator.h
  swarmanoid/footbot/ci_footbot_base_leds_actuator.h
  swarmanoid/footbot/ci_footbot_light_sensor.h
  swarmanoid/footbot/ci_footbot_motor_ground_sensor.h
  swarmanoid/footbot/ci_footbot_proximity_sensor.h
  swarmanoid/footbot/ci_footbot_turret_torque_sensor.h
  swarmanoid/footbot/ci_footbot_turret_encoder_sensor.h
  swarmanoid/footbot/ci_footbot_turret_actuator.h
  swarmanoid/footbot/ci_footbot_wheel_speed_sensor.h
  swarmanoid/footbot/ci_footbot_gyroscopic_sensor.h
  swarmanoid/footbot/ci_footbot_wheels_actuator.h
  swarmanoid/footbot/ci_footbot_accelerometer_sensor.h
)

set(argos2_common_control_interface_HEADERS_swarmanoid_handbot
  swarmanoid/handbot/ci_handbot_arm_encoders_sensor.h
  swarmanoid/handbot/ci_handbot_arms_actuator.h
  swarmanoid/handbot/ci_handbot_beacon_leds_actuator.h
  swarmanoid/handbot/ci_handbot_gripper_cameras.h
  swarmanoid/handbot/ci_handbot_leds_actuator.h
  swarmanoid/handbot/ci_handbot_retrieve_book_actuator.h
  swarmanoid/handbot/ci_handbot_proximity_sensor.h
  swarmanoid/handbot/ci_handbot_head_camera_sensor.h
)

set(argos2_common_control_interface_HEADERS_swarmanoid_eyebot
   swarmanoid/eyebot/ci_eyebot_altitude_sensor.h
   swarmanoid/eyebot/ci_eyebot_distance_scanner_sensor.h
   swarmanoid/eyebot/ci_eyebot_leds_actuator.h
   swarmanoid/eyebot/ci_eyebot_pan_and_tilt_camera_actuator.h
   swarmanoid/eyebot/ci_eyebot_pan_and_tilt_camera_sensor.h
   swarmanoid/eyebot/ci_eyebot_propellers_actuator.h
   swarmanoid/eyebot/ci_eyebot_speech_actuator.h
)

#########################################################
# Various robots headers
#
set(argos2_common_control_interface_HEADERS_epuck
  e-puck/ci_epuck_leds_actuator.h
  e-puck/ci_epuck_light_sensor.h
  e-puck/ci_epuck_proximity_sensor.h
  e-puck/ci_epuck_accelerometer_sensor.h
  e-puck/ci_epuck_micro_sensor.h
  e-puck/ci_epuck_ground_sensor.h
  e-puck/ci_epuck_wheels_actuator.h
  e-puck/ci_epuck_range_and_bearing_sensor.h
  e-puck/ci_epuck_range_and_bearing_actuator.h
  e-puck/ci_epuck_camera_sensor.h
  e-puck/ci_epuck_encoder_sensor.h
  e-puck/ci_epuck_omnidirectional_camera_sensor.h
)

set(argos2_common_control_interface_HEADERS_bluebot
  bluebot/ci_bluebot_proximity_sensor.h
  bluebot/ci_bluebot_wheels_actuator.h
)

set(argos2_common_control_interface_HEADERS_booth
  booth/ci_booth_led_actuator.h
  booth/ci_booth_presence_sensor.h
)

set(argos2_common_control_interface_HEADERS_neural_networks
  neural_networks/ci_ctrnn_multilayer_controller.h
  neural_networks/ci_neural_network_controller.h
  neural_networks/ci_perceptron_controller.h
  neural_networks/ci_rnctn_controller.h
)

#########################################################
# Behavioral toolkit headers
#
set(argos2_common_control_interface_HEADERS_bt
  behavioral_toolkit/ci_behavior_controller.h
  behavioral_toolkit/ci_behavior.h
  behavioral_toolkit/ci_robot_state.h
  behavioral_toolkit/ci_robot_data.h
)

set(argos2_common_control_interface_HEADERS_bt_swarmanoid
  behavioral_toolkit/swarmanoid/ci_swarmanoid_robot_state.h
)

set(argos2_common_control_interface_HEADERS_bt_swarmanoid_footbot
  behavioral_toolkit/swarmanoid/footbot/ci_footbot_state.h
)

set(argos2_common_control_interface_HEADERS_bt_swarmanoid_handbot
  behavioral_toolkit/swarmanoid/handbot/ci_handbot_state.h
)

set(argos2_common_control_interface_HEADERS_bt_swarmanoid_eyebot
  behavioral_toolkit/swarmanoid/eyebot/ci_eyebot_state.h
)

set(argos2_common_control_interface_HEADERS_bt_utility
  behavioral_toolkit/utility_classes/helper.h
)

set(argos2_common_control_interface_HEADERS_bt_utility_fsm
  behavioral_toolkit/utility_classes/fsm/fsm.h
  behavioral_toolkit/utility_classes/fsm/transition.h
  behavioral_toolkit/utility_classes/fsm/state.h
  behavioral_toolkit/utility_classes/fsm/action.h
)

#########################################################
# All implementations
#
set(argos2_common_control_interface_IMPLEMENTATIONS
  # base
  ci_clock_sensor.cpp
  ci_controller.cpp
  ci_robot.cpp
  )

if( ARGOS_COMPILE_REAL_SWARMANOID OR (NOT ARGOS_COMPILE_REAL_EPUCK) )
  set(argos2_common_control_interface_IMPLEMENTATIONS
    ${argos2_common_control_interface_IMPLEMENTATIONS}
    # swarmanoid robots
    swarmanoid/footbot/ci_footbot_base_ground_sensor.cpp
    swarmanoid/footbot/ci_footbot_gripper_actuator.cpp
    swarmanoid/footbot/ci_footbot_leds_actuator.cpp
    swarmanoid/footbot/ci_footbot_base_leds_actuator.cpp
    swarmanoid/footbot/ci_footbot_light_sensor.cpp
    swarmanoid/footbot/ci_footbot_motor_ground_sensor.cpp
    swarmanoid/footbot/ci_footbot_proximity_sensor.cpp
    swarmanoid/footbot/ci_footbot_turret_actuator.cpp
    swarmanoid/footbot/ci_footbot_turret_torque_sensor.cpp
    swarmanoid/footbot/ci_footbot_turret_encoder_sensor.cpp
    swarmanoid/footbot/ci_footbot_wheels_actuator.cpp
    swarmanoid/handbot/ci_handbot_proximity_sensor.cpp
    swarmanoid/handbot/ci_handbot_arms_actuator.cpp
    swarmanoid/handbot/ci_handbot_beacon_leds_actuator.cpp
    swarmanoid/handbot/ci_handbot_gripper_cameras.cpp
    swarmanoid/handbot/ci_handbot_leds_actuator.cpp
    swarmanoid/handbot/ci_handbot_retrieve_book_actuator.cpp
    swarmanoid/eyebot/ci_eyebot_leds_actuator.cpp
    swarmanoid/eyebot/ci_eyebot_speech_actuator.cpp
    swarmanoid/eyebot/ci_eyebot_distance_scanner_sensor.cpp
    # behavioral toolkit
    behavioral_toolkit/ci_robot_state.cpp
    behavioral_toolkit/swarmanoid/ci_swarmanoid_robot_state.cpp
    behavioral_toolkit/swarmanoid/footbot/ci_footbot_state.cpp
    behavioral_toolkit/swarmanoid/handbot/ci_handbot_state.cpp
    behavioral_toolkit/swarmanoid/eyebot/ci_eyebot_state.cpp
    )
endif( ARGOS_COMPILE_REAL_SWARMANOID OR (NOT ARGOS_COMPILE_REAL_EPUCK) )

if( ARGOS_COMPILE_REAL_EPUCK OR (NOT ARGOS_COMPILE_REAL_SWARMANOID) )
  set(argos2_common_control_interface_IMPLEMENTATIONS
    ${argos2_common_control_interface_IMPLEMENTATIONS}
    # epuck robots
    e-puck/ci_epuck_leds_actuator.cpp
    e-puck/ci_epuck_light_sensor.cpp
    e-puck/ci_epuck_proximity_sensor.cpp
    e-puck/ci_epuck_accelerometer_sensor.cpp
    e-puck/ci_epuck_micro_sensor.cpp
    # bluebot
    bluebot/ci_bluebot_proximity_sensor.cpp
    )
endif( ARGOS_COMPILE_REAL_EPUCK OR (NOT ARGOS_COMPILE_REAL_SWARMANOID) )

add_library(argos2_common_control_interface SHARED ${argos2_common_control_interface_IMPLEMENTATIONS})
if( NOT ARGOS_COMPILE_REAL_SWARMANOID AND NOT ARGOS_COMPILE_REAL_EPUCK )
  link_googleperftools_to(argos2_common_control_interface)
endif( NOT ARGOS_COMPILE_REAL_SWARMANOID AND NOT ARGOS_COMPILE_REAL_EPUCK )

install(FILES ${argos2_common_control_interface_HEADERS_base} DESTINATION include/argos2/common/control_interface)
if( ARGOS_COMPILE_REAL_SWARMANOID OR (NOT ARGOS_COMPILE_REAL_EPUCK) )
  install(FILES ${argos2_common_control_interface_HEADERS_swarmanoid} DESTINATION include/argos2/common/control_interface/swarmanoid)
  install(FILES ${argos2_common_control_interface_HEADERS_swarmanoid_footbot} DESTINATION include/argos2/common/control_interface/swarmanoid/footbot)
  install(FILES ${argos2_common_control_interface_HEADERS_swarmanoid_handbot} DESTINATION include/argos2/common/control_interface/swarmanoid/handbot)
  install(FILES ${argos2_common_control_interface_HEADERS_swarmanoid_eyebot} DESTINATION include/argos2/common/control_interface/swarmanoid/eyebot)
  install(FILES ${argos2_common_control_interface_HEADERS_bt} DESTINATION include/argos2/common/control_interface/behavioral_toolkit)
  install(FILES ${argos2_common_control_interface_HEADERS_bt_utility} DESTINATION include/argos2/common/control_interface/behavioral_toolkit/utility_classes)
  install(FILES ${argos2_common_control_interface_HEADERS_bt_utility_fsm} DESTINATION include/argos2/common/control_interface/behavioral_toolkit/utility_classes/fsm)
  install(FILES ${argos2_common_control_interface_HEADERS_bt_swarmanoid} DESTINATION include/argos2/common/control_interface/behavioral_toolkit/swarmanoid)
  install(FILES ${argos2_common_control_interface_HEADERS_bt_swarmanoid_footbot} DESTINATION include/argos2/common/control_interface/behavioral_toolkit/swarmanoid/footbot)
  install(FILES ${argos2_common_control_interface_HEADERS_bt_swarmanoid_eyebot} DESTINATION include/argos2/common/control_interface/behavioral_toolkit/swarmanoid/eyebot)
  install(FILES ${argos2_common_control_interface_HEADERS_bt_swarmanoid_handbot} DESTINATION include/argos2/common/control_interface/behavioral_toolkit/swarmanoid/handbot)
endif( ARGOS_COMPILE_REAL_SWARMANOID OR (NOT ARGOS_COMPILE_REAL_EPUCK) )

if( ARGOS_COMPILE_REAL_EPUCK OR (NOT ARGOS_COMPILE_REAL_SWARMANOID) )
  install(FILES ${argos2_common_control_interface_HEADERS_epuck} DESTINATION include/argos2/common/control_interface/e-puck)
  install(FILES ${argos2_common_control_interface_HEADERS_bluebot} DESTINATION include/argos2/common/control_interface/bluebot)
endif( ARGOS_COMPILE_REAL_EPUCK OR (NOT ARGOS_COMPILE_REAL_SWARMANOID) )

install(TARGETS argos2_common_control_interface
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos2
  ARCHIVE DESTINATION lib/argos2)
