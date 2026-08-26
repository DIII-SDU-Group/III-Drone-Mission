cmake_minimum_required(VERSION 3.8)
project(iii_drone_mission)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

include(CTest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(OpenSSL REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  ament_index_cpp

  builtin_interfaces
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs

  px4_msgs
  px4_ros2_cpp

  tf2
  tf2_ros
  tf2_geometry_msgs
  
  iii_drone_core
  iii_drone_interfaces
  iii_drone_configuration

  behaviortree_cpp
  behaviortree_ros2

  dynmsg
)

# Loop through dependencies and issue find_package:
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

# yaml-cpp: require system package to avoid vendored deprecation warnings.
find_package(yaml-cpp REQUIRED)

# Include dir
set(III_DRONE_MISSION_INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/include"
)

# Library files:
set(III_DRONE_MISSION_HDRS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/px4/modes/maneuver_mode.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/px4/modes/mode_provider.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/px4/mode_executors/generic_mode_executor.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/port_types.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/behavior_node_registry.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/powerline_geometry.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/hover_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/hover_on_cable_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/fly_to_position_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/follow_waypoint_path_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/fly_to_object_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/cable_landing_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/cable_takeoff_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/pl_mapper_command_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/gripper_command_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/update_powerline_overview_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/get_powerline_overview_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/get_pylon_overview_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/powerline_waypoint_provider_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/phase_waypoint_provider_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/mode_executor_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/log_message_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/rosbag_recording_action_nodes.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/runtime_intent_action_nodes.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/inspection_waypoint_progress_nodes.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/cable_charging_monitor_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/split_point_queue_action_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/verify_gripper_closed_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/verify_powerline_detected_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/select_target_line_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/target_provider_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/store_current_state_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/publish_powerline_waypoints_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/verify_disarmed_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/maneuver_action_status_nodes.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/battery_recharge_condition_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/trees/tree_executor.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/trees/tree_provider.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/mission_specification.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/mission_catalog.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/mission_executor.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/overview_gnss_persistence.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/rosbag_recorder_node/rosbag_recorder_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/pylon_overview_provider_node/pylon_overview_provider_node.hpp"

)

set(III_DRONE_MISSION_SRCS
  "${PROJECT_SOURCE_DIR}/src/px4/modes/maneuver_mode.cpp"
  "${PROJECT_SOURCE_DIR}/src/px4/modes/mode_provider.cpp"

  "${PROJECT_SOURCE_DIR}/src/px4/mode_executors/generic_mode_executor.cpp"

  "${PROJECT_SOURCE_DIR}/src/px4/setpoints/trajectory_setpoint.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/port_types.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/behavior_node_registry.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/powerline_geometry.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/maneuver_action_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_by_object_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_on_cable_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/fly_to_position_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/follow_waypoint_path_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/fly_to_object_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/cable_landing_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/cable_takeoff_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/pl_mapper_command_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/gripper_command_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/update_powerline_overview_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/get_powerline_overview_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/get_pylon_overview_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/powerline_waypoint_provider_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/phase_waypoint_provider_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/mode_executor_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/log_message_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/rosbag_recording_action_nodes.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/runtime_intent_action_nodes.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/inspection_waypoint_progress_nodes.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/cable_charging_monitor_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/split_point_queue_action_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/verify_gripper_closed_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/verify_powerline_detected_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/select_target_line_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/target_provider_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/store_current_state_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/publish_powerline_waypoints_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/verify_disarmed_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/maneuver_action_status_nodes.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/battery_recharge_condition_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/trees/tree_executor.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/trees/tree_provider.cpp"

  "${PROJECT_SOURCE_DIR}/src/mission/mission_specification.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/mission_catalog.cpp"

  "${PROJECT_SOURCE_DIR}/src/mission/mission_executor.cpp"
  "${PROJECT_SOURCE_DIR}/src/mission/overview_gnss_persistence.cpp"

)

include_directories(
  ${III_DRONE_MISSION_INCLUDE_DIRS}
)

# III-Drone-Mission-Lib:
add_library(iii_drone_mission_lib 
  SHARED
  ${III_DRONE_MISSION_HDRS}
  ${III_DRONE_MISSION_SRCS}
)

target_include_directories(iii_drone_mission_lib
  PUBLIC
  ${III_DRONE_MISSION_INCLUDE_DIRS}
)

target_link_libraries(iii_drone_mission_lib
  yaml-cpp
  OpenSSL::Crypto
)

ament_target_dependencies(iii_drone_mission_lib
  ${dependencies}
)

ament_export_libraries(iii_drone_mission_lib)
ament_export_include_directories(include)

install(TARGETS iii_drone_mission_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(iii_behavior_node_contract_exporter
  "${PROJECT_SOURCE_DIR}/src/tools/behavior_node_contract_exporter.cpp"
)
target_link_libraries(iii_behavior_node_contract_exporter
  iii_drone_mission_lib
  OpenSSL::Crypto
)
ament_target_dependencies(iii_behavior_node_contract_exporter ${dependencies})

include("${PROJECT_SOURCE_DIR}/cmake/IIIRegisterMission.cmake")

iii_register_mission(
  ID inspection-production
  SPECIFICATION mission_specification/mission_specification.yaml
  CLASSIFICATION production
  PROFILES real opti_track sim hil
  DEFAULT_FOR real opti_track sim
)
iii_register_mission(
  ID reach-charge-leave-experimental
  SPECIFICATION mission_specification/mission_specification_reach_charge_leave.yaml
  CLASSIFICATION experimental
  PROFILES real opti_track sim hil
)
iii_register_mission(
  ID ftp-blend-loop-test
  SPECIFICATION mission_specification/mission_specification_ftp_blend_loop.yaml
  CLASSIFICATION test
  PROFILES sim
)
iii_register_mission(
  ID ftp-legacy
  SPECIFICATION mission_specification/mission_specification.yaml.ftp_test
  CLASSIFICATION legacy
  STATUS deprecated
  PROFILES sim
)
iii_register_mission(
  ID opti-track-up-down-test
  SPECIFICATION mission_specification/mission_specification.yaml.opti_track
  CLASSIFICATION test
  PROFILES opti_track sim
)
iii_register_mission(
  ID reach-cable-simple-test
  SPECIFICATION mission_specification/mission_specification.yaml.simple_test
  CLASSIFICATION test
  PROFILES sim
)
iii_register_mission(
  ID up-down-test
  SPECIFICATION mission_specification/mission_specification.yaml.up_down
  CLASSIFICATION test
  PROFILES sim
)

set(III_MISSION_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/mission_catalog")
set(III_MISSION_REGISTRATIONS "${III_MISSION_GENERATED_DIR}/registrations.tsv")
set(III_BEHAVIOR_NODE_CONTRACT "${III_MISSION_GENERATED_DIR}/behavior-node-contract.json")
set(III_BEHAVIOR_NODE_MODELS "${III_MISSION_GENERATED_DIR}/models.xml")
set(III_MISSION_LOCAL_CATALOG "${III_MISSION_GENERATED_DIR}/local/catalog.json")
set(III_MISSION_QUALIFIED_CATALOG "${III_MISSION_GENERATED_DIR}/qualified/catalog.json")
set(III_MISSION_FIELD_CANDIDATES_CATALOG "${III_MISSION_GENERATED_DIR}/field-candidates/catalog.json")

file(MAKE_DIRECTORY "${III_MISSION_GENERATED_DIR}")
iii_write_mission_registrations("${III_MISSION_REGISTRATIONS}")
file(GLOB_RECURSE III_MISSION_ASSET_SOURCES CONFIGURE_DEPENDS
  "${PROJECT_SOURCE_DIR}/mission_specification/*"
  "${PROJECT_SOURCE_DIR}/behavior_trees/*"
)
get_property(III_MISSION_REGISTRATION_SOURCES GLOBAL PROPERTY III_MISSION_REGISTRATION_SOURCES)
get_filename_component(III_INTERFACES_SHARE_DIR "${iii_drone_interfaces_DIR}/.." ABSOLUTE)
set(III_MISSION_INTERFACE_CONTRACTS
  "${III_INTERFACES_SHARE_DIR}/msg/MissionModeStatus.msg"
  "${III_INTERFACES_SHARE_DIR}/srv/GetMissionCatalog.srv"
  "${III_INTERFACES_SHARE_DIR}/srv/SelectMissionCatalogEntry.srv"
)
foreach(contract IN LISTS III_MISSION_INTERFACE_CONTRACTS)
  if(NOT EXISTS "${contract}")
    message(FATAL_ERROR "installed mission interface contract is missing: ${contract}")
  endif()
endforeach()

add_custom_command(
  OUTPUT "${III_BEHAVIOR_NODE_CONTRACT}" "${III_BEHAVIOR_NODE_MODELS}"
  COMMAND "$<TARGET_FILE:iii_behavior_node_contract_exporter>"
          "${III_BEHAVIOR_NODE_CONTRACT}"
          "${III_BEHAVIOR_NODE_MODELS}"
  DEPENDS iii_behavior_node_contract_exporter
  COMMENT "Exporting authoritative behavior-node contract"
  VERBATIM
)

set(III_MISSION_CATALOG_COMMAND
  "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/iii_drone_mission/mission_catalog.py" generate
  --source-root "${PROJECT_SOURCE_DIR}"
  --registrations "${III_MISSION_REGISTRATIONS}"
  --node-contract "${III_BEHAVIOR_NODE_CONTRACT}"
  --runtime-contract "${PROJECT_SOURCE_DIR}/config/mission_runtime_contract.json"
  --models-xml "${III_BEHAVIOR_NODE_MODELS}"
  --local-output "${III_MISSION_GENERATED_DIR}/local"
  --qualified-output "${III_MISSION_GENERATED_DIR}/qualified"
  --field-candidates-output "${III_MISSION_GENERATED_DIR}/field-candidates"
)
set(III_MISSION_BUILD_CONTRACTS
  "${PROJECT_SOURCE_DIR}/CMakeLists.txt"
  "${PROJECT_SOURCE_DIR}/cmake/IIIRegisterMission.cmake"
  "${PROJECT_SOURCE_DIR}/iii_drone_mission/mission_catalog.py"
  "${PROJECT_SOURCE_DIR}/src/tools/behavior_node_contract_exporter.cpp"
  ${III_DRONE_MISSION_HDRS}
  ${III_DRONE_MISSION_SRCS}
)
foreach(contract IN LISTS III_MISSION_BUILD_CONTRACTS)
  list(APPEND III_MISSION_CATALOG_COMMAND --build-contract "${contract}")
endforeach()
foreach(contract IN LISTS III_MISSION_INTERFACE_CONTRACTS)
  list(APPEND III_MISSION_CATALOG_COMMAND --interface-contract "${contract}")
endforeach()

add_custom_command(
  OUTPUT
    "${III_MISSION_LOCAL_CATALOG}"
    "${III_MISSION_QUALIFIED_CATALOG}"
    "${III_MISSION_FIELD_CANDIDATES_CATALOG}"
  COMMAND ${III_MISSION_CATALOG_COMMAND}
  DEPENDS
    "${PROJECT_SOURCE_DIR}/iii_drone_mission/mission_catalog.py"
    "${PROJECT_SOURCE_DIR}/config/mission_runtime_contract.json"
    "${III_MISSION_REGISTRATIONS}"
    "${III_BEHAVIOR_NODE_CONTRACT}"
    "${III_BEHAVIOR_NODE_MODELS}"
    ${III_MISSION_REGISTRATION_SOURCES}
    ${III_MISSION_ASSET_SOURCES}
    ${III_MISSION_INTERFACE_CONTRACTS}
    ${III_MISSION_BUILD_CONTRACTS}
  COMMENT "Generating and validating immutable mission catalogs"
  VERBATIM
)

add_custom_target(iii_drone_mission_catalog ALL
  DEPENDS
    "${III_MISSION_LOCAL_CATALOG}"
    "${III_MISSION_QUALIFIED_CATALOG}"
    "${III_MISSION_FIELD_CANDIDATES_CATALOG}"
)

install(DIRECTORY "${III_MISSION_GENERATED_DIR}/local/"
  DESTINATION "share/${PROJECT_NAME}/mission_catalog"
)
install(DIRECTORY "${III_MISSION_GENERATED_DIR}/qualified/"
  DESTINATION "share/${PROJECT_NAME}/mission_catalog_variants/qualified"
)
install(DIRECTORY "${III_MISSION_GENERATED_DIR}/field-candidates/"
  DESTINATION "share/${PROJECT_NAME}/mission_catalog_variants/field-candidates"
)
install(FILES
  "${PROJECT_SOURCE_DIR}/config/mission_runtime_contract.json"
  "${PROJECT_SOURCE_DIR}/cmake/IIIRegisterMission.cmake"
  DESTINATION "share/${PROJECT_NAME}"
)

# Executables:
function(custom_cpp_target target_name target_sources)
  # Loop over optional arguments and add to target sources:
  foreach(arg ${ARGN})
    list(APPEND target_sources "${arg}")
  endforeach()

  # Print target sources:
  message(STATUS "Target sources for ${target_name}:")
  foreach(source ${target_sources})
    message(STATUS "  ${source}")
  endforeach()

  add_executable(${target_name}
    "${target_sources}"
  )

  add_dependencies(${target_name}
    iii_drone_mission_lib
  )

  target_include_directories(${target_name}
    PUBLIC
    "${III_DRONE_MISSION_INCLUDE_DIRS}"
  )

  target_link_libraries(${target_name}
    iii_drone_mission_lib
  )

  ament_target_dependencies(${target_name}
    ${dependencies}
  )

  target_link_libraries(${target_name} "${cpp_typesupport_target}")

  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )

endfunction()

# custom_cpp_target(px4_mode_test
#   "${PROJECT_SOURCE_DIR}/src/px4_mode_test.cpp"
# )

custom_cpp_target(mission_executor
  "${PROJECT_SOURCE_DIR}/src/mission/mission_executor_node/mission_executor_node.cpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/mission_executor_node/mission_executor_node.hpp"
)

custom_cpp_target(powerline_overview_provider
  "${PROJECT_SOURCE_DIR}/src/mission/powerline_overview_provider_node/powerline_overview_provider_node.cpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/powerline_overview_provider_node/powerline_overview_provider_node.hpp"
)

custom_cpp_target(pylon_overview_provider
  "${PROJECT_SOURCE_DIR}/src/mission/pylon_overview_provider_node/pylon_overview_provider_node.cpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/pylon_overview_provider_node/pylon_overview_provider_node.hpp"
)

custom_cpp_target(rosbag_recorder
  "${PROJECT_SOURCE_DIR}/src/mission/rosbag_recorder_node/rosbag_recorder_node.cpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/rosbag_recorder_node/rosbag_recorder_node.hpp"
)

custom_cpp_target(custom_operation
  "${PROJECT_SOURCE_DIR}/src/operations/custom_operation_node.cpp"
)

############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Export information to downstream packages
ament_export_dependencies(${dependencies})

# Install header files
install(DIRECTORY 
  "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(iii_drone_mission_catalog_test
    test/test_mission_catalog.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
  )

  ament_add_gtest(iii_drone_mission_specification_test
    test/mission_specification_test.cpp
  )

  ament_add_gtest(iii_drone_behavior_node_registry_test
    test/behavior_node_registry_test.cpp
  )

  ament_add_gtest(iii_drone_corridor_inspection_route_test
    test/corridor_inspection_route_test.cpp
  )

  ament_add_gtest(iii_drone_cable_axis_yaw_settler_test
    test/cable_axis_yaw_settler_test.cpp
  )
  ament_add_gtest(iii_drone_overview_gnss_persistence_test
    test/overview_gnss_persistence_test.cpp
  )

  if(TARGET iii_drone_mission_specification_test)
    target_include_directories(iii_drone_mission_specification_test PRIVATE ${III_DRONE_MISSION_INCLUDE_DIRS})
    target_link_libraries(iii_drone_mission_specification_test iii_drone_mission_lib)
    ament_target_dependencies(iii_drone_mission_specification_test ${dependencies})
  endif()
  if(TARGET iii_drone_behavior_node_registry_test)
    target_include_directories(iii_drone_behavior_node_registry_test PRIVATE ${III_DRONE_MISSION_INCLUDE_DIRS})
    target_link_libraries(iii_drone_behavior_node_registry_test iii_drone_mission_lib)
    ament_target_dependencies(iii_drone_behavior_node_registry_test ${dependencies})
  endif()

  if(TARGET iii_drone_corridor_inspection_route_test)
    target_include_directories(iii_drone_corridor_inspection_route_test PRIVATE ${III_DRONE_MISSION_INCLUDE_DIRS})
    target_link_libraries(iii_drone_corridor_inspection_route_test iii_drone_mission_lib)
    ament_target_dependencies(iii_drone_corridor_inspection_route_test ${dependencies})
  endif()
  if(TARGET iii_drone_overview_gnss_persistence_test)
    target_include_directories(iii_drone_overview_gnss_persistence_test PRIVATE ${III_DRONE_MISSION_INCLUDE_DIRS})
    target_link_libraries(iii_drone_overview_gnss_persistence_test iii_drone_mission_lib)
    ament_target_dependencies(iii_drone_overview_gnss_persistence_test ${dependencies})
  endif()

  if(TARGET iii_drone_cable_axis_yaw_settler_test)
    target_include_directories(iii_drone_cable_axis_yaw_settler_test PRIVATE ${III_DRONE_MISSION_INCLUDE_DIRS})
    target_link_libraries(iii_drone_cable_axis_yaw_settler_test iii_drone_mission_lib)
    ament_target_dependencies(iii_drone_cable_axis_yaw_settler_test ${dependencies})
  endif()
endif()

ament_package()
