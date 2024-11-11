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

# find dependencies
find_package(ament_cmake REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action

  builtin_interfaces
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs

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

# Yaml cpp
include(FetchContent)

FetchContent_Declare(
  yaml_cpp
  GIT_REPOSITORY https://github.com/DIII-SDU-Group/yaml-cpp.git
  GIT_TAG        0.8.0
)

FetchContent_GetProperties(yaml_cpp)

if(NOT yaml_cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml_cpp)
  if(NOT TARGET yaml-cpp)
    add_subdirectory(${yaml_cpp_SOURCE_DIR} ${yaml_cpp_BINARY_DIR})
  endif()
endif()

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

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/hover_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/hover_on_cable_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/fly_to_position_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/fly_to_object_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/cable_landing_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/cable_takeoff_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/pl_mapper_command_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/gripper_command_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/update_powerline_overview_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/get_powerline_overview_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/powerline_waypoint_provider_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/mode_executor_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/action_nodes/log_message_action_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/verify_gripper_closed_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/verify_powerline_detected_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/select_target_line_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/target_provider_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/store_current_state_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/publish_powerline_waypoints_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/trees/tree_executor.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/behavior/trees/tree_provider.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/mission_specification.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_mission/mission/mission_executor.hpp"

)

set(III_DRONE_MISSION_SRCS
  "${PROJECT_SOURCE_DIR}/src/px4/modes/maneuver_mode.cpp"
  "${PROJECT_SOURCE_DIR}/src/px4/modes/mode_provider.cpp"

  "${PROJECT_SOURCE_DIR}/src/px4/mode_executors/generic_mode_executor.cpp"

  "${PROJECT_SOURCE_DIR}/src/px4/setpoints/trajectory_setpoint.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/port_types.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/maneuver_action_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_by_object_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_on_cable_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/fly_to_position_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/fly_to_object_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/cable_landing_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/cable_takeoff_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/pl_mapper_command_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/gripper_command_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/update_powerline_overview_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/get_powerline_overview_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/powerline_waypoint_provider_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/mode_executor_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/log_message_action_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/verify_gripper_closed_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/verify_powerline_detected_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/select_target_line_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/target_provider_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/store_current_state_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/publish_powerline_waypoints_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/trees/tree_executor.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/trees/tree_provider.cpp"

  "${PROJECT_SOURCE_DIR}/src/mission/mission_specification.cpp"

  "${PROJECT_SOURCE_DIR}/src/mission/mission_executor.cpp"

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

target_sources(iii_drone_mission_lib
  PUBLIC
  ${III_DRONE_MISSION_HDRS}
  ${III_DRONE_MISSION_SRCS}
)

target_include_directories(iii_drone_mission_lib
  PUBLIC
  ${III_DRONE_MISSION_INCLUDE_DIRS}
)

target_link_libraries(iii_drone_mission_lib
  yaml-cpp
)

ament_target_dependencies(iii_drone_mission_lib
  ${dependencies}
)

ament_export_libraries(iii_drone_mission_lib)
ament_export_include_directories(${III_DRONE_MISSION_INCLUDE_DIRS})

install(TARGETS iii_drone_mission_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

############
# Install ##
############
# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# Install Python executables

# Export information to downstream packages
ament_export_dependencies(${dependencies})

# Install header files
install(DIRECTORY 
  "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION include
)

ament_package()
