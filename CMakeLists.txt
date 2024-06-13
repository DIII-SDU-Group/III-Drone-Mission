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

# Includes and link:
set(III_DRONE_INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/include/"
)

include_directories(src/matlab_MPC_5Hz_hp10)
link_directories(src/matlab_MPC_5Hz_hp10)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(iii_drone_interfaces REQUIRED)
find_package(iii_drone_configuration REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  Eigen3
  sensor_msgs
  px4_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  cv_bridge
  image_transport
  iii_drone_interfaces
  iii_drone_configuration
  behaviortree_cpp
  behaviortree_ros2
  px4_ros2_cpp
)

# MATLAB MPC Library:
add_subdirectory("${PROJECT_SOURCE_DIR}/src/extern")

# Library files:
set(III_DRONE_LIBRARY_HEADERS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/math.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/types.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/timestamp.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/atomic.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/history.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/token.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/single_line.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/powerline.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/powerline_direction.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/hough_transformer.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/state.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/reference.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/reference_trajectory.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_generator.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_generator_client.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/combined_drone_awareness_handler.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_types.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_queue.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_scheduler.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/hover_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/hover_by_object_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/hover_on_cable_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/fly_to_position_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/fly_to_object_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/cable_landing_maneuver_server.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/cable_takeoff_maneuver_server.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver/maneuver_reference_client.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/single_line_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/projection_plane_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/powerline_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/point_cloud_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/state_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/reference_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/reference_trajectory_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/gripper_status_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/target_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/maneuver_adapter.hpp"

  # # "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/offboard_control_mode_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/adapters/px4/vehicle_status_adapter.hpp"

)

set(III_DRONE_LIBRARY_SOURCES
  "${PROJECT_SOURCE_DIR}/src/utils/math.cpp"
  "${PROJECT_SOURCE_DIR}/src/utils/types.cpp"
  "${PROJECT_SOURCE_DIR}/src/utils/timestamp.cpp"

  "${PROJECT_SOURCE_DIR}/src/perception/single_line.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/powerline.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/powerline_direction.cpp"
  "${PROJECT_SOURCE_DIR}/src/perception/hough_transformer.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/state.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/reference.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/reference_trajectory.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_generator.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_generator_client.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/combined_drone_awareness_handler.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_types.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_queue.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_scheduler.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/hover_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/hover_by_object_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/hover_on_cable_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/fly_to_position_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/fly_to_object_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/cable_landing_maneuver_server.cpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver/cable_takeoff_maneuver_server.cpp"

  "${PROJECT_SOURCE_DIR}/src/control/maneuver/maneuver_reference_client.cpp"

  "${PROJECT_SOURCE_DIR}/src/adapters/single_line_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/projection_plane_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/powerline_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/point_cloud_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/state_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/reference_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/reference_trajectory_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/gripper_status_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/target_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/maneuver_adapter.cpp"

  ## "${PROJECT_SOURCE_DIR}/src/adapters/px4/offboard_control_mode_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/px4/trajectory_setpoint_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/px4/vehicle_odometry_adapter.cpp"
  "${PROJECT_SOURCE_DIR}/src/adapters/px4/vehicle_status_adapter.cpp"

)

set(III_DRONE_PX4_HDRS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/px4/modes/maneuver_mode.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/px4/mode_executors/generic_mode_executor.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/px4/setpoints/trajectory_setpoint.hpp"

)

set(III_DRONE_PX4_SRCS
  "${PROJECT_SOURCE_DIR}/src/px4/modes/maneuver_mode.cpp"

  "${PROJECT_SOURCE_DIR}/src/px4/mode_executors/generic_mode_executor.cpp"

  "${PROJECT_SOURCE_DIR}/src/px4/setpoints/trajectory_setpoint.cpp"

)

set(III_DRONE_BEHAVIOR_HDRS
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/maneuver_action_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/hover_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/hover_on_cable_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/fly_to_position_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/fly_to_object_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/cable_landing_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/cable_takeoff_maneuver_action_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/action_nodes/pl_mapper_command_action_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/condition_nodes/verify_powerline_detected_condition_node.hpp"
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/condition_nodes/select_target_line_condition_node.hpp"

  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/behavior/trees/take_charging_position_tree.hpp"
)

set(III_DRONE_BEHAVIOR_SRCS
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/maneuver_action_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_by_object_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/hover_on_cable_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/fly_to_position_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/fly_to_object_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/cable_landing_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/cable_takeoff_maneuver_action_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/action_nodes/pl_mapper_command_action_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/verify_powerline_detected_condition_node.cpp"
  "${PROJECT_SOURCE_DIR}/src/behavior/condition_nodes/select_target_line_condition_node.cpp"

  "${PROJECT_SOURCE_DIR}/src/behavior/trees/take_charging_position_tree.cpp"

)

include_directories(
  ${III_DRONE_INCLUDE_DIRS}
  ${MATLAB_MPC_INCLUDE_DIRS}  
)

# III-Drone-Lib:
add_library(iii_drone_lib 
  SHARED
  ${III_DRONE_LIBRARY_HEADERS}
  ${III_DRONE_LIBRARY_SOURCES}
)

target_sources(iii_drone_lib
  PUBLIC
  ${III_DRONE_LIBRARY_HEADERS}
  ${III_DRONE_LIBRARY_SOURCES}
)

target_include_directories(iii_drone_lib
  PUBLIC
  ${III_DRONE_INCLUDE_DIRS}
)
target_include_directories(iii_drone_lib
  PRIVATE
  ${MATLAB_MPC_INCLUDE_DIRS}
)

target_link_libraries(iii_drone_lib
  matlab_mpc
  # yaml-cpp::yaml-cpp
)

ament_target_dependencies(iii_drone_lib
  ${dependencies}
)

ament_export_libraries(iii_drone_lib)
ament_export_include_directories(${III_DRONE_INCLUDE_DIRS})

install(TARGETS iii_drone_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# # III-Drone-PX4:
# add_library(iii_drone_px4_lib 
#   SHARED
#   ${III_DRONE_PX4_HDRS}
#   ${III_DRONE_PX4_SRCS}
# )

# target_sources(iii_drone_px4_lib
#   PUBLIC
#   ${III_DRONE_PX4_HDRS}
#   ${III_DRONE_PX4_SRCS}
# )

# target_include_directories(iii_drone_px4_lib
#   PUBLIC
#   ${III_DRONE_INCLUDE_DIRS}
# )

# target_link_libraries(iii_drone_px4_lib
#   iii_drone_lib
# )

# ament_target_dependencies(iii_drone_px4_lib
#   ${dependencies}
# )

# ament_export_libraries(iii_drone_px4_lib)

# install(TARGETS iii_drone_px4_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# # III-Drone-Behavior:
# add_library(iii_drone_behavior_lib 
#   SHARED
#   ${III_DRONE_BEHAVIOR_HDRS}
#   ${III_DRONE_BEHAVIOR_SRCS}
# )

# target_sources(iii_drone_behavior_lib
#   PUBLIC
#   ${III_DRONE_BEHAVIOR_HDRS}
#   ${III_DRONE_BEHAVIOR_SRCS}
# )

# target_include_directories(iii_drone_behavior_lib
#   PUBLIC
#   ${III_DRONE_INCLUDE_DIRS}
# )

# target_link_libraries(iii_drone_behavior_lib
#   iii_drone_lib
#   iii_drone_px4_lib
# )

# ament_target_dependencies(iii_drone_behavior_lib
#   ${dependencies}
# )

# ament_export_libraries(iii_drone_behavior_lib)
# ament_export_include_directories(${III_DRONE_INCLUDE_DIRS})

# install(TARGETS iii_drone_behavior_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

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
    iii_drone_lib
    # iii_drone_px4_lib
    # iii_drone_behavior_lib
  )

  target_include_directories(${target_name}
    PUBLIC
    "${III_DRONE_INCLUDE_DIRS}"
  )

  target_link_libraries(${target_name}
    iii_drone_lib
    # iii_drone_px4_lib
    # iii_drone_behavior_lib
  )

  ament_target_dependencies(${target_name}
    ${dependencies}
  )

  target_link_libraries(${target_name} "${cpp_typesupport_target}")

  install(TARGETS ${target_name}
    DESTINATION lib/${PROJECT_NAME}
  )

endfunction()

# Drone frame broadcaster:
custom_cpp_target(drone_frame_broadcaster
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/utils/drone_frame_broadcaster_node/drone_frame_broadcaster_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/utils/drone_frame_broadcaster_node/drone_frame_broadcaster_node.cpp"
)

# Hough transformer:
custom_cpp_target(hough_transformer
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/hough_transformer_node/hough_transformer_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/perception/hough_transformer_node/hough_transformer_node.cpp"
)

# PL dir computer:
custom_cpp_target(pl_dir_computer
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/pl_dir_computer_node/pl_dir_computer_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/perception/pl_dir_computer_node/pl_dir_computer_node.cpp"
)

# PL mapper:
custom_cpp_target(pl_mapper
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/perception/pl_mapper_node/pl_mapper_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/perception/pl_mapper_node/pl_mapper_node.cpp"
)

# Trajectory generator:
custom_cpp_target(trajectory_generator
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/trajectory_generator_node/trajectory_generator_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/control/trajectory_generator_node/trajectory_generator_node.cpp"
)

# Maneuver controller:
custom_cpp_target(maneuver_controller
  "${PROJECT_SOURCE_DIR}/include/iii_drone_core/control/maneuver_controller_node/maneuver_controller_node.hpp"
  "${PROJECT_SOURCE_DIR}/src/control/maneuver_controller_node/maneuver_controller_node.cpp"
)

# # Continuous mission orchestrator:
# custom_cpp_target(continuous_mission_orchestrator
#   "${PROJECT_SOURCE_DIR}/include/iii_drone_core/mission/continuous_mission_orchestrator_node/continuous_mission_orchestrator_node.hpp"
#   "${PROJECT_SOURCE_DIR}/src/mission/continuous_mission_orchestrator_node/continuous_mission_orchestrator_node.cpp"
# )

# custom_cpp_target(test_tree
#   "${PROJECT_SOURCE_DIR}/src/behavior/trees/test_tree.cpp"
# )

# custom_cpp_target(px4_mode_test
#   "${PROJECT_SOURCE_DIR}/src/px4/px4_mode_test.cpp"
# )

# Tests:
if (BUILD_TESTING)

  # custom_cpp_target(combined_drone_awareness_handler_test
  #   "${PROJECT_SOURCE_DIR}/test/control/combined_drone_awareness_handler_test.cpp"
  # )

endif()

############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  "${PROJECT_SOURCE_DIR}/scripts/payload/charger_gripper_node/charger_gripper_node.py"
  DESTINATION lib/${PROJECT_NAME}
)

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs OpenCV)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs geometry_msgs sensor_msgs)
endif()

# Install header files
install(DIRECTORY 
  "${PROJECT_SOURCE_DIR}/include/"
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# If building tests, merge the test/launch directory with the main launch directory
if (BUILD_TESTING)
  install(DIRECTORY
    test/launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
