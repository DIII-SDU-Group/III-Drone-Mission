#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <chrono>
#include <memory>
#include <thread>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>
#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/behavior/action_nodes/hover_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/hover_on_cable_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/fly_to_object_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/fly_to_position_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/cable_landing_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/cable_takeoff_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/pl_mapper_command_action_node.hpp>

#include <iii_drone_core/behavior/condition_nodes/verify_powerline_detected_condition_node.hpp>
#include <iii_drone_core/behavior/condition_nodes/select_target_line_condition_node.hpp>

#include <iii_drone_core/behavior/port_types.hpp>

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/px4/mode_executors/generic_mode_executor.hpp>
#include <iii_drone_core/px4/modes/maneuver_mode.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/bt_factory.h>

#include <behaviortree_ros2/ros_node_params.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class TakeChargingPositionTree {
    public:
        TakeChargingPositionTree(
            iii_drone::configuration::Configurator::SharedPtr configurator,
            rclcpp::Node * node,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
        );

        void StartExecution();

        void StopExecution();

    private:
        iii_drone::configuration::Configurator::SharedPtr configurator_;

        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        rclcpp::Node * node_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        BT::BehaviorTreeFactory factory_;

        BT::Tree tree_;

        iii_drone::px4::GenericModeExecutor::UniquePtr mode_executor_;

        iii_drone::px4::ManeuverMode::UniquePtr take_charging_position_mode_;
        iii_drone::px4::ManeuverMode::UniquePtr hover_on_cable_mode_;
        iii_drone::px4::ManeuverMode::UniquePtr hover_mode_;

        void initPX4();

        std::thread execute_thread_;

        iii_drone::utils::Atomic<bool> running_ = false;
        iii_drone::utils::Atomic<bool> finished_ = false;
        iii_drone::utils::Atomic<bool> success_ = false;

        void execute();

        void registerNodes();



    };

} // namespace behavior
} // namespace iii_drone