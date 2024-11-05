#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <chrono>
#include <memory>
#include <thread>
#include <wordexp.h>

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

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/action_nodes/hover_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/hover_on_cable_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/fly_to_object_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/fly_to_position_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/cable_landing_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/cable_takeoff_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/pl_mapper_command_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/gripper_command_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/update_powerline_overview_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/get_powerline_overview_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/powerline_waypoint_provider_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/mode_executor_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/log_message_action_node.hpp>

#include <iii_drone_mission/behavior/condition_nodes/verify_powerline_detected_condition_node.hpp>
#include <iii_drone_mission/behavior/condition_nodes/select_target_line_condition_node.hpp>
#include <iii_drone_mission/behavior/condition_nodes/target_provider_condition_node.hpp>
#include <iii_drone_mission/behavior/condition_nodes/store_current_state_condition_node.hpp>
#include <iii_drone_mission/behavior/condition_nodes/publish_powerline_waypoints_condition_node.hpp>
#include <iii_drone_mission/behavior/condition_nodes/verify_gripper_closed_condition_node.hpp>
#include <iii_drone_mission/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.hpp>

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/blackboard.h>

#include <behaviortree_cpp/decorators/loop_node.h>

#include <behaviortree_ros2/ros_node_params.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class TreeExecutor {
    public:
        TreeExecutor(
            const std::string & tree_name,
            const std::string & tree_xml_file,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator,
            rclcpp::Node * node,
            BT::Blackboard::Ptr global_blackboard
        );

        ~TreeExecutor();

        void FinalizeInitialization();
        void Deinitialize();

        void StartExecution();

        void StopExecution(bool wait = true);

        bool running() const;
        bool finished() const;
        bool success() const;

        const BT::BehaviorTreeFactory & factory() const;

        typedef std::shared_ptr<TreeExecutor> SharedPtr;

    private:
        std::string tree_name_;
        std::string tree_xml_file_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator_;

        rclcpp::Node * node_;

        BT::BehaviorTreeFactory factory_;

        BT::Tree tree_;

        BT::Blackboard::Ptr global_blackboard_;
        BT::Blackboard::Ptr local_blackboard_;

        std::thread execute_thread_;

        iii_drone::utils::Atomic<bool> running_ = false;
        iii_drone::utils::Atomic<bool> finished_ = false;
        iii_drone::utils::Atomic<bool> success_ = false;

        void execute();

        void registerNodes();
        void unregisterNodes();

    };

} // namespace behavior
} // namespace iii_drone