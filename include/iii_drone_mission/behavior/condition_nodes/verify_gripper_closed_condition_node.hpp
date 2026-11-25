#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <optional>

/*****************************************************************************/
// III-Drone-Mission:

// #include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/gripper_status.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class VerifyGripperClosedConditionNode : public BT::RosTopicSubNode<iii_drone_interfaces::msg::GripperStatus> {
    public:
        VerifyGripperClosedConditionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params
        );

        BT::NodeStatus onTick(const std::shared_ptr<iii_drone_interfaces::msg::GripperStatus> & last_msg) override;

        /**
         * @brief How long the gripper status may be missing before it is
         * reported: the first tick of a fresh subscription has no sample yet
         * and the tree retries the check (2026-11-25 wind runs).
         */
        static constexpr std::chrono::milliseconds kMissingStatusWarnAfter{1000};

    private:
        rclcpp::Node::SharedPtr node_ptr_;

        std::optional<std::chrono::steady_clock::time_point> missing_since_;

    };

} // namespace behavior
}  // namespace iii_drone