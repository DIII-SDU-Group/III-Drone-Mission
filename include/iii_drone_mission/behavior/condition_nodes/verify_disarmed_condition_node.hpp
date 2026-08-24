#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_status.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class VerifyDisarmedConditionNode : public BT::RosTopicSubNode<px4_msgs::msg::VehicleStatus> {
    public:
        VerifyDisarmedConditionNode(
            const std::string & name,
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params
        );

        BT::NodeStatus onTick(const std::shared_ptr<px4_msgs::msg::VehicleStatus> & last_msg) override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;

    };

} // namespace behavior
} // namespace iii_drone
