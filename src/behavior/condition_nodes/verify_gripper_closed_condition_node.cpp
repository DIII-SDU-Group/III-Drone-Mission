/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/verify_gripper_closed_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

VerifyGripperClosedConditionNode::VerifyGripperClosedConditionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosTopicSubNode<iii_drone_interfaces::msg::GripperStatus>(
        name, 
        conf, 
        params,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
),  node_ptr_(params.nh) { }

NodeStatus VerifyGripperClosedConditionNode::onTick(const std::shared_ptr<iii_drone_interfaces::msg::GripperStatus> & last_msg) {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "VerifyGripperClosedConditionNode::onTick(): Checking gripper closed"
    );

    if (!last_msg) {
        RCLCPP_WARN(
            node_ptr_->get_logger(),
            "VerifyGripperClosedConditionNode::onTick(): No gripper status message received"
        );
        return NodeStatus::FAILURE;
    }

    if (last_msg->gripper_status == iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "VerifyGripperClosedConditionNode::onTick(): Gripper is closed"
        );
        return NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "VerifyGripperClosedConditionNode::onTick(): Gripper is not closed"
    );

    return NodeStatus::FAILURE;

}