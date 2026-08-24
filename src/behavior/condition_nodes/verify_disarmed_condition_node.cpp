/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/verify_disarmed_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

VerifyDisarmedConditionNode::VerifyDisarmedConditionNode(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosTopicSubNode<px4_msgs::msg::VehicleStatus>(
        name,
        conf,
        params,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
),  node_ptr_(params.nh.lock()) { }

NodeStatus VerifyDisarmedConditionNode::onTick(const std::shared_ptr<px4_msgs::msg::VehicleStatus> & last_msg) {

    if (!last_msg) {
        RCLCPP_WARN(
            node_ptr_->get_logger(),
            "VerifyDisarmedConditionNode::onTick(): No vehicle status message received"
        );
        return NodeStatus::FAILURE;
    }

    if (last_msg->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "VerifyDisarmedConditionNode::onTick(): Vehicle is disarmed, arming_state=%u",
            last_msg->arming_state
        );
        return NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "VerifyDisarmedConditionNode::onTick(): Vehicle is still armed"
    );

    return NodeStatus::FAILURE;

}
