/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.hpp>

#include <cmath>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

using namespace BT;

namespace {

double wrapToPi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double wrapCableAxisYawError(double angle) {
    angle = wrapToPi(angle);
    if (angle > M_PI_2) {
        angle -= M_PI;
    } else if (angle < -M_PI_2) {
        angle += M_PI;
    }
    return angle;
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GetGripperAlignmentYawConditionNode::GetGripperAlignmentYawConditionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer
) : RosTopicSubNode<iii_drone_interfaces::msg::Powerline>(
        name, 
        conf, 
        params
),  node_(params.nh.lock()),
    tf_buffer_(tf_buffer) { }

PortsList GetGripperAlignmentYawConditionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<int>("target_cable_id", -1, "Target cable ID. Uses the first line when negative."),
        OutputPort<float>("target_yaw")
    });

}

NodeStatus GetGripperAlignmentYawConditionNode::onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) {

    RCLCPP_DEBUG(node_->get_logger(), "GetGripperAlignmentYawConditionNode::onTick(): Ticking.");

    if (!last_msg) {
        RCLCPP_DEBUG(node_->get_logger(), "GetGripperAlignmentYawConditionNode::onTick(): No powerline message received.");
        return NodeStatus::FAILURE;
    }

    if (last_msg->lines.size() == 0) {
        RCLCPP_DEBUG(node_->get_logger(), "GetGripperAlignmentYawConditionNode::onTick(): No lines in powerline message.");
        return NodeStatus::FAILURE;
    }

    int target_cable_id = -1;
    getInput("target_cable_id", target_cable_id);

    RCLCPP_DEBUG(
        node_->get_logger(),
        "GetGripperAlignmentYawConditionNode::onTick(): Requested target_cable_id=%d with %zu detected line(s)",
        target_cable_id,
        last_msg->lines.size()
    );

    PowerlineAdapter powerline_adapter(*last_msg);

    SingleLineAdapter target_line;
    try {
        if (target_cable_id >= 0 && powerline_adapter.HasLine(target_cable_id)) {
            target_line = powerline_adapter.GetLine(target_cable_id);
        } else {
            target_line = powerline_adapter.single_line_adapters().front();
        }
    } catch (std::runtime_error & ex) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GetGripperAlignmentYawConditionNode::onTick(): Could not select target line: %s",
            ex.what()
        );
        return NodeStatus::FAILURE;
    }

    geometry_msgs::msg::QuaternionStamped source_q_cable_msg;
    source_q_cable_msg.header.stamp = target_line.stamp();
    source_q_cable_msg.header.frame_id = target_line.frame_id();
    source_q_cable_msg.quaternion = quaternionMsgFromQuaternion(target_line.quaternion());

    geometry_msgs::msg::QuaternionStamped gripper_q_cable_msg;
    try {
        gripper_q_cable_msg = tf_buffer_->transform(
            source_q_cable_msg,
            "cable_gripper"
        );
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GetGripperAlignmentYawConditionNode::onTick(): Could not transform cable quaternion to gripper frame: %s",
            ex.what()
        );
        return NodeStatus::FAILURE;
    }

    geometry_msgs::msg::TransformStamped world_T_drone_msg;
    try {
        world_T_drone_msg = tf_buffer_->lookupTransform(
            "world",
            "drone",
            tf2::TimePointZero
        );
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GetGripperAlignmentYawConditionNode::onTick(): Could not transform drone to world frame: %s",
            ex.what()
        );
        return NodeStatus::FAILURE;
    }

    const double raw_yaw_error = quatToEul(quaternionFromQuaternionMsg(gripper_q_cable_msg.quaternion))[2];
    const double yaw_error = wrapCableAxisYawError(raw_yaw_error);
    const double current_yaw = quatToEul(quaternionFromTransformMsg(world_T_drone_msg.transform))[2];
    const double target_yaw = current_yaw + yaw_error;

    setOutput("target_yaw", static_cast<float>(target_yaw));

    RCLCPP_DEBUG(
        node_->get_logger(),
        "GetGripperAlignmentYawConditionNode::onTick(): Selected line id=%d position=[%.3f, %.3f, %.3f] frame=%s",
        target_line.id(),
        target_line.position()[0],
        target_line.position()[1],
        target_line.position()[2],
        target_line.frame_id().c_str()
    );

    RCLCPP_INFO(
        node_->get_logger(),
        "GetGripperAlignmentYawConditionNode::onTick(): Target line id=%d frame=%s current_yaw=%f raw_gripper_yaw_error=%f yaw_error=%f target_yaw=%f",
        target_line.id(),
        target_line.frame_id().c_str(),
        current_yaw,
        raw_yaw_error,
        yaw_error,
        target_yaw
    );

    return NodeStatus::SUCCESS;

}
