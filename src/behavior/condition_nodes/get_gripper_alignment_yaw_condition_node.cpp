/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

using namespace BT;

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

    quaternion_t q_world_to_gripper;
    q_world_to_gripper = quaternionFromQuaternionMsg(last_msg->lines[0].pose.orientation);

    geometry_msgs::msg::TransformStamped T_drone_to_gripper = tf_buffer_->lookupTransform(
        "drone",
        "cable_gripper",
        tf2::TimePointZero
    );

    quaternion_t q_drone_to_gripper = quaternionFromTransformMsg(T_drone_to_gripper.transform);

    quaternion_t q_world_to_drone = quatMultiply(
        q_world_to_gripper,
        quatInv(q_drone_to_gripper)
    );

    euler_angles_t eul_world_to_drone = quatToEul(q_world_to_drone);

    setOutput("target_yaw", eul_world_to_drone[2]);

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "GetGripperAlignmentYawConditionNode::onTick(): Target gripper alignment yaw: %f", 
        eul_world_to_drone[2]
    );

    return NodeStatus::SUCCESS;

}