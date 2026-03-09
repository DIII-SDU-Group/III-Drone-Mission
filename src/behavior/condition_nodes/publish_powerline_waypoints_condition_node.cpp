/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/publish_powerline_waypoints_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PublishPowerlineWaypointsConditionNode::PublishPowerlineWaypointsConditionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosTopicPubNode<nav_msgs::msg::Path>(
        name, 
        conf, 
        params
) { }

PortsList PublishPowerlineWaypointsConditionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<SharedQueue<point_t>>("waypoints"),
        InputPort<float>("target_yaw")
    });

}

bool PublishPowerlineWaypointsConditionNode::setMessage(nav_msgs::msg::Path & msg) {

    // Get the waypoints.
    SharedQueue<point_t> waypoints;
    if (!getInput("waypoints", waypoints)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PublishPowerlineWaypointsConditionNode::setMessage(): Failed to get waypoints"
        );
        return false;
    }

    if (waypoints->size() == 0) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PublishPowerlineWaypointsConditionNode::setMessage(): No waypoints"
        );
        return false;
    }

    // Get the target yaw.
    float target_yaw;
    if (!getInput("target_yaw", target_yaw)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PublishPowerlineWaypointsConditionNode::setMessage(): Failed to get target yaw"
        );
        return false;
    }

    // Set the header.
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "world";

    // Set the poses.
    for (unsigned int i = 0; i < waypoints->size(); i++) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = msg.header;

        pose_stamped.pose.position = pointMsgFromPoint(waypoints->at(i));
        pose_stamped.pose.orientation = quaternionMsgFromQuaternion(
            eulToQuat(
                euler_angles_t(0.0, 0.0, target_yaw)
            )
        );

        msg.poses.push_back(pose_stamped);
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "PublishPowerlineWaypointsConditionNode::setMessage(): Waypoints set"
    );

    return true;

}
