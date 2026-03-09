/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control::maneuver;
using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

HoverByObjectManeuverActionNode::HoverByObjectManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : ManeuverActionNode<iii_drone_interfaces::action::HoverByObject>(
        name, 
        conf, 
        params,
        maneuver_reference_client
) { }

PortsList HoverByObjectManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<float>("duration_s", 1., "Duration of the hover maneuver in seconds"),
        InputPort<bool>("sustain_action", false, "Sustain the action after the duration"),
        InputPort<iii_drone_interfaces::msg::Target>("target", iii_drone_interfaces::msg::Target(), "Target to hover by")
    });

}

bool HoverByObjectManeuverActionNode::setGoal(Goal & goal) {

    RCLCPP_INFO(node_ptr_->get_logger(), "HoverByObjectManeuverActionNode::setGoal()");
    
    getInput("duration_s", goal.duration_s);
    getInput("target", goal.target);
    getInput("sustain_action", goal.sustain_action);

    int stop_maneuver_after_timeout_ms;

    getInput("stop_maneuver_after_timeout_ms", stop_maneuver_after_timeout_ms);

    if (goal.duration_s <= 0) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverByObjectManeuverActionNode::setGoal(): %s: Duration must be positive",
            name_.c_str()
        );

        return false;
    }

    if (goal.target.target_id < 0) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverByObjectManeuverActionNode::setGoal(): %s: Target ID must be positive",
            name_.c_str()
        );

        return false;
    }

    if (goal.target.target_type == iii_drone_interfaces::msg::Target::TARGET_TYPE_NONE) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverByObjectManeuverActionNode::setGoal(): %s: Target type must be set",
            name_.c_str()
        );

        return false;
    }

    if (goal.sustain_action && stop_maneuver_after_timeout_ms > 0) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverByObjectManeuverActionNode::setGoal(): %s: Stop maneuver after timeout can not be positive when sustaining the action",
            name_.c_str()
        );

        return false;
    }

    if (goal.duration_s <= 0) {
        return false;
    }

    if (goal.target.target_id < 0) {
        return false;
    }

    if (goal.target.target_type == iii_drone_interfaces::msg::Target::TARGET_TYPE_NONE) {
        return false;
    }

    return true;

}
