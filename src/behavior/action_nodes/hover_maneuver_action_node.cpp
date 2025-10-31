/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/hover_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control::maneuver;
using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

HoverManeuverActionNode::HoverManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : ManeuverActionNode<iii_drone_interfaces::action::Hover>(
        name, 
        conf, 
        params,
        maneuver_reference_client
) { }

PortsList HoverManeuverActionNode::providedPorts() {
    
    return providedManeuverActionNodePorts({
        InputPort<float>("duration_s", 1., "Duration of the hover maneuver in seconds"),
        InputPort<float>("sustain_duration_s", 0., "Duration to sustain the action if not fully"),
        InputPort<bool>("sustain_action", false, "Sustain the action for the duration")
    });

}

bool HoverManeuverActionNode::setGoal(Goal & goal) {
    
    getInput("duration_s", goal.duration_s);
    getInput("sustain_duration_s", goal.sustain_duration_s);
    getInput("sustain_action", goal.sustain_action);
    
    int stop_maneuver_after_timeout_ms;
    getInput("stop_maneuver_after_timeout_ms", stop_maneuver_after_timeout_ms);

    if (goal.sustain_action && stop_maneuver_after_timeout_ms > 0) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverManeuverActionNode::setGoal(): %s: Stop maneuver after timeout can not be positive when sustaining the action",
            name_.c_str()
        );

        return false;
    }

    if (stop_maneuver_after_timeout_ms > (goal.duration_s - goal.sustain_duration_s) * 1000) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverManeuverActionNode::setGoal(): %s: Stop maneuver after timeout is greater than the time left after stopping sustaining the action",
            name_.c_str()
        );

        return false;
    }

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "HoverManeuverActionNode::setGoal(): %s: Setting goal: duration_s = %f, sustain_duration_s = %f, sustain_action = %d",
        name_.c_str(),
        goal.duration_s,
        goal.sustain_duration_s,
        goal.sustain_action
    );

    return goal.duration_s > 0;

}