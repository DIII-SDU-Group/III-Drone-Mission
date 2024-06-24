/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/hover_on_cable_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::configuration;
using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

HoverOnCableManeuverActionNode::HoverOnCableManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    ParameterBundle::SharedPtr parameter_bundle
) : ManeuverActionNode<iii_drone_interfaces::action::HoverOnCable>(
        name, 
        conf, 
        params,
        maneuver_reference_client
),  parameter_bundle_(parameter_bundle) { }

PortsList HoverOnCableManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<int>("target_cable_id", "The target cable ID"),
        InputPort<float>("duration_s", 1., "Duration of the hover maneuver in seconds"),
        InputPort<bool>("sustain_action", false, "Sustain the action for the duration of the action")
    });

}

bool HoverOnCableManeuverActionNode::setGoal(Goal & goal) {

    RCLCPP_INFO(node_->get_logger(), "HoverOnCableManeuverActionNode::setGoal()");
    
    getInput("target_cable_id", goal.target_cable_id);
    getInput("duration_s", goal.duration_s);
    getInput("sustain_action", goal.sustain_action);

    int stop_maneuver_after_timeout_ms;
    getInput("stop_maneuver_after_timeout_ms", stop_maneuver_after_timeout_ms);

    if (goal.sustain_action && stop_maneuver_after_timeout_ms > 0) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "HoverOnCableManeuverActionNode::setGoal(): %s: Stop maneuver after timeout can not be positive when sustaining the action",
            name_.c_str()
        );

        return false;
    }

    goal.target_z_velocity = parameter_bundle_->GetParameter("hover_on_cable_target_z_velocity").as_double();
    goal.target_yaw_rate = parameter_bundle_->GetParameter("hover_on_cable_target_yaw_rate").as_double();

    if (goal.duration_s <= 0) {
        return false;
    }

    if (goal.target_z_velocity < 0) {
        return false;
    }

    return true;

}