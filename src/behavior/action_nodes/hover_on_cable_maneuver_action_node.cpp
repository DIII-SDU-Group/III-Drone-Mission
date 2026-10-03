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
    Configuration::SharedPtr configuration
) : ManeuverActionNode<iii_drone_interfaces::action::HoverOnCable>(
        name, 
        conf, 
        params,
        maneuver_reference_client
),  configuration_(configuration) { }

PortsList HoverOnCableManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<int>("target_cable_id", "The target cable ID"),
        InputPort<float>("duration_s", 1., "Duration of the hover maneuver in seconds"),
        InputPort<bool>("sustain_action", false, "Sustain the action for the duration of the action"),
        InputPort<float>("target_upwards_velocity"),
        InputPort<bool>(
            "cable_release_push", false,
            "Push up with /behavior/cable_release_push_acceleration (an acceleration, not a velocity) to hold the cable while the gripper opens")
    });

}

bool HoverOnCableManeuverActionNode::setManeuverGoal(Goal & goal) {

    RCLCPP_INFO(node_ptr_->get_logger(), "HoverOnCableManeuverActionNode::setManeuverGoal()");
    
    getInput("target_cable_id", goal.target_cable_id);
    getInput("duration_s", goal.duration_s);
    getInput("sustain_action", goal.sustain_action);

    int stop_maneuver_after_timeout_ms;
    getInput("stop_maneuver_after_timeout_ms", stop_maneuver_after_timeout_ms);

    bool cable_release_push = false;
    getInput("cable_release_push", cable_release_push);

    // A cable release push succeeds once established and must then keep
    // holding the cable until CableTakeoff takes over: it keeps its reference
    // for stop_maneuver_after_timeout_ms after success. A plain sustained hover
    // ends with its duration.
    if (cable_release_push && (!goal.sustain_action || stop_maneuver_after_timeout_ms <= 0)) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverOnCableManeuverActionNode::setManeuverGoal(): %s: a cable release push must sustain the action and keep its reference after success (positive stop_maneuver_after_timeout_ms)",
            name_.c_str()
        );
        return false;
    }

    if (goal.sustain_action && !cable_release_push && stop_maneuver_after_timeout_ms > 0) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "HoverOnCableManeuverActionNode::setManeuverGoal(): %s: Stop maneuver after timeout can not be positive when sustaining the action",
            name_.c_str()
        );

        return false;
    }

    if (!getInput("target_upwards_velocity", goal.target_z_velocity)) {
        goal.target_z_velocity = configuration_->GetParameter("/behavior/hover_on_cable_target_z_velocity").as_double();
    }

    goal.target_yaw_rate = configuration_->GetParameter("/behavior/hover_on_cable_target_yaw_rate").as_double();

    goal.push_upwards_acceleration = cable_release_push
        ? configuration_->GetParameter("/behavior/cable_release_push_acceleration").as_double()
        : 0.0;

    if (goal.duration_s <= 0) {
        return false;
    }

    if (goal.target_z_velocity < 0) {
        return false;
    }

    return true;

}
