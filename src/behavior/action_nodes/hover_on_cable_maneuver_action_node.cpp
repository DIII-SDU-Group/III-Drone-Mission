/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/action_nodes/hover_on_cable_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control::maneuver;
using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

HoverOnCableManeuverActionNode::HoverOnCableManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : ManeuverActionNode<iii_drone_interfaces::action::HoverOnCable>(
        name, 
        conf, 
        params,
        maneuver_reference_client
) { }

PortsList HoverOnCableManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<float>("duration_s", 1., "Duration of the hover maneuver in seconds"),
        InputPort<float>("target_z_velocity", 0., "Target z velocity"),
        InputPort<float>("target_yaw_rate", 0., "Target yaw rate")
    });

}

bool HoverOnCableManeuverActionNode::setGoal(Goal & goal) {

    RCLCPP_INFO(node_->get_logger(), "HoverOnCableManeuverActionNode::setGoal()");
    
    getInput("duration_s", goal.duration_s);
    getInput("target_z_velocity", goal.target_z_velocity);
    getInput("target_yaw_rate", goal.target_yaw_rate);

    if (goal.duration_s <= 0) {
        return false;
    }

    if (goal.target_z_velocity < 0) {
        return false;
    }

    return true;

}