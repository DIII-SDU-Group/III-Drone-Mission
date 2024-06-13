/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/action_nodes/hover_maneuver_action_node.hpp>

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
        InputPort<float>("duration_s", 1., "Duration of the hover maneuver in seconds")
    });

}

bool HoverManeuverActionNode::setGoal(Goal & goal) {
    
    getInput("duration_s", goal.duration_s);

    return goal.duration_s > 0;

}