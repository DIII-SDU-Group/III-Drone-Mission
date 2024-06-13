/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/action_nodes/cable_landing_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control::maneuver;

using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

CableLandingManeuverActionNode::CableLandingManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : ManeuverActionNode<iii_drone_interfaces::action::CableLanding>(
        name, 
        conf, 
        params,
        maneuver_reference_client
) { }

PortsList CableLandingManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<int>("target_cable_id")
    });

}

bool CableLandingManeuverActionNode::setGoal(Goal & goal) {
    
    getInput("target_cable_id", goal.target_cable_id);

    if (goal.target_cable_id < 0) {
        return false;
    }

    return true;

}