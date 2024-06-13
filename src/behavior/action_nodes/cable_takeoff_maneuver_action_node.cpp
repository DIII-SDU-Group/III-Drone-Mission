/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/action_nodes/cable_takeoff_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control::maneuver;

using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

CableTakeoffManeuverActionNode::CableTakeoffManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle
) : ManeuverActionNode<iii_drone_interfaces::action::CableTakeoff>(
        name, 
        conf, 
        params,
        maneuver_reference_client
),  parameter_bundle_(parameter_bundle) { }

PortsList CableTakeoffManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<int>("target_cable_id"),
        InputPort<float>("target_cable_distance")
    });

}

bool CableTakeoffManeuverActionNode::setGoal(Goal & goal) {
    
    getInput("target_cable_id", goal.target_cable_id);
    getInput("target_cable_distance", goal.target_cable_distance);    

    if (goal.target_cable_id < 0) {
        return false;
    }

    if (goal.target_cable_distance < parameter_bundle_->GetParameter("cable_takeoff_min_target_cable_distance").as_double() 
        || goal.target_cable_distance > parameter_bundle_->GetParameter("cable_takeoff_max_target_cable_distance").as_double()) {
        return false;
    }

    return true;

}