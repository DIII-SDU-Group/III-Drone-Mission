/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/action_nodes/fly_to_position_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;
using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

FlyToPositionManeuverActionNode::FlyToPositionManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : ManeuverActionNode<iii_drone_interfaces::action::FlyToPosition>(
        name, 
        conf, 
        params,
        maneuver_reference_client
) { 

    setGetFinalReferenceCallback(
        std::bind(
            &FlyToPositionManeuverActionNode::getFinalReference, 
            this, 
            std::placeholders::_1
        )
    );

}

PortsList FlyToPositionManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<std::string>("frame_id"),
        InputPort<geometry_msgs::msg::Point>("target_position"),
        InputPort<float>("target_yaw")
    });

}

bool FlyToPositionManeuverActionNode::setGoal(Goal & goal) {
    
    // getInput("frame_id", goal.frame_id);
    // getInput("target_position", goal.target_position);
    // getInput("target_yaw", goal.target_yaw);

    goal.frame_id = "drone";
    goal.target_position.x = 1.;
    goal.target_position.y = 1.;
    goal.target_position.z = 0.;
    goal.target_yaw = 0.;

    return true;

}

Reference FlyToPositionManeuverActionNode::getFinalReference(const typename BT::RosActionNode<iii_drone_interfaces::action::FlyToPosition>::WrappedResult & wr) const {

    return ReferenceAdapter(wr.result->target_reference).reference();

}