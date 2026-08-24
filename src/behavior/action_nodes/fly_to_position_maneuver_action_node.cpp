/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/fly_to_position_maneuver_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::control;
using namespace iii_drone::types;
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
        InputPort<point_t>("target_position"),
        InputPort<float>("target_yaw"),
        InputPort<bool>("blend_to_next", false, "Return action success at the reached threshold and let the next FTP blend from the streamed reference"),
        InputPort<bool>("ignore_altitude", false, "Bypass the minimum target altitude check"),
        InputPort<float>("completion_position_tolerance_m", 0.0F, "Override terminal position tolerance; zero uses the configured default")
    });

}

bool FlyToPositionManeuverActionNode::setGoal(Goal & goal) {

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "FlyToPositionManeuverActionNode::setGoal()"
    );
    
    point_t position;
    
    getInput("frame_id", goal.frame_id);
    getInput("target_position", position);
    getInput("target_yaw", goal.target_yaw);
    getInput("blend_to_next", goal.blend_to_next);
    getInput("ignore_altitude", goal.ignore_altitude);
    getInput("completion_position_tolerance_m", goal.completion_position_tolerance_m);
    current_goal_blend_to_next_ = goal.blend_to_next;

    goal.target_position = pointMsgFromPoint(position);

    return true;

}

bool FlyToPositionManeuverActionNode::shouldStopManeuverOnSuccessfulResult(
    const typename BT::RosActionNode<iii_drone_interfaces::action::FlyToPosition>::WrappedResult &
) const {

    if (current_goal_blend_to_next_) {
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "FlyToPositionManeuverActionNode::shouldStopManeuverOnSuccessfulResult(): %s: preserving reference stream for blend_to_next handoff",
            name_.c_str()
        );
    }

    return !current_goal_blend_to_next_;

}

bool FlyToPositionManeuverActionNode::shouldAttachToActiveManeuverStreamOnGoalAccepted() const {

    // An active stream exists only when the preceding maneuver deliberately
    // preserved it for a blended successor. The current goal's blend_to_next
    // controls its outgoing handoff, not whether it accepts the incoming one.
    return true;

}

Reference FlyToPositionManeuverActionNode::getFinalReference(const typename BT::RosActionNode<iii_drone_interfaces::action::FlyToPosition>::WrappedResult & wr) const {

    return ReferenceAdapter(wr.result->target_reference).reference();

}
