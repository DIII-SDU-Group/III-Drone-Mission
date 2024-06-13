/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp>

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
// ) : RosActionNode<iii_drone_interfaces::action::HoverByObject>(
) : ManeuverActionNode<iii_drone_interfaces::action::HoverByObject>(
        name, 
        conf, 
        params,
        maneuver_reference_client
) { }

PortsList HoverByObjectManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<float>("duration_s", 1., "Duration of the hover maneuver in seconds"),
        InputPort<iii_drone_interfaces::msg::Target>("target", iii_drone_interfaces::msg::Target(), "Target to hover by")
    });

}

bool HoverByObjectManeuverActionNode::setGoal(Goal & goal) {

    RCLCPP_INFO(node_->get_logger(), "HoverByObjectManeuverActionNode::setGoal()");
    
    getInput("duration_s", goal.duration_s);
    getInput("target", goal.target);

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