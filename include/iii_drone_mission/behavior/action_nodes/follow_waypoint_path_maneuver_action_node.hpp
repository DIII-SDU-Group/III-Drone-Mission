#pragma once

#include <iii_drone_interfaces/action/follow_waypoint_path.hpp>

#include <iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/port_types.hpp>

namespace iii_drone::behavior {

class FollowWaypointPathManeuverActionNode : public ManeuverActionNode<
    iii_drone_interfaces::action::FollowWaypointPath
> {
public:
    using Action = iii_drone_interfaces::action::FollowWaypointPath;
    using Goal = Action::Goal;

    FollowWaypointPathManeuverActionNode(
        const std::string & name,
        const BT::NodeConfig & config,
        const BT::RosNodeParams & params,
        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
    );

    static BT::PortsList providedPorts();
    bool setGoal(Goal & goal) override;
    BT::NodeStatus onFeedback(
        const std::shared_ptr<const Action::Feedback> feedback
    ) override;

private:
    iii_drone::control::Reference getFinalReference(
        const BT::RosActionNode<Action>::WrappedResult & result
    ) const;
};

}  // namespace iii_drone::behavior
