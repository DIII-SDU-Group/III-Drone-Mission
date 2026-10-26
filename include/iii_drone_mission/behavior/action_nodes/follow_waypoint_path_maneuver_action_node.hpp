#pragma once

#include <iii_drone_interfaces/action/follow_waypoint_path.hpp>
#include <cstddef>
#include <cstdint>

#include <iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/port_types.hpp>

namespace iii_drone::behavior {

inline uint8_t FollowWaypointPathTransitionMode(
    std::size_t index,
    std::size_t waypoint_count,
    bool repeat,
    std::size_t repeat_from_index
) {
    const bool prefix_stop = index < repeat_from_index && index + 1 == repeat_from_index;
    const bool final_stop = !repeat && index + 1 == waypoint_count;
    return prefix_stop || final_stop
        ? iii_drone_interfaces::msg::Waypoint::TRANSITION_STOP
        : iii_drone_interfaces::msg::Waypoint::TRANSITION_BLEND;
}

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
    bool setManeuverGoal(Goal & goal) override;
    BT::NodeStatus onFeedback(
        const std::shared_ptr<const Action::Feedback> feedback
    ) override;
    void onHalt() override;

private:
    bool shouldStopManeuverOnSuccessfulResult(
        const typename BT::RosActionNode<Action>::WrappedResult & wr
    ) const override;
    iii_drone::control::Reference getFinalReference(
        const BT::RosActionNode<Action>::WrappedResult & result
    ) const;
};

}  // namespace iii_drone::behavior
