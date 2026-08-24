#include <iii_drone_mission/behavior/action_nodes/follow_waypoint_path_maneuver_action_node.hpp>

#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/utils/types.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control;
using namespace iii_drone::types;

FollowWaypointPathManeuverActionNode::FollowWaypointPathManeuverActionNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const BT::RosNodeParams & params,
    iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : ManeuverActionNode<Action>(name, config, params, maneuver_reference_client) {
    setGetFinalReferenceCallback(std::bind(
        &FollowWaypointPathManeuverActionNode::getFinalReference,
        this,
        std::placeholders::_1
    ));
}

BT::PortsList FollowWaypointPathManeuverActionNode::providedPorts() {
    return providedManeuverActionNodePorts({
        BT::InputPort<std::string>("frame_id"),
        BT::InputPort<BT::SharedQueue<point_t>>("waypoints"),
        BT::InputPort<int>("repeat_from_index", 0, "First waypoint in the repeating suffix."),
        BT::InputPort<float>("target_yaw"),
        BT::InputPort<bool>("repeat", true, "Repeat from repeat_from_index."),
        BT::InputPort<float>("blend_radius_m", 0.0F, "Non-positive uses configured radius."),
        BT::InputPort<float>("nominal_speed_m_s", 0.0F, "Non-positive uses configured speed."),
        BT::InputPort<float>("max_acceleration_m_s2", 0.0F, "Non-positive uses configured acceleration."),
        BT::InputPort<float>("max_jerk_m_s3", 0.0F, "Non-positive uses configured jerk."),
        BT::InputPort<std::string>("selected_route", "", "Canonical inspection route name."),
        BT::InputPort<int>("loop_route_offset", 0, "Canonical offset of waypoint zero."),
        BT::OutputPort<bool>("resume_valid"),
        BT::OutputPort<std::string>("resume_selected_route"),
        BT::OutputPort<int>("resume_waypoint_index"),
        BT::OutputPort<int>("resume_loop_start_index"),
        BT::OutputPort<int>("resume_route_offset"),
        BT::OutputPort<point_t>("resume_position"),
    });
}

bool FollowWaypointPathManeuverActionNode::setGoal(Goal & goal) {
    BT::SharedQueue<point_t> waypoints;
    int repeat_from_index = 0;
    float target_yaw = 0.0F;
    if (
        !getInput("waypoints", waypoints) || !waypoints || waypoints->empty() ||
        !getInput("repeat_from_index", repeat_from_index) ||
        !getInput("target_yaw", target_yaw)
    ) {
        return false;
    }

    if (!getInput("frame_id", goal.frame_id)) {
        return false;
    }
    getInput("repeat", goal.repeat);
    getInput("nominal_speed_m_s", goal.nominal_speed_m_s);
    getInput("max_acceleration_m_s2", goal.max_acceleration_m_s2);
    getInput("max_jerk_m_s3", goal.max_jerk_m_s3);
    if (repeat_from_index < 0 || repeat_from_index >= static_cast<int>(waypoints->size())) {
        return false;
    }
    goal.repeat_from_index = static_cast<uint32_t>(repeat_from_index);

    float blend_radius = 0.0F;
    getInput("blend_radius_m", blend_radius);
    goal.waypoints.clear();
    goal.waypoints.reserve(waypoints->size());
    for (std::size_t index = 0; index < waypoints->size(); ++index) {
        iii_drone_interfaces::msg::Waypoint waypoint;
        waypoint.position = pointMsgFromPoint(waypoints->at(index));
        waypoint.yaw = target_yaw;
        const bool prefix_stop =
            index < static_cast<std::size_t>(repeat_from_index) &&
            index + 1 == static_cast<std::size_t>(repeat_from_index);
        const bool final_stop = !goal.repeat && index + 1 == waypoints->size();
        waypoint.transition_mode = prefix_stop || final_stop
            ? iii_drone_interfaces::msg::Waypoint::TRANSITION_STOP
            : iii_drone_interfaces::msg::Waypoint::TRANSITION_BLEND;
        waypoint.blend_radius_m = blend_radius;
        waypoint.speed_limit_m_s = 0.0F;
        goal.waypoints.push_back(waypoint);
    }
    return true;
}

BT::NodeStatus FollowWaypointPathManeuverActionNode::onFeedback(
    const std::shared_ptr<const Action::Feedback> feedback
) {
    std::string selected_route;
    int loop_start_index = 0;
    int loop_route_offset = 0;
    if (
        !feedback ||
        !getInput("selected_route", selected_route) || selected_route.empty() ||
        !getInput("repeat_from_index", loop_start_index) || loop_start_index < 0 ||
        !getInput("loop_route_offset", loop_route_offset) || loop_route_offset < 0
    ) {
        return BT::NodeStatus::RUNNING;
    }

    const auto & position = feedback->vehicle_pose.pose.position;
    point_t interrupted_position;
    interrupted_position << position.x, position.y, position.z;
    setOutput("resume_selected_route", selected_route);
    setOutput("resume_waypoint_index", static_cast<int>(feedback->active_waypoint_index));
    setOutput("resume_loop_start_index", loop_start_index);
    setOutput("resume_route_offset", loop_route_offset);
    setOutput("resume_position", interrupted_position);
    setOutput("resume_valid", true);
    return BT::NodeStatus::RUNNING;
}

Reference FollowWaypointPathManeuverActionNode::getFinalReference(
    const BT::RosActionNode<Action>::WrappedResult & result
) const {
    return iii_drone::adapters::ReferenceAdapter(result.result->target_reference).reference();
}
