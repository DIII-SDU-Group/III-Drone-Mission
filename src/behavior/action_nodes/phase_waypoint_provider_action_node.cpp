/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/phase_waypoint_provider_action_node.hpp>
#include <iii_drone_mission/behavior/powerline_geometry.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <vector>

using namespace iii_drone::adapters;
using namespace iii_drone::behavior;
using namespace iii_drone::configuration;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace BT;
namespace pl_geom = iii_drone::behavior::powerline_geometry;

/*****************************************************************************/
// Helpers
/*****************************************************************************/

namespace {

    constexpr double kPointEpsilonM = 1e-4;
    constexpr double kCentralConductorFractionOfHalfWidth = 0.25;

    point_t xyOnly(point_t point) {
        point[2] = 0.0;
        return point;
    }

    void appendIfMoved(
        std::deque<point_t> & queue,
        const point_t & point,
        const point_t & start_position
    ) {
        const point_t & previous = queue.empty() ? start_position : queue.back();
        if ((previous - point).norm() >= kPointEpsilonM) {
            queue.push_back(point);
        }
    }

    struct RouteCandidate {
        std::array<point_t, 8> waypoints;
        std::string name;
        bool starts_on_positive_side = false;
    };

    std::optional<pl_geom::PowerlineSideSplit> splitInspectionSideConductors(
        const std::vector<point_t> & points,
        const vector_t & cross_corridor_no_z
    ) {
        const auto fallback = pl_geom::SplitByLargestLateralGap(
            points,
            cross_corridor_no_z
        );
        if (!fallback || points.size() < 3) {
            return fallback;
        }

        const auto highest = std::max_element(
            points.begin(),
            points.end(),
            [](const point_t & lhs, const point_t & rhs) {
                return lhs[2] < rhs[2];
            }
        );
        const auto lateral = [&](const point_t & point) {
            return xyOnly(point).dot(cross_corridor_no_z);
        };
        const auto [minimum, maximum] = std::minmax_element(
            points.begin(),
            points.end(),
            [&](const point_t & lhs, const point_t & rhs) {
                return lateral(lhs) < lateral(rhs);
            }
        );

        const double minimum_lateral = lateral(*minimum);
        const double maximum_lateral = lateral(*maximum);
        const double corridor_center = (minimum_lateral + maximum_lateral) / 2.0;
        const double corridor_half_width = (maximum_lateral - minimum_lateral) / 2.0;
        if (
            corridor_half_width <= kPointEpsilonM ||
            std::abs(lateral(*highest) - corridor_center) >
                kCentralConductorFractionOfHalfWidth * corridor_half_width
        ) {
            return fallback;
        }

        std::vector<point_t> side_conductors;
        side_conductors.reserve(points.size() - 1);
        for (auto point = points.begin(); point != points.end(); ++point) {
            if (point != highest) {
                side_conductors.push_back(*point);
            }
        }

        const auto split = pl_geom::SplitByLargestLateralGap(
            side_conductors,
            cross_corridor_no_z
        );
        return split ? split : fallback;
    }

} // namespace

/*****************************************************************************/
// Route generation
/*****************************************************************************/

CorridorInspectionEligibility iii_drone::behavior::EvaluateCorridorInspectionStart(
    const std::vector<point_t> & powerline_points,
    const vector_t & powerline_direction,
    const point_t & pylon_a,
    const point_t & pylon_b,
    const point_t & start_position,
    double inspection_clearance_m,
    double pylon_end_clearance_m,
    double pylon_structure_extent_m,
    double pylon_span_margin_m,
    double max_pylon_direction_mismatch_rad
) {
    CorridorInspectionEligibility result;
    result.required_lateral_clearance_m = inspection_clearance_m;
    result.pylon_span_margin_m = pylon_span_margin_m;
    if (
        powerline_points.size() < 2 ||
        inspection_clearance_m <= 0.0 ||
        pylon_end_clearance_m <= 0.0 ||
        pylon_structure_extent_m < 0.0 ||
        pylon_span_margin_m < 0.0
    ) {
        result.failure_reasons.push_back("inspection geometry or configuration is invalid");
        return result;
    }

    const auto powerline_axes = pl_geom::ComputeAxes(powerline_direction);
    if (!powerline_axes) {
        result.failure_reasons.push_back("powerline direction is invalid");
        return result;
    }
    if (!pl_geom::PylonSpanMatchesPowerlineDirection(
        pylon_a,
        pylon_b,
        powerline_axes->direction_no_z,
        max_pylon_direction_mismatch_rad
    )) {
        result.failure_reasons.push_back("pylon span does not follow the powerline direction");
        return result;
    }

    vector_t corridor_direction = xyOnly(pylon_b - pylon_a);
    if (corridor_direction.dot(powerline_axes->direction_no_z) < 0.0) {
        corridor_direction *= -1.0;
    }
    const auto axes = pl_geom::ComputeAxes(corridor_direction);
    if (!axes) {
        result.failure_reasons.push_back("pylon positions do not define a corridor");
        return result;
    }
    const auto side_split = splitInspectionSideConductors(
        powerline_points,
        axes->cross_corridor_no_z
    );
    if (!side_split) {
        result.failure_reasons.push_back("conductors cannot be split into corridor sides");
        return result;
    }
    const auto positive_outer = pl_geom::FurthestPointXY(
        side_split->positive_points,
        side_split->middle_point
    );
    const auto negative_outer = pl_geom::FurthestPointXY(
        side_split->negative_points,
        side_split->middle_point
    );
    const auto positive_highest = pl_geom::SelectHighestPoint(side_split->positive_points);
    const auto negative_highest = pl_geom::SelectHighestPoint(side_split->negative_points);
    if (!positive_outer || !negative_outer || !positive_highest || !negative_highest) {
        result.failure_reasons.push_back("outer or top side conductors are unavailable");
        return result;
    }

    point_t pylon_start = pylon_a;
    point_t pylon_end = pylon_b;
    if ((xyOnly(pylon_end) - xyOnly(pylon_start)).dot(axes->direction_no_z) < 0.0) {
        std::swap(pylon_start, pylon_end);
    }
    const double pylon_start_along = xyOnly(pylon_start).dot(axes->direction_no_z);
    const double pylon_end_along = xyOnly(pylon_end).dot(axes->direction_no_z);
    const double current_along = xyOnly(start_position).dot(axes->direction_no_z);
    const double permitted_start = pylon_start_along - pylon_span_margin_m;
    const double permitted_end = pylon_end_along + pylon_span_margin_m;
    result.distance_from_start_boundary_m = current_along - permitted_start;
    result.distance_to_end_boundary_m = permitted_end - current_along;
    result.between_pylons =
        result.distance_from_start_boundary_m >= -kPointEpsilonM &&
        result.distance_to_end_boundary_m >= -kPointEpsilonM;

    const double start_cross = xyOnly(start_position).dot(axes->cross_corridor_no_z);
    const double positive_outer_cross = xyOnly(*positive_outer).dot(axes->cross_corridor_no_z);
    const double negative_outer_cross = xyOnly(*negative_outer).dot(axes->cross_corridor_no_z);
    const double middle_cross = xyOnly(side_split->middle_point).dot(axes->cross_corridor_no_z);
    const bool positive_side = start_cross >= middle_cross;
    result.side = positive_side ? "positive" : "negative";
    result.measured_lateral_clearance_m = positive_side
        ? start_cross - positive_outer_cross
        : negative_outer_cross - start_cross;

    const double route_start_along = pylon_start_along +
        pylon_structure_extent_m + pylon_end_clearance_m;
    const double route_end_along = pylon_end_along -
        pylon_structure_extent_m - pylon_end_clearance_m;
    if (route_start_along >= route_end_along) {
        result.failure_reasons.push_back("pylon span is too short for the configured endpoint clearances");
        return result;
    }
    result.evaluable = true;
    if (!result.between_pylons) {
        result.failure_reasons.push_back("aircraft is outside the permitted longitudinal pylon span");
    }
    if (result.measured_lateral_clearance_m + kPointEpsilonM < inspection_clearance_m) {
        result.failure_reasons.push_back(
            "aircraft is inside the corridor or lacks the required outer-conductor clearance"
        );
    }

    const double ingress_along = std::clamp(current_along, route_start_along, route_end_along);
    const double ingress_cross = positive_side
        ? positive_outer_cross + inspection_clearance_m
        : negative_outer_cross - inspection_clearance_m;
    result.ingress_point =
        axes->direction_no_z * ingress_along + axes->cross_corridor_no_z * ingress_cross;
    result.ingress_point[2] = positive_side ? (*positive_highest)[2] : (*negative_highest)[2];
    result.ingress_point_valid = true;
    result.eligible = result.failure_reasons.empty();
    return result;
}

std::optional<CorridorInspectionRoute> iii_drone::behavior::BuildCorridorInspectionRoute(
    const std::vector<point_t> & powerline_points,
    const vector_t & powerline_direction,
    const point_t & pylon_a,
    const point_t & pylon_b,
    const point_t & start_position,
    double inspection_clearance_m,
    double pylon_end_clearance_m,
    double pylon_structure_extent_m,
    double under_cable_clearance_m,
    double inside_corridor_threshold_m,
    double pylon_span_margin_m,
    double max_pylon_direction_mismatch_rad,
    const std::optional<CorridorInspectionResume> & resume,
    double resume_position_tolerance_m
) {
    (void)under_cable_clearance_m;
    (void)inside_corridor_threshold_m;
    if (
        powerline_points.size() < 2 ||
        inspection_clearance_m <= 0.0 ||
        pylon_end_clearance_m <= 0.0 ||
        pylon_structure_extent_m < 0.0 ||
        under_cable_clearance_m < 0.0
    ) {
        return std::nullopt;
    }

    const auto powerline_axes = pl_geom::ComputeAxes(powerline_direction);
    const auto highest_conductor = pl_geom::SelectHighestPoint(powerline_points);
    if (!powerline_axes || !highest_conductor) {
        return std::nullopt;
    }

    if (!pl_geom::PylonSpanMatchesPowerlineDirection(
        pylon_a,
        pylon_b,
        powerline_axes->direction_no_z,
        max_pylon_direction_mismatch_rad
    )) {
        return std::nullopt;
    }

    vector_t corridor_direction = xyOnly(pylon_b - pylon_a);
    if (corridor_direction.dot(powerline_axes->direction_no_z) < 0.0) {
        corridor_direction *= -1.0;
    }
    const auto axes = pl_geom::ComputeAxes(corridor_direction);
    if (!axes) {
        return std::nullopt;
    }

    const auto side_split = splitInspectionSideConductors(
        powerline_points,
        axes->cross_corridor_no_z
    );
    if (!side_split) {
        return std::nullopt;
    }

    const auto positive_outer = pl_geom::FurthestPointXY(
        side_split->positive_points,
        side_split->middle_point
    );
    const auto negative_outer = pl_geom::FurthestPointXY(
        side_split->negative_points,
        side_split->middle_point
    );
    if (!positive_outer || !negative_outer) {
        return std::nullopt;
    }
    const auto positive_highest = pl_geom::SelectHighestPoint(side_split->positive_points);
    const auto negative_highest = pl_geom::SelectHighestPoint(side_split->negative_points);
    if (!positive_highest || !negative_highest) {
        return std::nullopt;
    }

    point_t pylon_start = pylon_a;
    point_t pylon_end = pylon_b;
    if (
        (xyOnly(pylon_end) - xyOnly(pylon_start)).dot(axes->direction_no_z) < 0.0
    ) {
        std::swap(pylon_start, pylon_end);
    }

    const double pylon_centerline_setback_m =
        pylon_structure_extent_m + pylon_end_clearance_m;
    const double start_along =
        xyOnly(pylon_start).dot(axes->direction_no_z) + pylon_centerline_setback_m;
    const double end_along =
        xyOnly(pylon_end).dot(axes->direction_no_z) - pylon_centerline_setback_m;
    if (start_along >= end_along) {
        return std::nullopt;
    }
    const double positive_cross =
        xyOnly(*positive_outer).dot(axes->cross_corridor_no_z) + inspection_clearance_m;
    const double negative_cross =
        xyOnly(*negative_outer).dot(axes->cross_corridor_no_z) - inspection_clearance_m;
    const double crossing_z = (*highest_conductor)[2] + inspection_clearance_m;
    const double positive_inspection_z = (*positive_highest)[2];
    const double negative_inspection_z = (*negative_highest)[2];

    const auto point_from_along_cross = [&](double along, double cross, double z) {
        point_t point =
            axes->direction_no_z * along + axes->cross_corridor_no_z * cross;
        point[2] = z;
        return point;
    };

    const point_t positive_start =
        point_from_along_cross(start_along, positive_cross, positive_inspection_z);
    const point_t positive_end =
        point_from_along_cross(end_along, positive_cross, positive_inspection_z);
    const point_t negative_start =
        point_from_along_cross(start_along, negative_cross, negative_inspection_z);
    const point_t negative_end =
        point_from_along_cross(end_along, negative_cross, negative_inspection_z);
    const point_t positive_start_high =
        point_from_along_cross(start_along, positive_cross, crossing_z);
    const point_t positive_end_high =
        point_from_along_cross(end_along, positive_cross, crossing_z);
    const point_t negative_start_high =
        point_from_along_cross(start_along, negative_cross, crossing_z);
    const point_t negative_end_high =
        point_from_along_cross(end_along, negative_cross, crossing_z);

    const std::array<RouteCandidate, 4> candidates{{
        {{positive_start, positive_start_high, negative_start_high, negative_start,
          negative_end, negative_end_high, positive_end_high, positive_end},
         "positive_start", true},
        {{positive_end, positive_end_high, negative_end_high, negative_end,
          negative_start, negative_start_high, positive_start_high, positive_start},
         "positive_end", true},
        {{negative_start, negative_start_high, positive_start_high, positive_start,
          positive_end, positive_end_high, negative_end_high, negative_end},
         "negative_start", false},
        {{negative_end, negative_end_high, positive_end_high, positive_end,
          positive_start, positive_start_high, negative_start_high, negative_start},
         "negative_end", false},
    }};

    if (
        resume &&
        resume_position_tolerance_m >= 0.0 &&
        (start_position - resume->interrupted_position).norm() <= resume_position_tolerance_m &&
        resume->active_waypoint_index >= resume->loop_start_index
    ) {
        const auto resumed_candidate = std::find_if(
            candidates.begin(),
            candidates.end(),
            [&](const RouteCandidate & candidate) {
                return candidate.name == resume->selected_route;
            }
        );
        const std::size_t active_loop_index =
            resume->active_waypoint_index - resume->loop_start_index;
        if (
            resumed_candidate != candidates.end() &&
            active_loop_index < resumed_candidate->waypoints.size()
        ) {
            CorridorInspectionRoute route;
            route.selected_route = resumed_candidate->name;
            route.resumed = true;
            route.loop_route_offset =
                (resume->loop_route_offset + active_loop_index) %
                resumed_candidate->waypoints.size();
            route.loop_start_index = 0;
            for (std::size_t index = 0; index < resumed_candidate->waypoints.size(); ++index) {
                route.waypoints.push_back(resumed_candidate->waypoints.at(
                    (route.loop_route_offset + index) % resumed_candidate->waypoints.size()
                ));
            }
            return route;
        }
    }

    const auto eligibility = EvaluateCorridorInspectionStart(
        powerline_points,
        powerline_direction,
        pylon_a,
        pylon_b,
        start_position,
        inspection_clearance_m,
        pylon_end_clearance_m,
        pylon_structure_extent_m,
        pylon_span_margin_m,
        max_pylon_direction_mismatch_rad
    );
    if (!eligibility.evaluable || !eligibility.eligible || !eligibility.ingress_point_valid) {
        return std::nullopt;
    }

    const bool positive_side = eligibility.side == "positive";
    const RouteCandidate * selected = nullptr;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (const auto & candidate : candidates) {
        if (candidate.starts_on_positive_side != positive_side) {
            continue;
        }
        const double distance =
            (xyOnly(candidate.waypoints.front()) - xyOnly(eligibility.ingress_point)).norm();
        if (distance < closest_distance) {
            selected = &candidate;
            closest_distance = distance;
        }
    }
    if (selected == nullptr) {
        return std::nullopt;
    }

    CorridorInspectionRoute route;
    route.selected_route = selected->name;
    appendIfMoved(route.waypoints, eligibility.ingress_point, start_position);

    const bool ingress_is_loop_start =
        !route.waypoints.empty() &&
        (route.waypoints.back() - selected->waypoints.front()).norm() < kPointEpsilonM;
    route.loop_start_index = ingress_is_loop_start
        ? route.waypoints.size() - 1
        : route.waypoints.size();
    route.waypoints.insert(
        route.waypoints.end(),
        ingress_is_loop_start ? std::next(selected->waypoints.begin()) : selected->waypoints.begin(),
        selected->waypoints.end()
    );
    return route;
}

/*****************************************************************************/
// Behavior-tree node
/*****************************************************************************/

PhaseWaypointProviderActionNode::PhaseWaypointProviderActionNode(
    const std::string & name,
    const NodeConfig & config,
    rclcpp::Node * node,
    Configuration::SharedPtr configuration
) : SyncActionNode(name, config),
    node_(node),
    configuration_(configuration) { }

PortsList PhaseWaypointProviderActionNode::providedPorts() {
    return {
        InputPort<iii_drone_interfaces::msg::Powerline>("stored_powerline"),
        InputPort<iii_drone_interfaces::msg::PylonOverview>("stored_pylon_overview"),
        InputPort<State>("start_state"),
        InputPort<bool>("resume_valid", false, "Use persisted inspection progress when valid."),
        InputPort<std::string>("resume_selected_route"),
        InputPort<int>("resume_waypoint_index"),
        InputPort<int>("resume_loop_start_index"),
        InputPort<int>("resume_route_offset"),
        InputPort<point_t>("resume_position"),
        OutputPort<SharedQueue<point_t>>("waypoints"),
        OutputPort<int>("loop_start_index"),
        OutputPort<int>("loop_route_offset"),
        OutputPort<float>("waypoint_target_yaw"),
        OutputPort<std::string>("selected_route")
    };
}

double PhaseWaypointProviderActionNode::parameterOr(
    const std::string & name,
    double fallback
) const {
    if (configuration_ && configuration_->HasParameter(name)) {
        return configuration_->GetParameter(name).as_double();
    }
    return fallback;
}

NodeStatus PhaseWaypointProviderActionNode::tick() {
    iii_drone_interfaces::msg::Powerline stored_powerline;
    if (!getInput("stored_powerline", stored_powerline)) {
        RCLCPP_WARN(node_->get_logger(), "PhaseWaypointProviderActionNode::tick(): stored_powerline missing");
        return NodeStatus::FAILURE;
    }

    iii_drone_interfaces::msg::PylonOverview stored_pylon_overview;
    if (!getInput("stored_pylon_overview", stored_pylon_overview)) {
        RCLCPP_WARN(node_->get_logger(), "PhaseWaypointProviderActionNode::tick(): stored_pylon_overview missing");
        return NodeStatus::FAILURE;
    }
    if (stored_pylon_overview.pylons.size() != 2) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): Expected exactly 2 pylons, got %lu",
            stored_pylon_overview.pylons.size()
        );
        return NodeStatus::FAILURE;
    }

    State start_state;
    if (!getInput("start_state", start_state)) {
        RCLCPP_WARN(node_->get_logger(), "PhaseWaypointProviderActionNode::tick(): start_state missing");
        return NodeStatus::FAILURE;
    }

    PowerlineAdapter powerline_adapter(stored_powerline);
    const std::vector<point_t> powerline_points = powerline_adapter.GetPoints();
    point_t pylon_a;
    pylon_a << stored_pylon_overview.pylons.at(0).x, stored_pylon_overview.pylons.at(0).y, 0.0;
    point_t pylon_b;
    pylon_b << stored_pylon_overview.pylons.at(1).x, stored_pylon_overview.pylons.at(1).y, 0.0;

    std::optional<CorridorInspectionResume> resume;
    bool resume_valid = false;
    getInput("resume_valid", resume_valid);
    if (resume_valid) {
        CorridorInspectionResume candidate;
        int active_waypoint_index = -1;
        int loop_start_index = -1;
        int loop_route_offset = -1;
        if (
            getInput("resume_selected_route", candidate.selected_route) &&
            getInput("resume_waypoint_index", active_waypoint_index) &&
            getInput("resume_loop_start_index", loop_start_index) &&
            getInput("resume_route_offset", loop_route_offset) &&
            getInput("resume_position", candidate.interrupted_position) &&
            active_waypoint_index >= 0 &&
            loop_start_index >= 0 &&
            loop_route_offset >= 0
        ) {
            candidate.active_waypoint_index = static_cast<std::size_t>(active_waypoint_index);
            candidate.loop_start_index = static_cast<std::size_t>(loop_start_index);
            candidate.loop_route_offset = static_cast<std::size_t>(loop_route_offset);
            resume = candidate;
        }
    }

    const double inspection_clearance_m = parameterOr(
        "/inspection_demo/inspection_clearance_m", 1.5
    );
    const double pylon_end_clearance_m = parameterOr(
        "/inspection_demo/pylon_end_clearance_m", 2.0
    );
    const double pylon_structure_extent_m = parameterOr(
        "/inspection_demo/pylon_structure_extent_m", 2.0
    );
    const double pylon_span_margin_m = parameterOr(
        "/inspection_demo/pylon_span_margin_m", 0.5
    );
    const double max_direction_mismatch_rad = parameterOr(
        "/inspection_demo/max_pylon_powerline_direction_mismatch_rad", 0.35
    );
    const auto eligibility = EvaluateCorridorInspectionStart(
        powerline_points,
        powerline_adapter.projection_plane().normal,
        pylon_a,
        pylon_b,
        start_state.position(),
        inspection_clearance_m,
        pylon_end_clearance_m,
        pylon_structure_extent_m,
        pylon_span_margin_m,
        max_direction_mismatch_rad
    );
    if (!resume && !eligibility.eligible) {
        for (const auto & reason : eligibility.failure_reasons) {
            RCLCPP_WARN(
                node_->get_logger(),
                "PhaseWaypointProviderActionNode: fresh inspection start rejected: %s",
                reason.c_str()
            );
        }
        return NodeStatus::FAILURE;
    }

    const auto route = BuildCorridorInspectionRoute(
        powerline_points,
        powerline_adapter.projection_plane().normal,
        pylon_a,
        pylon_b,
        start_state.position(),
        inspection_clearance_m,
        pylon_end_clearance_m,
        pylon_structure_extent_m,
        parameterOr("/behavior/under_cable_clearance_m", 1.0),
        parameterOr("/behavior/inside_powerline_xy_distance_threshold_m", 2.0),
        pylon_span_margin_m,
        max_direction_mismatch_rad,
        resume,
        0.75
    );
    if (!route) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): Failed to generate corridor inspection route"
        );
        return NodeStatus::FAILURE;
    }

    const vector_t pylon_span = pylon_b - pylon_a;
    const auto waypoint_target_yaw = pl_geom::ComputePowerlineAlignedYaw(
        pylon_span,
        start_state.yaw()
    );
    if (!waypoint_target_yaw) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): Failed to compute powerline-aligned yaw"
        );
        return NodeStatus::FAILURE;
    }

    auto waypoints = std::make_shared<std::deque<point_t>>(route->waypoints);
    setOutput("waypoints", waypoints);
    setOutput("loop_start_index", static_cast<int>(route->loop_start_index));
    setOutput("loop_route_offset", static_cast<int>(route->loop_route_offset));
    setOutput("waypoint_target_yaw", static_cast<float>(*waypoint_target_yaw));
    setOutput("selected_route", route->selected_route);

    RCLCPP_INFO(
        node_->get_logger(),
        "PhaseWaypointProviderActionNode::tick(): Generated %lu waypoint(s), loop_start=%lu, route=%s, resumed=%s, route_offset=%lu, target_yaw=%.3f",
        waypoints->size(),
        route->loop_start_index,
        route->selected_route.c_str(),
        route->resumed ? "true" : "false",
        route->loop_route_offset,
        *waypoint_target_yaw
    );
    for (size_t i = 0; i < waypoints->size(); ++i) {
        const auto & waypoint = waypoints->at(i);
        RCLCPP_INFO(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): waypoint[%lu]=[%.3f, %.3f, %.3f]",
            i,
            waypoint[0],
            waypoint[1],
            waypoint[2]
        );
    }

    return NodeStatus::SUCCESS;
}
