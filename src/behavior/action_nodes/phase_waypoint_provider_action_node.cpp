/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/phase_waypoint_provider_action_node.hpp>
#include <iii_drone_mission/behavior/powerline_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <iterator>
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

    point_t xyOnly(point_t point) {
        point[2] = 0.0;
        return point;
    }

    double signOrPositive(double value) {
        return value < 0.0 ? -1.0 : 1.0;
    }

    void appendIfDistinct(std::deque<point_t> & queue, const point_t & point) {
        if (!queue.empty() && (queue.back() - point).norm() < kPointEpsilonM) {
            return;
        }
        queue.push_back(point);
    }

    point_t pylonPoint(const iii_drone_interfaces::msg::Pylon & pylon, double z) {
        point_t point;
        point[0] = pylon.x;
        point[1] = pylon.y;
        point[2] = z;
        return point;
    }

} // namespace

/*****************************************************************************/
// Implementation
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
        OutputPort<SharedQueue<point_t>>("waypoints"),
        OutputPort<float>("waypoint_target_yaw"),
        OutputPort<std::string>("selected_phase")
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
    std::vector<point_t> powerline_points = powerline_adapter.GetPoints();
    if (powerline_points.size() != 4) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): Inspection demo requires exactly 4 conductors, got %lu",
            powerline_points.size()
        );
        return NodeStatus::FAILURE;
    }

    const auto axes = pl_geom::ComputeAxes(powerline_adapter.projection_plane().normal);
    const auto top_conductor = pl_geom::SelectHighestPoint(powerline_points);
    if (!axes || !top_conductor) {
        RCLCPP_WARN(node_->get_logger(), "PhaseWaypointProviderActionNode::tick(): Invalid powerline geometry");
        return NodeStatus::FAILURE;
    }

    const point_t top_point = *top_conductor;
    const point_t top_xy = xyOnly(top_point);

    std::vector<InspectionConductor> positive_side;
    std::vector<InspectionConductor> negative_side;
    for (const auto & point : powerline_points) {
        if ((point - top_point).norm() < kPointEpsilonM) {
            continue;
        }

        const double cross = (xyOnly(point) - top_xy).dot(axes->cross_corridor_no_z);
        InspectionConductor conductor{
            .point = point,
            .cross_sign = signOrPositive(cross),
            .top = false
        };
        if (cross >= 0.0) {
            positive_side.push_back(conductor);
        } else {
            negative_side.push_back(conductor);
        }
    }

    if (positive_side.empty() || negative_side.empty()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): Could not split remaining conductors around top conductor"
        );
        return NodeStatus::FAILURE;
    }

    auto z_descending = [](const InspectionConductor & lhs, const InspectionConductor & rhs) {
        return lhs.point[2] > rhs.point[2];
    };
    std::sort(positive_side.begin(), positive_side.end(), z_descending);
    std::sort(negative_side.begin(), negative_side.end(), z_descending);

    const bool positive_is_one_side = positive_side.size() == 1;
    const auto & one_side = positive_is_one_side ? positive_side : negative_side;
    const auto & two_side = positive_is_one_side ? negative_side : positive_side;

    if (one_side.empty() || two_side.size() < 2) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): Expected one conductor on one side and at least two on the other side (positive=%lu negative=%lu)",
            positive_side.size(),
            negative_side.size()
        );
        return NodeStatus::FAILURE;
    }

    std::vector<InspectionConductor> ordered_conductors;
    ordered_conductors.push_back(one_side.front());
    ordered_conductors.push_back(InspectionConductor{.point = top_point, .cross_sign = 0.0, .top = true});
    ordered_conductors.push_back(two_side.at(0));
    ordered_conductors.push_back(two_side.at(1));

    point_t pylon_a = pylonPoint(stored_pylon_overview.pylons.at(0), 0.0);
    point_t pylon_b = pylonPoint(stored_pylon_overview.pylons.at(1), 0.0);
    vector_t span_direction = xyOnly(pylon_b) - xyOnly(pylon_a);
    const double span_norm = span_direction.norm();
    if (span_norm < 1e-6) {
        RCLCPP_WARN(node_->get_logger(), "PhaseWaypointProviderActionNode::tick(): Pylon span is degenerate");
        return NodeStatus::FAILURE;
    }
    span_direction /= span_norm;
    const double max_pylon_direction_mismatch =
        parameterOr("/inspection_demo/max_pylon_powerline_direction_mismatch_rad", 0.35);
    if (!pl_geom::PylonSpanMatchesPowerlineDirection(pylon_a, pylon_b, axes->direction_no_z, max_pylon_direction_mismatch)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PhaseWaypointProviderActionNode::tick(): Pylon span direction does not match powerline direction within %.3f rad",
            max_pylon_direction_mismatch
        );
        return NodeStatus::FAILURE;
    }
    if (span_direction.dot(axes->direction_no_z) < 0.0) {
        span_direction *= -1.0;
    }

    point_t pylon_start = pylon_a;
    point_t pylon_end = pylon_b;
    if ((xyOnly(pylon_end) - xyOnly(pylon_start)).dot(span_direction) < 0.0) {
        std::swap(pylon_start, pylon_end);
    }

    const double inspection_clearance = parameterOr("/inspection_demo/inspection_clearance_m", 1.5);
    const double pylon_low_height = parameterOr("/inspection_demo/pylon_low_height_above_ground_m", 1.5);
    const double ground_z = parameterOr("/simulation/ground_plane_z", 0.0);
    const double top_clearance_z = top_point[2] + inspection_clearance;
    const double top_cross = top_xy.dot(axes->cross_corridor_no_z);

    auto point_from_along_cross = [&](double along, double cross, double z) {
        point_t point = axes->direction_no_z * along + axes->cross_corridor_no_z * cross;
        point[2] = z;
        return point;
    };

    auto pylon_along = [&](const point_t & pylon) {
        return xyOnly(pylon).dot(axes->direction_no_z);
    };

    auto inspection_point = [&](const InspectionConductor & conductor, const point_t & pylon) {
        const double along = pylon_along(pylon);
        const double conductor_cross = xyOnly(conductor.point).dot(axes->cross_corridor_no_z);
        if (conductor.top) {
            return point_from_along_cross(along, top_cross, top_clearance_z);
        }

        const double cross = conductor_cross + conductor.cross_sign * inspection_clearance;
        return point_from_along_cross(along, cross, conductor.point[2]);
    };

    auto pylon_scan_points = [&](const point_t & pylon, double beyond_sign) {
        std::vector<point_t> points;
        const double along = pylon_along(pylon) + beyond_sign * inspection_clearance;
        const double cross = xyOnly(pylon).dot(axes->cross_corridor_no_z);
        points.push_back(point_from_along_cross(along, cross, top_clearance_z));
        points.push_back(point_from_along_cross(along, cross, ground_z + pylon_low_height));
        points.push_back(point_from_along_cross(along, cross, top_clearance_z));
        return points;
    };

    auto build_route_from_phase = [&](size_t start_phase) {
        std::deque<point_t> route;
        for (size_t offset = 0; offset < ordered_conductors.size(); ++offset) {
            const size_t phase = (start_phase + offset) % ordered_conductors.size();
            const bool forward = (phase % 2) == 0;
            const point_t & phase_start_pylon = forward ? pylon_start : pylon_end;
            const point_t & phase_end_pylon = forward ? pylon_end : pylon_start;
            const double beyond_sign = forward ? 1.0 : -1.0;

            const auto conductor_start = inspection_point(ordered_conductors.at(phase), phase_start_pylon);
            const auto conductor_end = inspection_point(ordered_conductors.at(phase), phase_end_pylon);

            if (!route.empty()) {
                point_t top_escape = route.back();
                top_escape[2] = top_clearance_z;
                appendIfDistinct(route, top_escape);
                point_t next_top = conductor_start;
                next_top[2] = top_clearance_z;
                appendIfDistinct(route, next_top);
            }

            appendIfDistinct(route, conductor_start);
            appendIfDistinct(route, conductor_end);

            point_t top_at_end = conductor_end;
            top_at_end[2] = top_clearance_z;
            appendIfDistinct(route, top_at_end);
            for (const auto & pylon_point : pylon_scan_points(phase_end_pylon, beyond_sign)) {
                appendIfDistinct(route, pylon_point);
            }
        }
        return route;
    };

    const point_t start_position = start_state.position();
    size_t selected_phase = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (size_t phase = 0; phase < ordered_conductors.size(); ++phase) {
        const auto route = build_route_from_phase(phase);
        if (route.empty()) {
            continue;
        }
        const double distance = (route.front() - start_position).norm();
        if (distance < best_distance) {
            best_distance = distance;
            selected_phase = phase;
        }
    }

    auto selected_route = build_route_from_phase(selected_phase);
    if (selected_route.empty()) {
        RCLCPP_WARN(node_->get_logger(), "PhaseWaypointProviderActionNode::tick(): Failed to generate selected inspection route");
        return NodeStatus::FAILURE;
    }

    std::deque<point_t> full_route;
    appendIfDistinct(full_route, start_position);

    const auto side_split = pl_geom::SplitByLargestLateralGap(powerline_points, axes->cross_corridor_no_z);
    const auto positive_outer = side_split ? pl_geom::FurthestPointXY(side_split->positive_points, side_split->middle_point) : std::nullopt;
    const auto negative_outer = side_split ? pl_geom::FurthestPointXY(side_split->negative_points, side_split->middle_point) : std::nullopt;
    const bool can_classify_corridor = side_split && positive_outer && negative_outer;
    bool inside_corridor_between_pylons = false;
    if (can_classify_corridor) {
        const double inside_threshold = parameterOr("/behavior/inside_powerline_xy_distance_threshold_m", 2.0);
        const auto corridor = pl_geom::ClassifyCorridor(
            start_position,
            side_split->middle_point,
            axes->cross_corridor_no_z,
            *positive_outer,
            *negative_outer,
            inside_threshold
        );
        const double start_along = xyOnly(start_position).dot(axes->direction_no_z);
        const double pylon_span_margin = parameterOr("/inspection_demo/pylon_span_margin_m", 0.5);
        const double min_along = std::min(pylon_along(pylon_start), pylon_along(pylon_end)) - pylon_span_margin;
        const double max_along = std::max(pylon_along(pylon_start), pylon_along(pylon_end)) + pylon_span_margin;
        inside_corridor_between_pylons = corridor.inside_corridor && start_along >= min_along && start_along <= max_along;
    }

    if (inside_corridor_between_pylons && side_split) {
        const double under_clearance = parameterOr("/behavior/under_cable_clearance_m", 1.0);
        const InspectionConductor entry_conductor = ordered_conductors.front();
        const double current_along = xyOnly(start_position).dot(axes->direction_no_z);
        const double entry_cross = xyOnly(entry_conductor.point).dot(axes->cross_corridor_no_z);
        const double under_z = entry_conductor.point[2] - under_clearance;
        appendIfDistinct(full_route, point_from_along_cross(current_along, top_cross, under_z));
        appendIfDistinct(full_route, point_from_along_cross(current_along, entry_cross + entry_conductor.cross_sign * inspection_clearance, under_z));
    }

    point_t vertical_top = full_route.back();
    vertical_top[2] = top_clearance_z;
    appendIfDistinct(full_route, vertical_top);

    point_t first_top = selected_route.front();
    first_top[2] = top_clearance_z;
    appendIfDistinct(full_route, first_top);

    for (const auto & waypoint : selected_route) {
        appendIfDistinct(full_route, waypoint);
    }

    auto waypoints = std::make_shared<std::deque<point_t>>();
    if (full_route.size() > 1) {
        waypoints->insert(waypoints->end(), std::next(full_route.begin()), full_route.end());
    }

    setOutput("waypoints", waypoints);
    setOutput("waypoint_target_yaw", static_cast<float>(start_state.yaw()));
    setOutput("selected_phase", "phase_" + std::to_string(selected_phase));

    RCLCPP_INFO(
        node_->get_logger(),
        "PhaseWaypointProviderActionNode::tick(): Generated %lu inspection waypoint(s), selected_phase=%lu, closest_distance=%.3f",
        waypoints->size(),
        selected_phase,
        best_distance
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
