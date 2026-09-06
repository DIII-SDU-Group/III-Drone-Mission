/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/powerline_waypoint_provider_action_node.hpp>
#include <iii_drone_mission/behavior/powerline_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::configuration;
using namespace iii_drone::math;
using namespace BT;
namespace pl_geom = iii_drone::behavior::powerline_geometry;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineWaypointProviderActionNode::PowerlineWaypointProviderActionNode(
    const std::string & name, 
    const NodeConfiguration & conf,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    rclcpp::Node * node,
    Configuration::SharedPtr params
) : SyncActionNode(name, conf), tf_buffer_(tf_buffer), node_(node), configuration_(params) {
    combined_drone_awareness_sub_ = node_->create_subscription<iii_drone_interfaces::msg::CombinedDroneAwareness>(
        "/control/maneuver_controller/combined_drone_awareness",
        rclcpp::QoS(1),
        [this](const iii_drone_interfaces::msg::CombinedDroneAwareness::SharedPtr msg) {
            latest_ground_altitude_estimate_.store(msg->ground_altitude_estimate);
        }
    );
}

PortsList PowerlineWaypointProviderActionNode::providedPorts() {

    return {
        InputPort<iii_drone_interfaces::msg::Powerline>("stored_powerline"),
        InputPort<iii_drone_interfaces::msg::PylonOverview>("stored_pylon_overview"),
        InputPort<State>("start_state"),
        OutputPort<SharedQueue<point_t>>("waypoints_depart"),
        OutputPort<SharedQueue<point_t>>("waypoints_return"),
        OutputPort<float>("waypoint_target_yaw"),
        OutputPort<float>("cable_facing_yaw"),
        OutputPort<int>("powerline_overview_required_line_id")
    };

}

int PowerlineWaypointProviderActionNode::lineIdForPoint(
    const PowerlineAdapter & powerline_adapter,
    const point_t & point
) const {

    for (const auto & line : powerline_adapter.single_line_adapters()) {
        if (line.position().isApprox(point, 1e-4)) {
            return line.id();
        }
    }

    return -1;

}

double PowerlineWaypointProviderActionNode::minimumWaypointZ() const {
    const double minimum_target_altitude = configuration_->GetParameter(
        "/control/maneuver_controller/minimum_target_altitude"
    ).as_double();
    const double mission_waypoint_margin = 0.5;
    const double fallback_minimum_z = minimum_target_altitude + mission_waypoint_margin;
    const double ground_altitude_estimate = latest_ground_altitude_estimate_.load();
    if (!std::isfinite(ground_altitude_estimate)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::minimumWaypointZ(): No combined drone awareness ground estimate available; using legacy absolute minimum z %.3f",
            fallback_minimum_z
        );
        return fallback_minimum_z;
    }

    return ground_altitude_estimate + minimum_target_altitude + mission_waypoint_margin;
}

NodeStatus PowerlineWaypointProviderActionNode::tick() {

    // Get the stored powerline.
    iii_drone_interfaces::msg::Powerline stored_powerline;
    if (!getInput("stored_powerline", stored_powerline)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Stored powerline not provided"
        );
        return NodeStatus::FAILURE;
    }

    State start_state;
    if (!getInput("start_state", start_state)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Start state not provided"
        );
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Computing waypoints"
    );

    PowerlineAdapter powerline_adapter(stored_powerline);

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Transforming powerline to world frame"
    );

    const std::string world_frame_id = configuration_->GetParameter("/tf/world_frame_id").as_string();
    if (!powerline_adapter.Transform(world_frame_id, tf_buffer_)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not transform stored powerline to %s",
            world_frame_id.c_str()
        );
        return NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Getting powerline points."
    );

    std::vector<point_t> powerline_points = powerline_adapter.GetPoints();

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found %lu powerline points:",
        powerline_points.size()
    );

    for (auto & point : powerline_points) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "[%f, %f, %f]",
            point[0], point[1], point[2]
        );
    }

    // for (auto & line_adapter : powerline_adapter.single_line_adapters()) {
    //     geometry_msgs::msg::PointStamped point_stamped_msg;
    //     point_stamped_msg.header.frame_id = line_adapter.frame_id();

    //     point_stamped_msg.point = pointMsgFromPoint(line_adapter.position());

    //     point_stamped_msg = tf_buffer_->transform(point_stamped_msg, "world");

    //     powerline_points.push_back(pointFromPointMsg(point_stamped_msg.point));
    // }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Getting powerline direction."
    );

    const auto powerline_axes = pl_geom::ComputeAxes(powerline_adapter.projection_plane().normal);
    if (!powerline_axes) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not compute valid powerline axes"
        );
        return NodeStatus::FAILURE;
    }

    vector_t powerline_direction = powerline_axes->direction;

    pl_geom::PowerlineAxes corridor_axes = *powerline_axes;
    std::optional<point_t> pylon_a;
    std::optional<point_t> pylon_b;
    iii_drone_interfaces::msg::PylonOverview stored_pylon_overview;
    if (getInput("stored_pylon_overview", stored_pylon_overview) &&
        stored_pylon_overview.pylons.size() == 2) {
        point_t first_pylon;
        first_pylon << stored_pylon_overview.pylons.at(0).x,
            stored_pylon_overview.pylons.at(0).y, 0.0;
        point_t second_pylon;
        second_pylon << stored_pylon_overview.pylons.at(1).x,
            stored_pylon_overview.pylons.at(1).y, 0.0;

        const double max_direction_mismatch = configuration_->HasParameter(
            "/inspection_demo/max_pylon_powerline_direction_mismatch_rad"
        ) ? configuration_->GetParameter(
            "/inspection_demo/max_pylon_powerline_direction_mismatch_rad"
        ).as_double() : 0.35;
        const auto pylon_aligned_axes = pl_geom::ComputePylonAlignedAxes(
            powerline_direction,
            first_pylon,
            second_pylon
        );
        if (pylon_aligned_axes) {
            corridor_axes = *pylon_aligned_axes;
            pylon_a = first_pylon;
            pylon_b = second_pylon;
            if (!pl_geom::PylonSpanMatchesPowerlineDirection(
                    first_pylon,
                    second_pylon,
                    powerline_axes->direction_no_z,
                    max_direction_mismatch
                )) {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "PowerlineWaypointProviderActionNode::tick(): Stored mapper direction differs from pylon span by more than %.3f rad; using pylon corridor direction for approach",
                    max_direction_mismatch
                );
            }
            RCLCPP_INFO(
                node_->get_logger(),
                "PowerlineWaypointProviderActionNode::tick(): Using pylon span as corridor axis [%.3f, %.3f, %.3f] instead of local mapper direction [%.3f, %.3f, %.3f]",
                corridor_axes.direction[0],
                corridor_axes.direction[1],
                corridor_axes.direction[2],
                powerline_axes->direction[0],
                powerline_axes->direction[1],
                powerline_axes->direction[2]
            );
        } else {
            RCLCPP_WARN(
                node_->get_logger(),
                "PowerlineWaypointProviderActionNode::tick(): Pylon span is degenerate; retaining stored mapper direction for approach"
            );
        }
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Powerline direction:\n[%f, %f, %f]",
        powerline_direction[0], powerline_direction[1], powerline_direction[2]
    );

    // geometry_msgs::msg::Vector3Stamped powerline_direction_msg;
    // powerline_direction_msg.header.frame_id = powerline_adapter.single_line_adapters().front().frame_id();
    // powerline_direction_msg.vector = vectorMsgFromVector(powerline_direction);

    // powerline_direction_msg = tf_buffer_->transform(powerline_direction_msg, "world");

    // powerline_direction = vectorFromVectorMsg(powerline_direction_msg.vector);

    // Powerline normal is powerline direction rotated 90 degrees around the z-axis.
    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Getting powerline normal."
    );
    vector_t powerline_normal = corridor_axes.cross_corridor;

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Powerline normal:\n[%f, %f, %f]",
        powerline_normal[0], powerline_normal[1], powerline_normal[2]
    );

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finding middle line point."
    );
    point_t middle_line_point;
    // double lowest_point_normal_dot_product = std::numeric_limits<double>::max();

    // vector_t normal_no_z = powerline_normal;
    // normal_no_z[2] = 0;
    // normal_no_z /= normal_no_z.norm();

    // for (auto & point : powerline_points) {
    //     point_t p_no_z = point;
    //     p_no_z[2] = 0;
    //     double normal_dot_product = abs(p_no_z.dot(normal_no_z));

    //     if (normal_dot_product < lowest_point_normal_dot_product) {
    //         lowest_point_normal_dot_product = normal_dot_product;
    //         middle_line_point = point;
    //     }
    // }

    // // Verify that the found middle point is the one with highest z value:
    // double highest_z = std::numeric_limits<double>::min();
    // for (auto & point : powerline_points) {
    //     if (point[2] > highest_z) {
    //         highest_z = point[2];
    //     }
    // }

    // if (middle_line_point[2] != highest_z) {
    //     RCLCPP_WARN(
    //         node_->get_logger(),
    //         "PowerlineWaypointProviderActionNode::tick(): Middle line point is not the one with the highest z value"
    //     );
    //     return NodeStatus::FAILURE;
    // }

    if (powerline_points.size() < 3) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Need at least 3 powerline points to compute cable-approach waypoints, got %lu",
            powerline_points.size()
        );
        return NodeStatus::FAILURE;
    }

    const auto highest_point = pl_geom::SelectHighestPoint(powerline_points);
    if (!highest_point) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not find highest powerline point"
        );
        return NodeStatus::FAILURE;
    }

    double highest_z = (*highest_point)[2];
    middle_line_point = *highest_point;

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found middle line point:\n[%f, %f, %f]",
        middle_line_point[0], middle_line_point[1], middle_line_point[2]
    );

    vector_t powerline_normal_no_z = corridor_axes.cross_corridor_no_z;

    const auto classified_points = pl_geom::SplitByLargestLateralGap(powerline_points, powerline_normal_no_z);
    if (!classified_points) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not split powerline points by lateral gap"
        );
        return NodeStatus::FAILURE;
    }

    middle_line_point = classified_points->middle_point;
    std::vector<point_t> positive_direction_line_points = classified_points->positive_points;
    std::vector<point_t> negative_direction_line_points = classified_points->negative_points;

    RCLCPP_INFO(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Split powerline sides at largest lateral gap %.3f (negative=%lu positive=%lu)",
        classified_points->largest_gap,
        negative_direction_line_points.size(),
        positive_direction_line_points.size()
    );

    if (positive_direction_line_points.empty() || negative_direction_line_points.empty()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not split powerline points around a middle conductor (positive=%lu, negative=%lu)",
            positive_direction_line_points.size(),
            negative_direction_line_points.size()
        );
        return NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found %lu points on the positive direction:",
        positive_direction_line_points.size()
    );

    for (auto & point : positive_direction_line_points) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "[%f, %f, %f]",
            point[0], point[1], point[2]
        );
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found %lu points on the negative direction:",
        negative_direction_line_points.size()
    );

    for (auto & point : negative_direction_line_points) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "[%f, %f, %f]",
            point[0], point[1], point[2]
        );
    }

    // Find the side of the powerline where the point with the lowest z value is higher than for the other side.
    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finding the entry side of the powerline."
    );
    const auto positive_direction_lowest_point = pl_geom::LowestPointByZ(positive_direction_line_points);
    const auto negative_direction_lowest_point = pl_geom::LowestPointByZ(negative_direction_line_points);
    if (!positive_direction_lowest_point || !negative_direction_lowest_point) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not compute lowest conductor on each side"
        );
        return NodeStatus::FAILURE;
    }

    bool positive_direction_is_higher = pl_geom::PositiveSideIsEntrySide(*classified_points);

    if (positive_direction_is_higher) {
        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Positive direction is the entry side: lowest positive z=%.3f > lowest negative z=%.3f",
            (*positive_direction_lowest_point)[2],
            (*negative_direction_lowest_point)[2]
        );
    } else {
        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Negative direction is the entry side: lowest negative z=%.3f >= lowest positive z=%.3f",
            (*negative_direction_lowest_point)[2],
            (*positive_direction_lowest_point)[2]
        );
    }

    // Find the furthest (in xy) point from the middle point.
    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finding the furthest points in each direction."
    );
    point_t middle_line_point_xy = middle_line_point;
    middle_line_point_xy[2] = 0;

    const auto positive_direction_furthest_point_result =
        pl_geom::FurthestPointXY(positive_direction_line_points, middle_line_point);
    if (!positive_direction_furthest_point_result) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not find positive-side furthest point"
        );
        return NodeStatus::FAILURE;
    }
    point_t positive_direction_furthest_point = *positive_direction_furthest_point_result;

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found furthest point in positive direction:\n[%f, %f, %f]",
        positive_direction_furthest_point[0], positive_direction_furthest_point[1], positive_direction_furthest_point[2]
    );

    const auto negative_direction_furthest_point_result =
        pl_geom::FurthestPointXY(negative_direction_line_points, middle_line_point);
    if (!negative_direction_furthest_point_result) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not find negative-side furthest point"
        );
        return NodeStatus::FAILURE;
    }
    point_t negative_direction_furthest_point = *negative_direction_furthest_point_result;

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found furthest point in negative direction:\n[%f, %f, %f]",
        negative_direction_furthest_point[0], negative_direction_furthest_point[1], negative_direction_furthest_point[2]
    );

    // Find if the start state is further away along the powerline normal than the furthest point in each direction:
    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finding the start state location."
    );
    point_t start_state_position = start_state.position();
    const std::string drone_frame_id = configuration_->GetParameter("/tf/drone_frame_id").as_string();
    try {
        const auto world_T_drone_msg = tf_buffer_->lookupTransform(
            world_frame_id,
            drone_frame_id,
            tf2::TimePointZero
        );
        const point_t tf_start_state_position = vectorFromTransformMsg(world_T_drone_msg.transform);
        RCLCPP_DEBUG(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Using TF start position [%f, %f, %f] instead of odometry state position [%f, %f, %f]",
            tf_start_state_position[0],
            tf_start_state_position[1],
            tf_start_state_position[2],
            start_state_position[0],
            start_state_position[1],
            start_state_position[2]
        );
        start_state_position = tf_start_state_position;
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not lookup %s -> %s transform for start position, falling back to odometry state: %s",
            world_frame_id.c_str(),
            drone_frame_id.c_str(),
            ex.what()
        );
    }
    point_t start_state_pos_no_z = start_state_position;
    start_state_pos_no_z[2] = 0;

    const auto corridor_classification = pl_geom::ClassifyCorridor(
        start_state_position,
        middle_line_point,
        powerline_normal_no_z,
        positive_direction_furthest_point,
        negative_direction_furthest_point,
        configuration_->GetParameter("/behavior/inside_powerline_xy_distance_threshold_m").as_double()
    );

    double start_state_position_dot_prod_plane_normal = corridor_classification.start_cross_dot;
    double positive_direction_furthest_point_dot_prod_plane_normal = corridor_classification.positive_outer_cross_dot;
    double negative_direction_furthest_point_dot_prod_plane_normal = corridor_classification.negative_outer_cross_dot;

    bool start_state_is_on_positive_side = corridor_classification.on_positive_outer_side;
    bool start_state_is_on_negative_side = corridor_classification.on_negative_outer_side;

    if (start_state_is_on_positive_side) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Start state is on the positive side"
        );
    } else if (start_state_is_on_negative_side) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Start state is on the negative side"
        );
    } else {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Start state is in the middle of the powerline"
        );
    }

    // Create a queue of waypoints.
    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Creating waypoints."
    );
    auto shared_queue = std::make_shared<std::deque<point_t>>();
    auto make_depart_queue = [&shared_queue]() {
        auto depart_queue = std::make_shared<std::deque<point_t>>();
        if (shared_queue->size() > 1) {
            depart_queue->insert(
                depart_queue->end(),
                std::next(shared_queue->begin()),
                shared_queue->end()
            );
        }
        return depart_queue;
    };
    auto log_waypoints = [&](const char * label, const SharedQueue<point_t> & queue) {
        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): %s waypoint count=%lu",
            label,
            queue ? queue->size() : 0
        );
        if (!queue) {
            return;
        }
        for (size_t i = 0; i < queue->size(); ++i) {
            const auto & point = queue->at(i);
            RCLCPP_INFO(
                node_->get_logger(),
                "PowerlineWaypointProviderActionNode::tick(): %s[%lu]=[%.3f, %.3f, %.3f]",
                label,
                i,
                point[0],
                point[1],
                point[2]
            );
        }
    };
    const double minimum_target_altitude = configuration_->GetParameter(
        "/control/maneuver_controller/minimum_target_altitude"
    ).as_double();
    const double waypoint_minimum_z = minimumWaypointZ();
    auto enforce_minimum_waypoint_altitude = [&](const char * label, const SharedQueue<point_t> & queue) {
        if (!queue) {
            return;
        }
        for (size_t i = 0; i < queue->size(); ++i) {
            auto & waypoint = queue->at(i);
            if (waypoint[2] < waypoint_minimum_z) {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "PowerlineWaypointProviderActionNode::tick(): Raising %s[%lu] altitude from %.3f to %.3f to satisfy fly-to-position minimum target altitude %.3f above current ground estimate with mission waypoint margin",
                    label,
                    i,
                    waypoint[2],
                    waypoint_minimum_z,
                    minimum_target_altitude
                );
                waypoint[2] = waypoint_minimum_z;
            }
        }
    };

    // Keep the start position for return planning, but do not command a no-op
    // departure waypoint to the current position.
    shared_queue->push_back(start_state_position);

    // Create waypoints
    // Get the xy distance from the middle line point to the start position along the powerline direction:
    vector_t powerline_direction_no_z = corridor_axes.direction_no_z;
    auto align_waypoint_to_start_span = [&](point_t waypoint) {
        point_t waypoint_xy = waypoint;
        waypoint_xy[2] = 0;
        const double span_delta = (start_state_pos_no_z - waypoint_xy).dot(powerline_direction_no_z);
        waypoint[0] += span_delta * powerline_direction_no_z[0];
        waypoint[1] += span_delta * powerline_direction_no_z[1];
        return waypoint;
    };
    // double start_state_position_dot_prod_plane_direction = start_state_pos_no_z.dot(powerline_direction_no_z);
    // double middle_line_point_dot_prod_plane_direction = middle_line_point_xy.dot(powerline_direction_no_z);
    // Corridor membership is lateral to the powerline, not distance along the
    // conductor span. A drone can be far along the span and still be inside the
    // corridor for under-cable approach planning.
    double distance_to_middle_line_point = corridor_classification.lateral_distance_to_middle;

    RCLCPP_INFO(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Corridor classification: lateral_distance=%.3f threshold=%.3f start_dot=%.3f positive_limit=%.3f negative_limit=%.3f start_positive=%s start_negative=%s",
        distance_to_middle_line_point,
        configuration_->GetParameter("/behavior/inside_powerline_xy_distance_threshold_m").as_double(),
        start_state_position_dot_prod_plane_normal,
        positive_direction_furthest_point_dot_prod_plane_normal,
        negative_direction_furthest_point_dot_prod_plane_normal,
        start_state_is_on_positive_side ? "true" : "false",
        start_state_is_on_negative_side ? "true" : "false"
    );

    const bool start_state_is_inside_corridor = corridor_classification.inside_corridor;
    const bool start_state_is_below_top_conductor = start_state_position[2] < highest_z;
    point_t under_cable_target_line_point = positive_direction_is_higher ? positive_direction_furthest_point : negative_direction_furthest_point;
    const auto waypoint_target_yaw_result = pl_geom::ComputeCableFacingYaw(
        powerline_normal_no_z,
        positive_direction_is_higher
    );
    if (!waypoint_target_yaw_result) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Could not compute a cable-facing yaw from the cross-corridor axis"
        );
        return NodeStatus::FAILURE;
    }
    const double waypoint_target_yaw = *waypoint_target_yaw_result;
    bool force_above_corridor_route = false;

    if (pylon_a && pylon_b) {
        const double pylon_a_along = pylon_a->dot(powerline_direction_no_z);
        const double pylon_b_along = pylon_b->dot(powerline_direction_no_z);
        const double start_along = start_state_pos_no_z.dot(powerline_direction_no_z);
        const double min_pylon_along = std::min(pylon_a_along, pylon_b_along);
        const double max_pylon_along = std::max(pylon_a_along, pylon_b_along);
        const double pylon_span_margin = configuration_->HasParameter("/inspection_demo/pylon_span_margin_m")
            ? configuration_->GetParameter("/inspection_demo/pylon_span_margin_m").as_double()
            : 0.0;
        const bool start_beyond_pylon_span =
            start_along < (min_pylon_along - pylon_span_margin) ||
            start_along > (max_pylon_along + pylon_span_margin);

        if (start_state_is_inside_corridor && start_beyond_pylon_span) {
            force_above_corridor_route = true;
            RCLCPP_INFO(
                node_->get_logger(),
                "PowerlineWaypointProviderActionNode::tick(): Pylon-aware route: start is inside corridor but beyond pylon span (start_along=%.3f span=[%.3f, %.3f] margin=%.3f); forcing top-clearance escape.",
                start_along,
                min_pylon_along,
                max_pylon_along,
                pylon_span_margin
            );
        }
    }

    auto outer_cable_under_waypoint = [&]() {
        point_t waypoint = under_cable_target_line_point;
        waypoint[2] -= configuration_->GetParameter("/behavior/under_cable_clearance_m").as_double();
        return waypoint;
    };

    int powerline_overview_required_line_id = lineIdForPoint(powerline_adapter, under_cable_target_line_point);
    if (powerline_overview_required_line_id < 0) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Failed to resolve under-cable target line id"
        );
        return NodeStatus::FAILURE;
    }
    // Preserve the vehicle heading during transit. A large yaw step on the
    // first blended waypoint can prevent the maneuver handoff from producing
    // references. Rotate toward the cable only at the final sensing position.
    setOutput("waypoint_target_yaw", static_cast<float>(start_state.yaw()));
    setOutput("cable_facing_yaw", static_cast<float>(waypoint_target_yaw));
    setOutput("powerline_overview_required_line_id", powerline_overview_required_line_id);
    RCLCPP_INFO(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Under-cable target overview line id: %d point=[%.3f, %.3f, %.3f] waypoint_yaw=%.3f",
        powerline_overview_required_line_id,
        under_cable_target_line_point[0],
        under_cable_target_line_point[1],
        under_cable_target_line_point[2],
        waypoint_target_yaw
    );
    auto append_under_outer_cable_waypoints = [&]() {
        point_t waypoint = outer_cable_under_waypoint();

        vector_t waypoint_displacement = configuration_->GetParameter("/behavior/horizontal_clearance_m").as_double() * powerline_normal / powerline_normal.norm();
        waypoint_displacement[2] = 0;
        waypoint_displacement *= positive_direction_is_higher ? 1 : -1;
        waypoint += waypoint_displacement;

        shared_queue->push_back(waypoint);

        point_t final_waypoint = outer_cable_under_waypoint();
        final_waypoint[2] = waypoint[2];

        shared_queue->push_back(final_waypoint);
    };

    if (force_above_corridor_route) {
        point_t vertical_escape = start_state_position;
        vertical_escape[2] = highest_z + configuration_->GetParameter("/behavior/top_clearance_m").as_double();
        shared_queue->push_back(vertical_escape);
    }

    if (start_state_is_inside_corridor && start_state_is_below_top_conductor && !force_above_corridor_route) {
        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Using direct inside-corridor under-cable approach; lateral_distance=%.3f threshold=%.3f start_z=%.3f highest_z=%.3f start_positive=%s start_negative=%s",
            distance_to_middle_line_point,
            configuration_->GetParameter("/behavior/inside_powerline_xy_distance_threshold_m").as_double(),
            start_state_position[2],
            highest_z,
            start_state_is_on_positive_side ? "true" : "false",
            start_state_is_on_negative_side ? "true" : "false"
        );
        const double minimum_direct_waypoint_z = minimumWaypointZ();
        point_t entry_under_waypoint = align_waypoint_to_start_span(outer_cable_under_waypoint());
        if (entry_under_waypoint[2] < minimum_direct_waypoint_z) {
            RCLCPP_INFO(
                node_->get_logger(),
                "PowerlineWaypointProviderActionNode::tick(): Raising inside-corridor entry-under-cable waypoint altitude from %.3f to %.3f to satisfy fly-to-position minimum target altitude %.3f with margin while keeping target line id %d",
                entry_under_waypoint[2],
                minimum_direct_waypoint_z,
                minimum_target_altitude,
                powerline_overview_required_line_id
            );
            entry_under_waypoint[2] = minimum_direct_waypoint_z;
        }
        point_t middle_under_waypoint = align_waypoint_to_start_span(middle_line_point);
        middle_under_waypoint[2] = entry_under_waypoint[2];

        setOutput("powerline_overview_required_line_id", powerline_overview_required_line_id);
        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Inside-corridor waypoint plan keeps entry-side overview line id: %d and preserves start span station; middle_waypoint=[%.3f, %.3f, %.3f] entry_under_waypoint=[%.3f, %.3f, %.3f]",
            powerline_overview_required_line_id,
            middle_under_waypoint[0],
            middle_under_waypoint[1],
            middle_under_waypoint[2],
            entry_under_waypoint[0],
            entry_under_waypoint[1],
            entry_under_waypoint[2]
        );

        shared_queue->push_back(middle_under_waypoint);
        shared_queue->push_back(entry_under_waypoint);

        auto shared_queue_depart = make_depart_queue();
        enforce_minimum_waypoint_altitude("depart", shared_queue_depart);
        setOutput("waypoints_depart", shared_queue_depart);

        // Reverse the waypoints for the return path as new shared queue, without modifying the original shared queue.
        std::shared_ptr<std::deque<point_t>> shared_queue_return = std::make_shared<std::deque<point_t>>(shared_queue->rbegin(), shared_queue->rend());
        enforce_minimum_waypoint_altitude("return", shared_queue_return);

        setOutput("waypoints_return", shared_queue_return);
        log_waypoints("depart", shared_queue_depart);
        log_waypoints("return", shared_queue_return);

        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Finished computing waypoints"
        );

        return NodeStatus::SUCCESS;

    } else if (start_state_is_inside_corridor) {
        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Start is laterally inside corridor but above top conductor; using top/side route instead of direct descent; lateral_distance=%.3f threshold=%.3f start_z=%.3f highest_z=%.3f",
            distance_to_middle_line_point,
            configuration_->GetParameter("/behavior/inside_powerline_xy_distance_threshold_m").as_double(),
            start_state_position[2],
            highest_z
        );
    }

    point_t negative_top_waypoint = negative_direction_furthest_point;
    negative_top_waypoint[2] = highest_z;
    negative_top_waypoint[2] += configuration_->GetParameter("/behavior/top_clearance_m").as_double();
    vector_t waypoint_displacement = configuration_->GetParameter("/behavior/horizontal_clearance_m").as_double() * powerline_normal / powerline_normal.norm();
    waypoint_displacement[2] = 0;
    waypoint_displacement *= -1;
    negative_top_waypoint += waypoint_displacement;

    // RLCPP_INFO(
    //     node_->get_logger(),
    //     "PowerlineWaypointProviderActionNode::tick(): negative_top_waypoint: [%f,%f,%f]",
    //     negative_top_waypoint[0],
    //     negative_top_waypoint[1],
    //     negative_top_waypoint[2]
    // );

    point_t positive_top_waypoint = positive_direction_furthest_point;
    positive_top_waypoint[2] = highest_z;
    positive_top_waypoint[2] += configuration_->GetParameter("/behavior/top_clearance_m").as_double();
    waypoint_displacement *= -1;
    positive_top_waypoint += waypoint_displacement;

    // RLCPP_INFO(
    //     node_->get_logger(),
    //     "PowerlineWaypointProviderActionNode::tick(): negative_top_waypoint: [%f,%f,%f]",
    //     negative_top_waypoint[0],
    //     negative_top_waypoint[1],
    //     negative_top_waypoint[2]
    // );

    if (positive_direction_is_higher) {
        if (start_state_is_on_negative_side) {
            shared_queue->push_back(negative_top_waypoint);
            shared_queue->push_back(positive_top_waypoint);
        } else if (!start_state_is_on_positive_side) {
            // Drone is in the middle of the powerline.
            // First waypoint is the current state xy, z same as negative_top_waypoint.
            point_t waypoint = start_state_position;
            waypoint[2] = negative_top_waypoint[2];
            shared_queue->push_back(waypoint);

            shared_queue->push_back(positive_top_waypoint);
        }
    } else {
        if (start_state_is_on_positive_side) {
            shared_queue->push_back(positive_top_waypoint);
            shared_queue->push_back(negative_top_waypoint);
        } else if (!start_state_is_on_negative_side) {
            // Drone is in the middle of the powerline.
            // First waypoint is the current state xy, z same as positive_top_waypoint.
            point_t waypoint = start_state_position;
            waypoint[2] = positive_top_waypoint[2];
            shared_queue->push_back(waypoint);

            shared_queue->push_back(negative_top_waypoint);
        }
    }

    append_under_outer_cable_waypoints();

    auto shared_queue_depart = make_depart_queue();
    enforce_minimum_waypoint_altitude("depart", shared_queue_depart);
    setOutput("waypoints_depart", shared_queue_depart);

    // Reverse the waypoints for the return path as new shared queue, without modifying the original shared queue.
    std::shared_ptr<std::deque<point_t>> shared_queue_return = std::make_shared<std::deque<point_t>>(shared_queue->rbegin(), shared_queue->rend());
    enforce_minimum_waypoint_altitude("return", shared_queue_return);

    setOutput("waypoints_return", shared_queue_return);

    RCLCPP_INFO(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finished computing waypoints:"
    );
    log_waypoints("depart", shared_queue_depart);
    log_waypoints("return", shared_queue_return);

    return NodeStatus::SUCCESS;

}

// SharedQueue<point_t> PowerlineWaypointProviderActionNode::applyLinearInterpolation(
//     SharedQueue<point_t> points
// ) {

//     float interpolation_segment_distance = configuration_->GetParameter("/behavior/interpolation_segment_distance_m").as_double();

//     SharedQueue<point_t> interpolated_points = std::make_shared<std::deque<point_t>>();

//     for (size_t i = 0; i < points->size() - 1; i++) {
//         point_t start_point = points->at(i);
//         point_t end_point = points->at(i + 1);

//         vector_t displacement = end_point - start_point;
//         // Inside the loop where you calculate the displacement and distance

//         float distance = displacement.norm();

//         // Calculate the initial number of segments
//         float initial_segments = distance / interpolation_segment_distance;

//         // Round to the nearest whole number to get an integer number of segments
//         int segments = std::round(initial_segments);

//         // If segments is 0, it means the distance is smaller than the interpolation segment distance.
//         // In this case, set segments to 1 to ensure at least one segment.
//         if (segments == 0) segments = 1;

//         // Recalculate the segment distance so that it divides the total distance exactly
//         float adjusted_segment_distance = distance / segments;

//         // Now use adjusted_segment_distance for interpolation
//         for (int j = 0; j < segments; j++) {
//             point_t interpolated_point = start_point + (j * adjusted_segment_distance / distance) * displacement;
//             interpolated_points->push_back(interpolated_point);
//         }

//         // Make sure to add the last point to the interpolated points
//         interpolated_points->push_back(end_point);
//     }

//     return interpolated_points;

// }
