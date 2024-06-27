/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/powerline_waypoint_provider_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace iii_drone::configuration;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineWaypointProviderActionNode::PowerlineWaypointProviderActionNode(
    const std::string & name, 
    const NodeConfiguration & conf,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    rclcpp::Node * node,
    ParameterBundle::SharedPtr params
) : SyncActionNode(name, conf), tf_buffer_(tf_buffer), node_(node), parameters_(params) { }

PortsList PowerlineWaypointProviderActionNode::providedPorts() {

    return {
        InputPort<iii_drone_interfaces::msg::Powerline>("stored_powerline"),
        InputPort<State>("start_state"),
        OutputPort<SharedQueue<point_t>>("waypoints_depart"),
        OutputPort<SharedQueue<point_t>>("waypoints_return")
    };

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

    powerline_adapter.Transform(parameters_->GetParameter("world_frame_id").as_string(), tf_buffer_);

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

    vector_t powerline_direction = powerline_adapter.projection_plane().normal / powerline_adapter.projection_plane().normal.norm();

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
    vector_t powerline_normal = powerline_direction;
    powerline_normal[0] = -powerline_direction[1];
    powerline_normal[1] = powerline_direction[0];

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
    double lowest_point_normal_dot_product = std::numeric_limits<double>::max();

    vector_t normal_no_z = powerline_normal;
    normal_no_z[2] = 0;
    normal_no_z /= normal_no_z.norm();

    for (auto & point : powerline_points) {
        point_t p_no_z = point;
        p_no_z[2] = 0;
        double normal_dot_product = abs(p_no_z.dot(normal_no_z));

        if (normal_dot_product < lowest_point_normal_dot_product) {
            lowest_point_normal_dot_product = normal_dot_product;
            middle_line_point = point;
        }
    }

    // Verify that the found middle point is the one with highest z value:
    double highest_z = std::numeric_limits<double>::min();
    for (auto & point : powerline_points) {
        if (point[2] > highest_z) {
            highest_z = point[2];
        }
    }

    if (middle_line_point[2] != highest_z) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Middle line point is not the one with the highest z value"
        );
        return NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found middle line point:\n[%f, %f, %f]",
        middle_line_point[0], middle_line_point[1], middle_line_point[2]
    );

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finding points on positive and negative direction."
    );

    std::vector<point_t> positive_direction_line_points;
    std::vector<point_t> negative_direction_line_points;

    for (auto & point : powerline_points) {
        // Check if the point is the middle point, continue
        if (point == middle_line_point) {
            continue;
        }
        if (point.dot(powerline_normal) > 0) {
            positive_direction_line_points.push_back(point);
        } else {
            negative_direction_line_points.push_back(point);
        }
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
    point_t positive_direction_lowest_point = positive_direction_line_points.front();
    for (auto & point : positive_direction_line_points) {
        if (point[2] < positive_direction_lowest_point[2]) {
            positive_direction_lowest_point = point;
        }
    }

    point_t negative_direction_lowest_point = negative_direction_line_points.front();
    for (auto & point : negative_direction_line_points) {
        if (point[2] < negative_direction_lowest_point[2]) {
            negative_direction_lowest_point = point;
        }
    }

    bool positive_direction_is_higher = positive_direction_lowest_point[2] > negative_direction_lowest_point[2];

    if (positive_direction_is_higher) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Positive direction is the entry side"
        );
    } else {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Negative direction is the entry side"
        );
    }

    // Find the furthest (in xy) point from the middle point.
    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finding the furthest points in each direction."
    );
    point_t middle_line_point_xy = middle_line_point;
    middle_line_point_xy[2] = 0;

    point_t positive_direction_furthest_point = positive_direction_line_points.front();
    double positive_direction_furthest_distance = std::numeric_limits<double>::min();

    for (auto & point : positive_direction_line_points) {
        point_t point_xy = point;
        point_xy[2] = 0;

        double distance = (point_xy - middle_line_point_xy).norm();

        if (distance > positive_direction_furthest_distance) {
            positive_direction_furthest_distance = distance;
            positive_direction_furthest_point = point;
        }
    }

    RCLCPP_DEBUG(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Found furthest point in positive direction:\n[%f, %f, %f]",
        positive_direction_furthest_point[0], positive_direction_furthest_point[1], positive_direction_furthest_point[2]
    );

    point_t negative_direction_furthest_point = negative_direction_line_points.front();
    double negative_direction_furthest_distance = std::numeric_limits<double>::min();

    for (auto & point : negative_direction_line_points) {
        point_t point_xy = point;
        point_xy[2] = 0;

        double distance = (point_xy - middle_line_point_xy).norm();

        if (distance > negative_direction_furthest_distance) {
            negative_direction_furthest_distance = distance;
            negative_direction_furthest_point = point;
        }
    }

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
    point_t start_state_pos_no_z = start_state_position;
    start_state_pos_no_z[2] = 0;

    double start_state_position_dot_prod_plane_normal = start_state_pos_no_z.dot(powerline_normal);
    double positive_direction_furthest_point_dot_prod_plane_normal = positive_direction_furthest_point.dot(powerline_normal);
    double negative_direction_furthest_point_dot_prod_plane_normal = negative_direction_furthest_point.dot(powerline_normal);

    bool start_state_is_on_positive_side = start_state_position_dot_prod_plane_normal > positive_direction_furthest_point_dot_prod_plane_normal;
    bool start_state_is_on_negative_side = start_state_position_dot_prod_plane_normal < negative_direction_furthest_point_dot_prod_plane_normal;

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

    // Push current position:
    shared_queue->push_back(start_state_position);

    // Create waypoints
    // Get the xy distance from the middle line point to the start position along the powerline direction:
    vector_t powerline_direction_no_z = powerline_direction;
    powerline_direction_no_z[2] = 0;
    powerline_direction_no_z /= powerline_direction_no_z.norm();
    double start_state_position_dot_prod_plane_direction = start_state_pos_no_z.dot(powerline_direction_no_z);
    double middle_line_point_dot_prod_plane_direction = middle_line_point_xy.dot(powerline_direction_no_z);
    double distance_to_middle_line_point = abs(start_state_position_dot_prod_plane_direction - middle_line_point_dot_prod_plane_direction);

    if (!start_state_is_on_positive_side && !start_state_is_on_negative_side && distance_to_middle_line_point < parameters_->GetParameter("inside_powerline_xy_distance_threshold_m").as_double()) {
        // Drone is inside the powerline, no need to go around it.
        RCLCPP_DEBUG(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Drone is inside the powerline, no need to go around it."
        );
        point_t waypoint = middle_line_point;
        waypoint[2] = positive_direction_is_higher ? positive_direction_furthest_point[2] : negative_direction_furthest_point[2];
        waypoint[2] -= parameters_->GetParameter("under_cable_clearance_m").as_double();

        shared_queue->push_back(waypoint);

        setOutput("waypoints_depart", shared_queue);

        // Reverse the waypoints for the return path as new shared queue, without modifying the original shared queue.
        std::shared_ptr<std::deque<point_t>> shared_queue_return = std::make_shared<std::deque<point_t>>(shared_queue->rbegin(), shared_queue->rend());

        setOutput("waypoints_return", shared_queue_return);

        RCLCPP_INFO(
            node_->get_logger(),
            "PowerlineWaypointProviderActionNode::tick(): Finished computing waypoints"
        );

        return NodeStatus::SUCCESS;

    }

    point_t negative_top_waypoint = negative_direction_furthest_point;
    negative_top_waypoint[2] = highest_z;
    negative_top_waypoint[2] += parameters_->GetParameter("top_clearance_m").as_double();
    vector_t waypoint_displacement = parameters_->GetParameter("horizontal_clearance_m").as_double() * powerline_normal / powerline_normal.norm();
    waypoint_displacement[2] = 0;
    waypoint_displacement *= -1;
    negative_top_waypoint += waypoint_displacement;

    point_t positive_top_waypoint = positive_direction_furthest_point;
    positive_top_waypoint[2] = highest_z;
    positive_top_waypoint[2] += parameters_->GetParameter("top_clearance_m").as_double();
    waypoint_displacement *= -1;
    positive_top_waypoint += waypoint_displacement;

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

    point_t waypoint;

    waypoint = positive_direction_is_higher ? positive_direction_furthest_point : negative_direction_furthest_point;
    waypoint[2] -= parameters_->GetParameter("under_cable_clearance_m").as_double();
    waypoint_displacement = parameters_->GetParameter("horizontal_clearance_m").as_double() * powerline_normal / powerline_normal.norm();
    waypoint_displacement[2] = 0;
    waypoint_displacement *= positive_direction_is_higher ? 1 : -1;
    waypoint += waypoint_displacement;

    shared_queue->push_back(waypoint);

    point_t waypoint_cp = waypoint;
    waypoint = middle_line_point;
    waypoint[2] = waypoint_cp[2];

    shared_queue->push_back(waypoint);

    setOutput("waypoints_depart", shared_queue);

    // Reverse the waypoints for the return path as new shared queue, without modifying the original shared queue.
    std::shared_ptr<std::deque<point_t>> shared_queue_return = std::make_shared<std::deque<point_t>>(shared_queue->rbegin(), shared_queue->rend());

    setOutput("waypoints_return", shared_queue_return);

    RCLCPP_INFO(
        node_->get_logger(),
        "PowerlineWaypointProviderActionNode::tick(): Finished computing waypoints"
    );

    return NodeStatus::SUCCESS;

}

// SharedQueue<point_t> PowerlineWaypointProviderActionNode::applyLinearInterpolation(
//     SharedQueue<point_t> points
// ) {

//     float interpolation_segment_distance = parameters_->GetParameter("interpolation_segment_distance_m").as_double();

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