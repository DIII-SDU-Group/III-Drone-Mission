/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/find_powerline_waypoint_provider_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::control;
using namespace iii_drone::types;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FindPowerlineWaypointProviderActionNode::FindPowerlineWaypointProviderActionNode(
    const std::string & name, 
    const NodeConfiguration & conf,
    tf2_ros::Buffer::SharedPtr tf_buffer
) : SyncActionNode(name, conf), tf_buffer_(tf_buffer) { }

PortsList FindPowerlineWaypointProviderActionNode::providedPorts() {

    return {
        InputPort<iii_drone_interfaces::msg::Powerline>("stored_powerline"),
        InputPort<State>("start_state"),
        OutputPort<SharedQueue<point_t>>("waypoints")
    };

}

NodeStatus FindPowerlineWaypointProviderActionNode::tick() {

    // Get the stored powerline.
    iii_drone_interfaces::msg::Powerline stored_powerline;
    if (!getInput("stored_powerline", stored_powerline)) {
        return NodeStatus::FAILURE;
    }

    State start_state;
    if (!getInput("start_state", start_state)) {
        return NodeStatus::FAILURE;
    }

    PowerlineAdapter powerline_adapter(stored_powerline);

    std::vector<point_t> powerline_points;

    for (auto & line_adapter : powerline_adapter.single_line_adapters()) {
        geometry_msgs::msg::PointStamped point_stamped_msg;
        point_stamped_msg.header.frame_id = line_adapter.frame_id();

        point_stamped_msg.point = pointMsgFromPoint(line_adapter.position());

        point_stamped_msg = tf_buffer_->transform(point_stamped_msg, "world");

        powerline_points.push_back(pointFromPointMsg(point_stamped_msg.point));
    }

    vector_t powerline_direction = powerline_adapter.projection_plane().normal;

    geometry_msgs::msg::Vector3Stamped powerline_direction_msg;
    powerline_direction_msg.header.frame_id = powerline_adapter.single_line_adapters().front().frame_id();
    powerline_direction_msg.vector = vectorMsgFromVector(powerline_direction);

    powerline_direction_msg = tf_buffer_->transform(powerline_direction_msg, "world");

    powerline_direction = vectorFromVectorMsg(powerline_direction_msg.vector);

    // Powerline normal is powerline direction rotated 90 degrees around the z-axis.
    vector_t powerline_normal = powerline_direction;
    powerline_normal[0] = -powerline_direction[1];
    powerline_normal[1] = powerline_direction[0];

    point_t middle_line_point;
    double lowest_point_normal_dot_product = std::numeric_limits<double>::max();

    for (auto & point : powerline_points) {
        double normal_dot_product = point.dot(powerline_normal);

        if (normal_dot_product < lowest_point_normal_dot_product) {
            lowest_point_normal_dot_product = normal_dot_product;
            middle_line_point = point;
        }
    }

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

    // Find the side of the powerline where the point with the lowest z value is higher than for the other side.
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

    // Find the furthest (in xy) point from the middle point.
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

    bool positive_direction_is_higher = positive_direction_lowest_point[2] > negative_direction_lowest_point[2];

    // Find if the start state is on the positive or negative side of the powerline.
    bool start_state_is_on_positive_side = start_state.position().dot(powerline_normal) > 0;

    // Create a queue of waypoints.
    auto shared_queue = std::make_shared<std::deque<point_t>>();

    point_t negative_top_waypoint = middle_line_point;
    negative_top_waypoint[2] += 2;
    vector_t waypoint_displacement = 2 * powerline_normal / powerline_normal.norm();
    waypoint_displacement[2] = 0;
    waypoint_displacement *= -1;
    negative_top_waypoint += waypoint_displacement;

    point_t positive_top_waypoint = middle_line_point;
    positive_top_waypoint[2] += 2;
    waypoint_displacement *= -1;
    positive_top_waypoint += waypoint_displacement;

    // Create waypoints
    if (positive_direction_is_higher) {
        if (!start_state_is_on_positive_side) {
            shared_queue->push_back(negative_top_waypoint);
            shared_queue->push_back(positive_top_waypoint);
        }
    } else {
        if (start_state_is_on_positive_side) {
            shared_queue->push_back(positive_top_waypoint);
            shared_queue->push_back(negative_top_waypoint);
        }
    }

    point_t waypoint;

    waypoint = negative_direction_lowest_point;
    waypoint[2] -= 1.5;
    waypoint_displacement = 2 * powerline_normal / powerline_normal.norm();
    waypoint_displacement[2] = 0;
    waypoint_displacement *= positive_direction_is_higher ? 1 : -1;
    waypoint += waypoint_displacement;

    shared_queue->push_back(waypoint);

    point_t waypoint_cp = waypoint;
    waypoint = middle_line_point;
    waypoint[2] = waypoint_cp[2];

    shared_queue->push_back(waypoint);

    setOutput("waypoints", shared_queue);

    return NodeStatus::SUCCESS;

}