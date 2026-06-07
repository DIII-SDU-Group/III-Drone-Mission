/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/verify_powerline_detected_condition_node.hpp>

#include <limits>

using namespace iii_drone::behavior;
using namespace iii_drone::types;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

VerifyPowerlineDetectedConditionNode::VerifyPowerlineDetectedConditionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer
) : RosTopicSubNode<iii_drone_interfaces::msg::Powerline>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh.lock()),
    tf_buffer_(tf_buffer) { }

PortsList VerifyPowerlineDetectedConditionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<unsigned int>("required_n_lines"),
        InputPort<iii_drone_interfaces::msg::Powerline>("powerline_overview"),
        InputPort<int>("powerline_overview_required_line_id"),
        InputPort<double>("line_match_distance_threshold_m"),
        InputPort<double>("relaxed_line_match_distance_threshold_m"),
        InputPort<double>("line_match_ambiguity_margin_m"),
        OutputPort<unsigned int>("n_lines"),
        OutputPort<int>("matched_detected_line_id"),
        OutputPort<double>("matched_line_distance_m")
    });

}

bool VerifyPowerlineDetectedConditionNode::getOverviewLinePoint(
    const iii_drone_interfaces::msg::Powerline & powerline_overview,
    int line_id,
    point_t & point
) {

    for (const auto & line : powerline_overview.lines) {
        if (line.id == line_id) {
            point = point_t(
                line.pose.position.x,
                line.pose.position.y,
                line.pose.position.z
            );
            return true;
        }
    }

    return false;

}

double VerifyPowerlineDetectedConditionNode::distanceInPlaneOrthogonalToDirection(
    const point_t & a,
    const point_t & b,
    const vector_t & direction
) {

    vector_t unit_direction = direction;
    if (unit_direction.norm() <= std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<double>::infinity();
    }
    unit_direction.normalize();

    const vector_t delta = a - b;
    const vector_t delta_in_orthogonal_plane = delta - delta.dot(unit_direction) * unit_direction;
    return delta_in_orthogonal_plane.norm();

}

bool VerifyPowerlineDetectedConditionNode::transformLinePointToFrame(
    const iii_drone_interfaces::msg::SingleLine & line,
    const std::string & target_frame_id,
    point_t & point
) const {

    if (line.header.frame_id == target_frame_id) {
        point = point_t(
            line.pose.position.x,
            line.pose.position.y,
            line.pose.position.z
        );
        return true;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = line.header;
    pose.pose = line.pose;

    try {
        const auto transformed_pose = tf_buffer_->transform(pose, target_frame_id);
        point = point_t(
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z
        );
        return true;
    } catch (tf2::TransformException & ex) {
        geometry_msgs::msg::PoseStamped latest_pose = pose;
        latest_pose.header.stamp = builtin_interfaces::msg::Time();
        try {
            const auto transformed_pose = tf_buffer_->transform(latest_pose, target_frame_id);
            point = point_t(
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z
            );
            RCLCPP_INFO_ONCE(
                node_ptr_->get_logger(),
                "VerifyPowerlineDetectedConditionNode::transformLinePointToFrame(): %s: Failed to transform detected line id %d from %s to %s at message stamp, using latest transform instead: %s",
                name().c_str(),
                line.id,
                line.header.frame_id.c_str(),
                target_frame_id.c_str(),
                ex.what()
            );
            return true;
        } catch (tf2::TransformException & latest_ex) {
            RCLCPP_WARN(
                node_ptr_->get_logger(),
                "VerifyPowerlineDetectedConditionNode::transformLinePointToFrame(): %s: Failed to transform detected line id %d from %s to %s at message stamp and latest transform: %s; latest: %s",
                name().c_str(),
                line.id,
                line.header.frame_id.c_str(),
                target_frame_id.c_str(),
                ex.what(),
                latest_ex.what()
            );
            return false;
        }
    }

}

NodeStatus VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(
    const iii_drone_interfaces::msg::Powerline & detected_powerline,
    const iii_drone_interfaces::msg::Powerline & powerline_overview,
    int powerline_overview_required_line_id,
    double line_match_distance_threshold_m,
    double relaxed_line_match_distance_threshold_m,
    double line_match_ambiguity_margin_m,
    int & matched_detected_line_id,
    double & matched_line_distance_m
) const {

    point_t overview_line_point;
    if (!getOverviewLinePoint(powerline_overview, powerline_overview_required_line_id, overview_line_point)) {
        RCLCPP_WARN(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(): %s: Overview line id %d not found",
            name().c_str(),
            powerline_overview_required_line_id
        );
        return NodeStatus::FAILURE;
    }

    const vector_t powerline_direction(
        powerline_overview.projection_plane.normal.x,
        powerline_overview.projection_plane.normal.y,
        powerline_overview.projection_plane.normal.z
    );
    const std::string overview_frame_id = powerline_overview.lines.empty() ?
        std::string() :
        powerline_overview.lines.front().header.frame_id;
    if (overview_frame_id.empty()) {
        RCLCPP_WARN(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(): %s: Overview frame id unavailable",
            name().c_str()
        );
        return NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(): %s: Matching overview line id %d point=[%.3f, %.3f, %.3f] frame=%s threshold=%.3f direction=[%.3f, %.3f, %.3f]",
        name().c_str(),
        powerline_overview_required_line_id,
        overview_line_point[0],
        overview_line_point[1],
        overview_line_point[2],
        overview_frame_id.c_str(),
        line_match_distance_threshold_m,
        powerline_direction[0],
        powerline_direction[1],
        powerline_direction[2]
    );

    double best_distance = std::numeric_limits<double>::infinity();
    int best_detected_line_id = -1;
    double best_nearest_other_overview_distance = std::numeric_limits<double>::infinity();
    for (const auto & detected_line : detected_powerline.lines) {
        point_t detected_line_point;
        if (!transformLinePointToFrame(detected_line, overview_frame_id, detected_line_point)) {
            continue;
        }
        const double distance = distanceInPlaneOrthogonalToDirection(
            overview_line_point,
            detected_line_point,
            powerline_direction
        );
        double nearest_other_overview_distance = std::numeric_limits<double>::infinity();
        int nearest_other_overview_line_id = -1;
        for (const auto & overview_line : powerline_overview.lines) {
            if (overview_line.id == powerline_overview_required_line_id) {
                continue;
            }
            const point_t other_overview_line_point(
                overview_line.pose.position.x,
                overview_line.pose.position.y,
                overview_line.pose.position.z
            );
            const double other_distance = distanceInPlaneOrthogonalToDirection(
                other_overview_line_point,
                detected_line_point,
                powerline_direction
            );
            if (other_distance < nearest_other_overview_distance) {
                nearest_other_overview_distance = other_distance;
                nearest_other_overview_line_id = overview_line.id;
            }
        }
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(): %s: Candidate detected line id %d point=[%.3f, %.3f, %.3f] frame=%s orthogonal_distance=%.3f nearest_other_overview_id=%d nearest_other_distance=%.3f ambiguity_margin=%.3f",
            name().c_str(),
            detected_line.id,
            detected_line_point[0],
            detected_line_point[1],
            detected_line_point[2],
            overview_frame_id.c_str(),
            distance,
            nearest_other_overview_line_id,
            nearest_other_overview_distance,
            line_match_ambiguity_margin_m
        );
        if (distance < best_distance) {
            best_distance = distance;
            best_detected_line_id = detected_line.id;
            best_nearest_other_overview_distance = nearest_other_overview_distance;
        }
    }

    matched_line_distance_m = best_distance;

    const bool strict_match = best_distance <= line_match_distance_threshold_m;
    const bool relaxed_unambiguous_match =
        best_distance <= relaxed_line_match_distance_threshold_m &&
        best_nearest_other_overview_distance - best_distance >= line_match_ambiguity_margin_m;

    if (strict_match || relaxed_unambiguous_match) {
        matched_detected_line_id = best_detected_line_id;
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(): %s: Overview line id %d matched detected line id %d at distance %.3f m (strict=%s relaxed_unambiguous=%s nearest_other_distance=%.3f relaxed_threshold=%.3f ambiguity_margin=%.3f)",
            name().c_str(),
            powerline_overview_required_line_id,
            best_detected_line_id,
            best_distance,
            strict_match ? "true" : "false",
            relaxed_unambiguous_match ? "true" : "false",
            best_nearest_other_overview_distance,
            relaxed_line_match_distance_threshold_m,
            line_match_ambiguity_margin_m
        );
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(): %s: Matched detected line id %d within orthogonal-plane threshold %.3f m",
            name().c_str(),
            best_detected_line_id,
            line_match_distance_threshold_m
        );
        return NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(
        node_ptr_->get_logger(),
        "VerifyPowerlineDetectedConditionNode::verifyLineMatchesPowerlineOverview(): %s: Overview line id %d not matched; best detected line id %d distance %.3f m exceeds strict %.3f m and relaxed/unambiguous gate %.3f m with nearest_other_distance %.3f m and ambiguity_margin %.3f m",
        name().c_str(),
        powerline_overview_required_line_id,
        best_detected_line_id,
        best_distance,
        line_match_distance_threshold_m,
        relaxed_line_match_distance_threshold_m,
        best_nearest_other_overview_distance,
        line_match_ambiguity_margin_m
    );

    matched_detected_line_id = -1;
    return NodeStatus::FAILURE;

}

NodeStatus VerifyPowerlineDetectedConditionNode::onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "VerifyPowerlineDetectedConditionNode::onTick(): %s: Checking powerline detection",
        name().c_str()
    );

    if (!last_msg) {
        setOutput("n_lines", 0);
        return NodeStatus::FAILURE;
    }

    unsigned int required_n_lines;
    unsigned int n_lines;

    getInput("required_n_lines", required_n_lines);

    n_lines = last_msg->lines.size();

    setOutput("n_lines", n_lines);
    setOutput("matched_detected_line_id", -1);
    setOutput("matched_line_distance_m", std::numeric_limits<double>::infinity());

    int powerline_overview_required_line_id = -1;
    getInput("powerline_overview_required_line_id", powerline_overview_required_line_id);
    if (powerline_overview_required_line_id >= 0) {
        iii_drone_interfaces::msg::Powerline powerline_overview;
        if (!getInput("powerline_overview", powerline_overview)) {
            RCLCPP_WARN(
                node_ptr_->get_logger(),
                "VerifyPowerlineDetectedConditionNode::onTick(): %s: powerline_overview input not provided",
                name().c_str()
            );
            return NodeStatus::FAILURE;
        }

        double line_match_distance_threshold_m;
        if (!getInput("line_match_distance_threshold_m", line_match_distance_threshold_m)) {
            RCLCPP_WARN(
                node_ptr_->get_logger(),
                "VerifyPowerlineDetectedConditionNode::onTick(): %s: line_match_distance_threshold_m input not provided",
                name().c_str()
            );
            return NodeStatus::FAILURE;
        }
        double relaxed_line_match_distance_threshold_m = line_match_distance_threshold_m;
        getInput("relaxed_line_match_distance_threshold_m", relaxed_line_match_distance_threshold_m);

        double line_match_ambiguity_margin_m = 0.35;
        getInput("line_match_ambiguity_margin_m", line_match_ambiguity_margin_m);

        int matched_detected_line_id = -1;
        double matched_line_distance_m = std::numeric_limits<double>::infinity();

        const NodeStatus status = verifyLineMatchesPowerlineOverview(
            *last_msg,
            powerline_overview,
            powerline_overview_required_line_id,
            line_match_distance_threshold_m,
            relaxed_line_match_distance_threshold_m,
            line_match_ambiguity_margin_m,
            matched_detected_line_id,
            matched_line_distance_m
        );

        setOutput("matched_detected_line_id", matched_detected_line_id);
        setOutput("matched_line_distance_m", matched_line_distance_m);

        return status;
    }

    if (n_lines >= required_n_lines) {
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::onTick(): %s: Powerline detected",
            name().c_str()
        );
        return NodeStatus::SUCCESS;
    } else {
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::onTick(): %s: Powerline not detected",
            name().c_str()
        );
        return NodeStatus::FAILURE;
    }

}
