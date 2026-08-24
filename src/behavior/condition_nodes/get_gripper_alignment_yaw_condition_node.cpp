/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.hpp>

#include <algorithm>
#include <cmath>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

using namespace BT;

namespace {

double wrapToPi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

double wrapCableAxisYawError(double angle) {
    angle = wrapToPi(angle);
    if (angle > M_PI_2) {
        angle -= M_PI;
    } else if (angle < -M_PI_2) {
        angle += M_PI;
    }
    return angle;
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GetGripperAlignmentYawConditionNode::GetGripperAlignmentYawConditionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer
) : RosTopicSubNode<iii_drone_interfaces::msg::Powerline>(
        name, 
        conf, 
        params
),  node_(params.nh.lock()),
    tf_buffer_(tf_buffer) { }

PortsList GetGripperAlignmentYawConditionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<int>("target_cable_id", -1, "Target cable ID. Uses the first line when negative."),
        InputPort<double>("settling_window_s", 1.5, "Required duration of stable live cable-axis samples."),
        InputPort<double>("settling_max_yaw_deviation_rad", 0.08, "Maximum cable-axis deviation from the live sample mean."),
        InputPort<int>("settling_min_samples", 6, "Minimum number of distinct live perception samples."),
        OutputPort<float>("target_yaw")
    });

}

NodeStatus GetGripperAlignmentYawConditionNode::onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) {

    RCLCPP_DEBUG(node_->get_logger(), "GetGripperAlignmentYawConditionNode::onTick(): Ticking.");

    if (!last_msg) {
        RCLCPP_DEBUG(node_->get_logger(), "GetGripperAlignmentYawConditionNode::onTick(): No powerline message received.");
        return NodeStatus::FAILURE;
    }

    if (last_msg->lines.size() == 0) {
        RCLCPP_DEBUG(node_->get_logger(), "GetGripperAlignmentYawConditionNode::onTick(): No lines in powerline message.");
        return NodeStatus::FAILURE;
    }

    int target_cable_id = -1;
    getInput("target_cable_id", target_cable_id);

    RCLCPP_DEBUG(
        node_->get_logger(),
        "GetGripperAlignmentYawConditionNode::onTick(): Requested target_cable_id=%d with %zu detected line(s)",
        target_cable_id,
        last_msg->lines.size()
    );

    PowerlineAdapter powerline_adapter(*last_msg);

    if (settling_target_cable_id_ != target_cable_id) {
        resetSettlingState();
        settling_target_cable_id_ = target_cable_id;
    }

    SingleLineAdapter target_line;
    try {
        if (target_cable_id >= 0) {
            if (!powerline_adapter.HasLine(target_cable_id)) {
                RCLCPP_DEBUG(
                    node_->get_logger(),
                    "GetGripperAlignmentYawConditionNode::onTick(): Target cable id=%d is absent from the latest live perception message; restarting settling window.",
                    target_cable_id
                );
                resetSettlingState();
                settling_target_cable_id_ = target_cable_id;
                return NodeStatus::FAILURE;
            }
            target_line = powerline_adapter.GetLine(target_cable_id);
        } else {
            target_line = powerline_adapter.single_line_adapters().front();
        }
    } catch (std::runtime_error & ex) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GetGripperAlignmentYawConditionNode::onTick(): Could not select target line: %s",
            ex.what()
        );
        return NodeStatus::FAILURE;
    }

    geometry_msgs::msg::QuaternionStamped source_q_cable_msg;
    source_q_cable_msg.header.stamp = target_line.stamp();
    source_q_cable_msg.header.frame_id = target_line.frame_id();
    source_q_cable_msg.quaternion = quaternionMsgFromQuaternion(target_line.quaternion());

    geometry_msgs::msg::QuaternionStamped gripper_q_cable_msg;
    try {
        gripper_q_cable_msg = tf_buffer_->transform(
            source_q_cable_msg,
            "cable_gripper"
        );
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GetGripperAlignmentYawConditionNode::onTick(): Could not transform cable quaternion to gripper frame: %s",
            ex.what()
        );
        return NodeStatus::FAILURE;
    }

    geometry_msgs::msg::TransformStamped world_T_drone_msg;
    try {
        world_T_drone_msg = tf_buffer_->lookupTransform(
            "world",
            "drone",
            tf2::TimePointZero
        );
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "GetGripperAlignmentYawConditionNode::onTick(): Could not transform drone to world frame: %s",
            ex.what()
        );
        return NodeStatus::FAILURE;
    }

    const double raw_yaw_error = quatToEul(quaternionFromQuaternionMsg(gripper_q_cable_msg.quaternion))[2];
    const double yaw_error = wrapCableAxisYawError(raw_yaw_error);
    const double current_yaw = quatToEul(quaternionFromTransformMsg(world_T_drone_msg.transform))[2];
    const double live_target_yaw = current_yaw + yaw_error;

    const int64_t line_stamp_ns = target_line.stamp().nanoseconds();
    if (line_stamp_ns == last_line_stamp_ns_) {
        return NodeStatus::FAILURE;
    }
    last_line_stamp_ns_ = line_stamp_ns;

    double settling_window_s = 1.5;
    double settling_max_yaw_deviation_rad = 0.08;
    int settling_min_samples = 6;
    getInput("settling_window_s", settling_window_s);
    getInput("settling_max_yaw_deviation_rad", settling_max_yaw_deviation_rad);
    getInput("settling_min_samples", settling_min_samples);

    const auto now = std::chrono::steady_clock::now();
    const double live_axis_yaw = wrapCableAxisYawError(live_target_yaw);

    const auto settled_axis_yaw = yaw_settler_.addSample(
        live_axis_yaw,
        now,
        settling_window_s,
        settling_max_yaw_deviation_rad,
        static_cast<std::size_t>(std::max(1, settling_min_samples))
    );
    const auto sample_count = yaw_settler_.sampleCount();
    const double sample_span_s = yaw_settler_.sampleSpanSeconds();
    const double max_deviation = yaw_settler_.maxDeviationRadians();

    if (!settled_axis_yaw) {
        RCLCPP_DEBUG(
            node_->get_logger(),
            "GetGripperAlignmentYawConditionNode::onTick(): Waiting for live yaw to settle: samples=%zu/%d span=%.2f/%.2f s max_deviation=%.3f/%.3f rad.",
            sample_count,
            settling_min_samples,
            sample_span_s,
            settling_window_s,
            max_deviation,
            settling_max_yaw_deviation_rad
        );
        return NodeStatus::FAILURE;
    }

    const double target_yaw = current_yaw + wrapCableAxisYawError(*settled_axis_yaw - current_yaw);

    setOutput("target_yaw", static_cast<float>(target_yaw));

    RCLCPP_DEBUG(
        node_->get_logger(),
        "GetGripperAlignmentYawConditionNode::onTick(): Selected line id=%d position=[%.3f, %.3f, %.3f] frame=%s",
        target_line.id(),
        target_line.position()[0],
        target_line.position()[1],
        target_line.position()[2],
        target_line.frame_id().c_str()
    );

    RCLCPP_INFO(
        node_->get_logger(),
        "GetGripperAlignmentYawConditionNode::onTick(): Target line id=%d frame=%s orientation_source=settled_live_perception samples=%zu span_s=%.2f max_deviation_rad=%.3f current_yaw=%f raw_gripper_yaw_error=%f yaw_error=%f target_yaw=%f",
        target_line.id(),
        target_line.frame_id().c_str(),
        sample_count,
        sample_span_s,
        max_deviation,
        current_yaw,
        raw_yaw_error,
        yaw_error,
        target_yaw
    );

    resetSettlingState();
    return NodeStatus::SUCCESS;

}

std::optional<double> CableAxisYawSettler::addSample(
    double axis_yaw,
    std::chrono::steady_clock::time_point received_at,
    double window_s,
    double max_yaw_deviation_rad,
    std::size_t min_samples
) {
    const auto window = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(0.0, window_s))
    );
    const auto cutoff = received_at - window;

    samples_.push_back({axis_yaw, received_at});

    // Keep the newest sample at or before the cutoff as the window boundary.
    while (samples_.size() > 1 && samples_[1].received_at <= cutoff) {
        samples_.pop_front();
    }

    const double mean_axis_yaw = meanCableAxisYaw();
    max_deviation_rad_ = 0.0;
    for (const auto & sample : samples_) {
        max_deviation_rad_ = std::max(
            max_deviation_rad_,
            std::abs(wrapCableAxisYawError(sample.axis_yaw - mean_axis_yaw))
        );
    }

    const bool covers_window = !samples_.empty() && samples_.front().received_at <= cutoff;
    if (
        !covers_window
        || samples_.size() < std::max<std::size_t>(1, min_samples)
        || max_deviation_rad_ > std::max(0.0, max_yaw_deviation_rad)
    ) {
        return std::nullopt;
    }

    return mean_axis_yaw;
}

void CableAxisYawSettler::reset() {
    samples_.clear();
    max_deviation_rad_ = 0.0;
}

std::size_t CableAxisYawSettler::sampleCount() const {
    return samples_.size();
}

double CableAxisYawSettler::sampleSpanSeconds() const {
    if (samples_.empty()) {
        return 0.0;
    }
    return std::chrono::duration<double>(
        samples_.back().received_at - samples_.front().received_at
    ).count();
}

double CableAxisYawSettler::maxDeviationRadians() const {
    return max_deviation_rad_;
}

double CableAxisYawSettler::meanCableAxisYaw() const {
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto & sample : samples_) {
        sin_sum += std::sin(2.0 * sample.axis_yaw);
        cos_sum += std::cos(2.0 * sample.axis_yaw);
    }
    return 0.5 * std::atan2(sin_sum, cos_sum);
}

void GetGripperAlignmentYawConditionNode::resetSettlingState() {
    last_line_stamp_ns_ = -1;
    yaw_settler_.reset();
}
