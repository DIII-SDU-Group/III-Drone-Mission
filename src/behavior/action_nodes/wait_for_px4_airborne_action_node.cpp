/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <cstdio>
#include <iii_drone_mission/behavior/action_nodes/wait_for_px4_airborne_action_node.hpp>

#include <cmath>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

WaitForPX4AirborneActionNode::WaitForPX4AirborneActionNode(
    const std::string & name,
    const NodeConfig & config,
    std::shared_ptr<rclcpp::Node> node
) : StatefulActionNode(name, config),
    node_(node),
    land_detected_(
        *node_,
        "/fmu/out/vehicle_land_detected",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local()
    ),
    position_setpoint_(
        *node_,
        "/fmu/out/vehicle_local_position_setpoint",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local()
    ),
    start_time_(0, 0, node_->get_clock()->get_clock_type())
{
}

PortsList WaitForPX4AirborneActionNode::providedPorts() {
    return {
        InputPort<int>("hold_ms", 1000, "How long PX4 must report airborne without interruption"),
        InputPort<int>("timeout_ms", 8000, "Fail when not held airborne within this time"),
        InputPort<double>("min_thrust", 0.1, "Normalized upward thrust PX4 must command (zero before PX4 takes off)"),
        InputPort<bool>("probe", false, "Not being airborne is an expected outcome (a branch condition): report it at INFO, not ERROR")
    };
}

bool WaitForPX4AirborneActionNode::Airborne(
    const px4_msgs::msg::VehicleLandDetected & sample,
    const rclcpp::Time & receive_time,
    const rclcpp::Time & now
) {
    return (now - receive_time).seconds() <= kMaxLandSampleAgeS &&
        !sample.landed && !sample.maybe_landed && !sample.ground_contact;
}

bool WaitForPX4AirborneActionNode::Thrusting(
    const px4_msgs::msg::VehicleLocalPositionSetpoint & sample,
    const rclcpp::Time & receive_time,
    const rclcpp::Time & now,
    double min_thrust
) {
    // NED: upward thrust is negative z.
    return (now - receive_time).seconds() <= kMaxThrustSampleAgeS &&
        std::isfinite(sample.thrust[2]) && -sample.thrust[2] >= min_thrust;
}

NodeStatus WaitForPX4AirborneActionNode::onStart() {
    int hold_ms = 1000;
    int timeout_ms = 8000;
    getInput("hold_ms", hold_ms);
    getInput("timeout_ms", timeout_ms);
    getInput("min_thrust", min_thrust_);
    probe_ = false;
    getInput("probe", probe_);
    if (hold_ms < 0 || timeout_ms <= hold_ms) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "WaitForPX4AirborneActionNode::onStart(): %s: need 0 <= hold_ms < timeout_ms, got %d and %d",
            name().c_str(), hold_ms, timeout_ms
        );
        return NodeStatus::FAILURE;
    }
    hold_s_ = hold_ms / 1000.0;
    timeout_s_ = timeout_ms / 1000.0;
    start_time_ = node_->get_clock()->now();
    airborne_since_.reset();
    return onRunning();
}

NodeStatus WaitForPX4AirborneActionNode::onRunning() {
    const rclcpp::Time now = node_->get_clock()->now();
    const auto sample = land_detected_.latest();
    const auto setpoint = position_setpoint_.latest();
    const bool airborne = sample && Airborne(sample->message, sample->receive_time, now) &&
        setpoint && Thrusting(setpoint->message, setpoint->receive_time, now, min_thrust_);

    if (airborne) {
        if (!airborne_since_) airborne_since_ = now;
        if ((now - *airborne_since_).seconds() >= hold_s_) {
            RCLCPP_INFO(
                node_->get_logger(),
                "WaitForPX4AirborneActionNode::onRunning(): %s: PX4 airborne with thrust %.3f for %.2f s.",
                name().c_str(), -setpoint->message.thrust[2], (now - *airborne_since_).seconds()
            );
            return NodeStatus::SUCCESS;
        }
    } else {
        airborne_since_.reset();
    }

    if ((now - start_time_).seconds() > timeout_s_) {
        char report[320];
        if (sample && setpoint) {
            std::snprintf(
                report, sizeof(report),
                "%s: PX4 not held airborne with thrust within %.1f s: "
                "landed=%d maybe_landed=%d ground_contact=%d (age %.2f s), thrust %.3f (age %.2f s).",
                name().c_str(), timeout_s_,
                sample->message.landed, sample->message.maybe_landed, sample->message.ground_contact,
                (now - sample->receive_time).seconds(),
                -setpoint->message.thrust[2], (now - setpoint->receive_time).seconds()
            );
        } else {
            std::snprintf(
                report, sizeof(report), "%s: no PX4 %s sample within %.1f s.",
                name().c_str(), sample ? "position setpoint" : "land-detector", timeout_s_
            );
        }
        if (probe_) {
            RCLCPP_INFO(node_->get_logger(), "WaitForPX4AirborneActionNode::onRunning(): %s", report);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "WaitForPX4AirborneActionNode::onRunning(): %s", report);
        }
        return NodeStatus::FAILURE;
    }

    return NodeStatus::RUNNING;
}

void WaitForPX4AirborneActionNode::onHalted() {
    airborne_since_.reset();
}
