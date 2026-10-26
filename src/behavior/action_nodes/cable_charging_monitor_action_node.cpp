/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/cable_charging_monitor_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::configuration;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

CableChargingMonitorActionNode::CableChargingMonitorActionNode(
    const std::string & name,
    const NodeConfig & config,
    std::shared_ptr<rclcpp::Node> node,
    Configuration::SharedPtr configuration,
    BT::Blackboard::Ptr global_blackboard
) : StatefulActionNode(name, config),
    node_(node),
    configuration_(configuration),
    global_blackboard_(global_blackboard),
    charger_status_(
        *node_,
        "/payload/charger_gripper/charger_status",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort()
    ),
    start_time_(0, 0, node_->get_clock()->get_clock_type())
{
}

PortsList CableChargingMonitorActionNode::providedPorts() {
    return {
        InputPort<double>("minimum_stay_on_cable_s", -1.0, "Override normal minimum charging dwell.")
    };
}

double CableChargingMonitorActionNode::parameterOr(
    const std::string & name,
    double fallback
) const {
    if (configuration_ && configuration_->HasParameter(name)) {
        return configuration_->GetParameter(name).as_double();
    }
    return fallback;
}

bool CableChargingMonitorActionNode::boolParameterOr(
    const std::string & name,
    bool fallback
) const {
    if (configuration_ && configuration_->HasParameter(name)) {
        return configuration_->GetParameter(name).as_bool();
    }
    return fallback;
}

bool CableChargingMonitorActionNode::blackboardBool(
    const std::string & key,
    bool fallback
) const {
    bool value = fallback;
    // The Cable Charging executor retains its local blackboard between
    // cycles. An Inspection intent can update the shared flag after the
    // previous Cable Charging cycle wrote a local false value, so read the
    // shared value first to avoid shadowing the new cross-mode intent.
    if (global_blackboard_ && global_blackboard_->get(key, value)) {
        return value;
    }
    if (config().blackboard && config().blackboard->get(key, value)) {
        return value;
    }
    return fallback;
}

NodeStatus CableChargingMonitorActionNode::onStart() {
    start_time_ = node_->get_clock()->now();
    return evaluateChargingState();
}

NodeStatus CableChargingMonitorActionNode::onRunning() {
    return evaluateChargingState();
}

void CableChargingMonitorActionNode::onHalted() {
    start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

NodeStatus CableChargingMonitorActionNode::evaluateChargingState() {
    const rclcpp::Time now = node_->get_clock()->now();
    if (start_time_.nanoseconds() == 0) {
        start_time_ = now;
    }

    if (blackboardBool("charging.interrupt_requested", false)) {
        // An explicit operator intent, not an anomaly.
        RCLCPP_INFO(node_->get_logger(), "CableChargingMonitorActionNode::evaluateChargingState(): Charging interrupted by runtime intent.");
        return NodeStatus::SUCCESS;
    }

    double minimum_stay_s = parameterOr("/cable_charging/minimum_stay_on_cable_s", 10.0);
    double input_minimum_stay_s = -1.0;
    if (getInput("minimum_stay_on_cable_s", input_minimum_stay_s) && input_minimum_stay_s >= 0.0) {
        minimum_stay_s = input_minimum_stay_s;
    }

    const double dwell_s = (now - start_time_).seconds();
    if (dwell_s < minimum_stay_s) {
        return NodeStatus::RUNNING;
    }

    const bool mission_bypass_battery_checks = boolParameterOr("/mission/bypass_battery_checks", false);
    const bool bypass_full_check =
        mission_bypass_battery_checks ||
        blackboardBool("charging.bypass_battery_full_check", false) ||
        blackboardBool("charging.stay_on_cable", false);

    if (bypass_full_check) {
        return NodeStatus::RUNNING;
    }

    const auto charger_status = charger_status_.latest();
    if (!charger_status) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "CableChargingMonitorActionNode::evaluateChargingState(): Waiting for charger status."
        );
        return NodeStatus::RUNNING;
    }

    if (charger_status->message.charger_status == iii_drone_interfaces::msg::ChargerStatus::CHARGER_STATUS_FULLY_CHARGED) {
        RCLCPP_INFO(
            node_->get_logger(),
            "CableChargingMonitorActionNode::evaluateChargingState(): Charger reports fully charged after %.2f s.",
            dwell_s
        );
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}
