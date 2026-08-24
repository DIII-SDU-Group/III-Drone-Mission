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
    start_time_(0, 0, node_->get_clock()->get_clock_type())
{
    charger_status_sub_ = node_->create_subscription<iii_drone_interfaces::msg::ChargerStatus>(
        "/payload/charger_gripper/charger_status",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const iii_drone_interfaces::msg::ChargerStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_charger_status_ = msg->charger_status;
            has_status_ = true;
        }
    );
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
    if (config().blackboard && config().blackboard->get(key, value)) {
        return value;
    }
    if (global_blackboard_ && global_blackboard_->get(key, value)) {
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
        RCLCPP_WARN(node_->get_logger(), "CableChargingMonitorActionNode::evaluateChargingState(): Charging interrupted by runtime intent.");
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

    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_status_) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            5000,
            "CableChargingMonitorActionNode::evaluateChargingState(): Waiting for charger status."
        );
        return NodeStatus::RUNNING;
    }

    if (latest_charger_status_ == iii_drone_interfaces::msg::ChargerStatus::CHARGER_STATUS_FULLY_CHARGED) {
        RCLCPP_INFO(
            node_->get_logger(),
            "CableChargingMonitorActionNode::evaluateChargingState(): Charger reports fully charged after %.2f s.",
            dwell_s
        );
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}
