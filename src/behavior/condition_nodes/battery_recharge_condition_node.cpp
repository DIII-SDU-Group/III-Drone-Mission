/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/battery_recharge_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::configuration;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ShouldRechargeBatteryLowConditionNode::ShouldRechargeBatteryLowConditionNode(
    const std::string & name,
    const NodeConfig & config,
    std::shared_ptr<rclcpp::Node> node,
    Configuration::SharedPtr configuration
) : SyncActionNode(name, config),
    node_(node),
    configuration_(configuration),
    latest_voltage_receive_time_(0, 0, node_->get_clock()->get_clock_type()),
    low_voltage_since_(0, 0, node_->get_clock()->get_clock_type()),
    last_stale_retry_time_(0, 0, node_->get_clock()->get_clock_type())
{
    battery_voltage_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/payload/charger_gripper/battery_voltage",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_voltage_ = msg->data;
            latest_voltage_receive_time_ = node_->get_clock()->now();
            has_voltage_ = true;
            stale_failure_count_ = 0;
        }
    );
}

PortsList ShouldRechargeBatteryLowConditionNode::providedPorts() {
    return {
        InputPort<double>("battery_voltage_threshold_v", -1.0, "Override low-voltage threshold."),
        InputPort<double>("battery_voltage_debounce_s", -1.0, "Override low-voltage debounce time."),
        InputPort<double>("battery_topic_timeout_s", -1.0, "Override battery topic freshness timeout."),
        InputPort<int>("battery_check_retry_count", -1, "Override stale/missing retry count."),
        InputPort<double>("battery_check_retry_interval_s", -1.0, "Override stale/missing retry interval."),
        InputPort<bool>("bypass_battery_checks", false, "Disable automatic battery checks.")
    };
}

double ShouldRechargeBatteryLowConditionNode::parameterOr(
    const std::string & name,
    double fallback
) const {
    if (configuration_ && configuration_->HasParameter(name)) {
        return configuration_->GetParameter(name).as_double();
    }
    return fallback;
}

int ShouldRechargeBatteryLowConditionNode::parameterOr(
    const std::string & name,
    int fallback
) const {
    if (configuration_ && configuration_->HasParameter(name)) {
        return static_cast<int>(configuration_->GetParameter(name).as_int());
    }
    return fallback;
}

bool ShouldRechargeBatteryLowConditionNode::boolParameterOr(
    const std::string & name,
    bool fallback
) const {
    if (configuration_ && configuration_->HasParameter(name)) {
        return configuration_->GetParameter(name).as_bool();
    }
    return fallback;
}

NodeStatus ShouldRechargeBatteryLowConditionNode::tick() {
    bool bypass = boolParameterOr("/mission/bypass_battery_checks", false);
    getInput("bypass_battery_checks", bypass);
    if (bypass) {
        return NodeStatus::FAILURE;
    }

    double threshold_v = parameterOr("/inspection_demo/battery_voltage_threshold_v", 14.0);
    double debounce_s = parameterOr("/inspection_demo/battery_voltage_debounce_s", 2.0);
    double timeout_s = parameterOr("/inspection_demo/battery_topic_timeout_s", 2.0);
    int retry_count = parameterOr("/inspection_demo/battery_check_retry_count", 3);
    double retry_interval_s = parameterOr("/inspection_demo/battery_check_retry_interval_s", 0.2);

    double input_double = -1.0;
    if (getInput("battery_voltage_threshold_v", input_double) && input_double >= 0.0) {
        threshold_v = input_double;
    }
    if (getInput("battery_voltage_debounce_s", input_double) && input_double >= 0.0) {
        debounce_s = input_double;
    }
    if (getInput("battery_topic_timeout_s", input_double) && input_double >= 0.0) {
        timeout_s = input_double;
    }
    if (getInput("battery_check_retry_interval_s", input_double) && input_double >= 0.0) {
        retry_interval_s = input_double;
    }
    int input_int = -1;
    if (getInput("battery_check_retry_count", input_int) && input_int >= 0) {
        retry_count = input_int;
    }

    const rclcpp::Time now = node_->get_clock()->now();

    std::lock_guard<std::mutex> lock(mutex_);
    auto should_count_stale_retry = [&]() {
        if (last_stale_retry_time_.nanoseconds() == 0 ||
            (now - last_stale_retry_time_).seconds() >= retry_interval_s)
        {
            last_stale_retry_time_ = now;
            return true;
        }
        return false;
    };

    if (!has_voltage_) {
        if (!should_count_stale_retry()) {
            return NodeStatus::FAILURE;
        }
        ++stale_failure_count_;
        if (stale_failure_count_ >= static_cast<unsigned int>(retry_count)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "ShouldRechargeBatteryLowConditionNode::tick(): No charger battery voltage received after %u checks spaced by %.3fs",
                stale_failure_count_,
                retry_interval_s
            );
        }
        return NodeStatus::FAILURE;
    }

    const double age_s = (now - latest_voltage_receive_time_).seconds();
    if (age_s > timeout_s) {
        if (!should_count_stale_retry()) {
            return NodeStatus::FAILURE;
        }
        ++stale_failure_count_;
        if (stale_failure_count_ >= static_cast<unsigned int>(retry_count)) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "ShouldRechargeBatteryLowConditionNode::tick(): Charger battery voltage stale age=%.3fs timeout=%.3fs after %u checks spaced by %.3fs",
                age_s,
                timeout_s,
                stale_failure_count_,
                retry_interval_s
            );
        }
        return NodeStatus::FAILURE;
    }

    stale_failure_count_ = 0;
    last_stale_retry_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    if (latest_voltage_ >= threshold_v) {
        low_voltage_since_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        return NodeStatus::FAILURE;
    }

    if (low_voltage_since_.nanoseconds() == 0) {
        low_voltage_since_ = now;
        return NodeStatus::FAILURE;
    }

    const double low_duration_s = (now - low_voltage_since_).seconds();
    if (low_duration_s < debounce_s) {
        return NodeStatus::FAILURE;
    }

    RCLCPP_WARN(
        node_->get_logger(),
        "ShouldRechargeBatteryLowConditionNode::tick(): Battery low %.2f V below %.2f V for %.2f s",
        latest_voltage_,
        threshold_v,
        low_duration_s
    );
    return NodeStatus::SUCCESS;
}
