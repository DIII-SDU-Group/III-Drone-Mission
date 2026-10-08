#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <chrono>
#include <memory>
#include <cstdint>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/latest_message_subscription.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class ShouldRechargeBatteryLowConditionNode : public BT::SyncActionNode {
    public:
        ShouldRechargeBatteryLowConditionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            std::shared_ptr<rclcpp::Node> node,
            iii_drone::configuration::Configuration::SharedPtr configuration
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<rclcpp::Node> node_;
        iii_drone::configuration::Configuration::SharedPtr configuration_;
        LatestMessageSubscription<std_msgs::msg::Float32> battery_voltage_;

        // Tick-thread state only.
        uint64_t last_voltage_sequence_ = 0;
        rclcpp::Time low_voltage_since_;
        rclcpp::Time last_stale_retry_time_;
        unsigned int stale_failure_count_ = 0;

        // The configured values, read at most once a second: the reactive
        // inspection tree evaluates this condition on every tick, and six
        // configuration lookups per tick were most of its cost. Tuning still
        // takes effect within a second.
        struct ConfiguredSettings {
            bool bypass = false;
            double threshold_v = 14.0;
            double debounce_s = 2.0;
            double timeout_s = 2.0;
            int retry_count = 3;
            double retry_interval_s = 0.2;
        };
        ConfiguredSettings configured_;
        std::chrono::steady_clock::time_point configured_read_at_{};
        const ConfiguredSettings & configuredSettings();

        double parameterOr(const std::string & name, double fallback) const;
        int parameterOr(const std::string & name, int fallback) const;
        bool boolParameterOr(const std::string & name, bool fallback) const;
    };

} // namespace behavior
} // namespace iii_drone
