#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <mutex>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

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
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_sub_;

        mutable std::mutex mutex_;
        bool has_voltage_ = false;
        float latest_voltage_ = 0.0f;
        rclcpp::Time latest_voltage_receive_time_;
        rclcpp::Time low_voltage_since_;
        rclcpp::Time last_stale_retry_time_;
        unsigned int stale_failure_count_ = 0;

        double parameterOr(const std::string & name, double fallback) const;
        int parameterOr(const std::string & name, int fallback) const;
        bool boolParameterOr(const std::string & name, bool fallback) const;
    };

} // namespace behavior
} // namespace iii_drone
