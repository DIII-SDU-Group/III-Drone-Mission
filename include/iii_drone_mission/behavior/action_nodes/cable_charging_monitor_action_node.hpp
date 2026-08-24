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

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/charger_status.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class CableChargingMonitorActionNode : public BT::StatefulActionNode {
    public:
        CableChargingMonitorActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            std::shared_ptr<rclcpp::Node> node,
            iii_drone::configuration::Configuration::SharedPtr configuration,
            BT::Blackboard::Ptr global_blackboard
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        std::shared_ptr<rclcpp::Node> node_;
        iii_drone::configuration::Configuration::SharedPtr configuration_;
        BT::Blackboard::Ptr global_blackboard_;
        rclcpp::Subscription<iii_drone_interfaces::msg::ChargerStatus>::SharedPtr charger_status_sub_;

        mutable std::mutex mutex_;
        bool has_status_ = false;
        uint8_t latest_charger_status_ = iii_drone_interfaces::msg::ChargerStatus::CHARGER_STATUS_DISABLED;
        rclcpp::Time start_time_;

        double parameterOr(const std::string & name, double fallback) const;
        bool boolParameterOr(const std::string & name, bool fallback) const;
        bool blackboardBool(const std::string & key, bool fallback) const;
        BT::NodeStatus evaluateChargingState();
    };

} // namespace behavior
} // namespace iii_drone
