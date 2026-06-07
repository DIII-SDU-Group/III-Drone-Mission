#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <deque>
#include <memory>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/pylon_overview.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class PhaseWaypointProviderActionNode : public BT::SyncActionNode {
    public:
        PhaseWaypointProviderActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            rclcpp::Node * node,
            iii_drone::configuration::Configuration::SharedPtr configuration
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        struct InspectionConductor {
            iii_drone::types::point_t point;
            double cross_sign = 0.0;
            bool top = false;
        };

        rclcpp::Node * node_;
        iii_drone::configuration::Configuration::SharedPtr configuration_;

        double parameterOr(const std::string & name, double fallback) const;
    };

} // namespace behavior
} // namespace iii_drone
