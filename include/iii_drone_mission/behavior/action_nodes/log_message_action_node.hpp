#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class LogMessageActionNode : public BT::SyncActionNode {
    public:
        LogMessageActionNode(
            const std::string & name,
            const BT::NodeConfig& config,
            std::shared_ptr<rclcpp::Node> node
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<rclcpp::Node> node_;

    };

} // namespace behavior
} // namespace iii_drone