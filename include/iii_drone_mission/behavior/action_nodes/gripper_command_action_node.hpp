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
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/gripper_command.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class GripperCommandActionNode : public BT::RosServiceNode<iii_drone_interfaces::srv::GripperCommand> {
    public:
        GripperCommandActionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params
        );

        static BT::PortsList providedPorts();

        bool setRequest(Request::SharedPtr & request) override;

        BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;

    };

} // namespace behavior
}  // namespace iii_drone