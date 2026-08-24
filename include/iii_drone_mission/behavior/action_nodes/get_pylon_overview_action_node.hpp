#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/pylon_overview.hpp>
#include <iii_drone_interfaces/srv/get_pylon_overview.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class GetPylonOverviewActionNode : public BT::RosServiceNode<iii_drone_interfaces::srv::GetPylonOverview> {
    public:
        GetPylonOverviewActionNode(
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
} // namespace iii_drone
