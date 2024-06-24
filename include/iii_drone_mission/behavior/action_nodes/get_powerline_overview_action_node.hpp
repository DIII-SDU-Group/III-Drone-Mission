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

#include <iii_drone_interfaces/srv/get_powerline_overview.hpp>
#include <iii_drone_interfaces/msg/powerline.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief PL mapper command action node, sends a PL mapper command.
     */
    class GetPowerlineOverviewActionNode : public BT::RosServiceNode<iii_drone_interfaces::srv::GetPowerlineOverview> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         */
        GetPowerlineOverviewActionNode(
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