#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/behavior/port_types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/pl_mapper_command.hpp>
#include <iii_drone_interfaces/msg/pl_mapper_command.hpp>

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
    class PLMapperCommandActionNode : public BT::RosServiceNode<iii_drone_interfaces::srv::PLMapperCommand> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         */
        PLMapperCommandActionNode(
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