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

#include <iii_drone_interfaces/msg/powerline.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Powerline subscription condition node, subscribes to the powerline topic
     * and verifies if a powerline has been detected.
     */
    class VerifyPowerlineDetectedConditionNode : public BT::RosTopicSubNode<iii_drone_interfaces::msg::Powerline> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         */
        VerifyPowerlineDetectedConditionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;

    };

} // namespace behavior
}  // namespace iii_drone