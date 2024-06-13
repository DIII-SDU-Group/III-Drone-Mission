#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/behavior/port_types.hpp>

#include <iii_drone_core/adapters/single_line_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/target.hpp>

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
     * and selects a target line.
     */
    class SelectTargetLineConditionNode : public BT::RosTopicSubNode<iii_drone_interfaces::msg::Powerline> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         * @param tf_buffer The TF buffer.
         * @param parameter_bundle The parameter bundle.
         */
        SelectTargetLineConditionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) override;

    private:
        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle_;

        typedef enum {
            SELECT_TARGET_LINE_METHOD_RANDOM_ABOVE = 0,
            SELECT_TARGET_LINE_METHOD_CLOSEST_ABOVE = 1,
            SELECT_TARGET_LINE_METHOD_CLOSEST_XY_ABOVE = 2,
            SELECT_TARGET_LINE_METHOD_CLOSEST_Z_ABOVE = 3,
            SELECT_TARGET_LINE_FARTHEST_ABOVE = 4
        } select_target_line_method_t;

        select_target_line_method_t getSelectTargetLineMethod();

    };

} // namespace behavior
}  // namespace iii_drone