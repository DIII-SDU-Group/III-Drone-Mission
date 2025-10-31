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

// #include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/adapters/single_line_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
// #include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class GetGripperAlignmentYawConditionNode : public BT::RosTopicSubNode<iii_drone_interfaces::msg::Powerline> {
    public:
        GetGripperAlignmentYawConditionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) override;

    private:
        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    };

} // namespace behavior
}  // namespace iii_drone