#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/adapters/single_line_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/condition_node.h>
// #include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class TargetProvider : public BT::ConditionNode {
    public:

        TargetProvider(
            const std::string & name,
            const BT::NodeConfig & config,
            rclcpp::Node * node,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle
        );

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts();

    private:
        rclcpp::Node * node_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle_;

        bool getFlyToCableTarget(
            int target_id,
            iii_drone_interfaces::msg::Target & target
        );

        bool getHoverByCableTarget(
            int target_id,
            iii_drone_interfaces::msg::Target & target
        );

    };

} // namespace behavior
} // namespace iii_drone