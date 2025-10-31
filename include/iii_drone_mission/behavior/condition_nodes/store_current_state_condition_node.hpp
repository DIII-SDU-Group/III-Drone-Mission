#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_odometry.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

// #include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>

#include <iii_drone_core/control/state.hpp>

#include <iii_drone_core/utils/types.hpp>

// #include <iii_drone_core/adapters/single_line_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

// #include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// III-Drone-Mission:

// #include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class StoreCurrentStateConditionNode : public BT::RosTopicSubNode<px4_msgs::msg::VehicleOdometry> {
    public:
        StoreCurrentStateConditionNode(
            const std::string & name,
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onTick(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> & last_msg) override;

    };

} // namespace behavior
} // namespace iii_drone