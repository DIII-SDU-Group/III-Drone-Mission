#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <deque>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/adapters/powerline_adapter.hpp>

#include <iii_drone_core/control/state.hpp>

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>

/*****************************************************************************/
// III-Drone-Mission:

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorators/loop_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Powerline waypoint provider action node.
     */
    class PowerlineWaypointProviderActionNode : public BT::SyncActionNode {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param tf_buffer The tf buffer.
         * @param node The ROS2 node.
         * @param params Parameter bundle
         */
        PowerlineWaypointProviderActionNode(
            const std::string & name, 
            const BT::NodeConfiguration & conf,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            rclcpp::Node * node,
            iii_drone::configuration::ParameterBundle::SharedPtr params
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        tf2_ros::Buffer::SharedPtr tf_buffer_;

        rclcpp::Node * node_;

        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        // std::shared_ptr<std::deque<iii_drone::types::point_t>> applyLinearInterpolation(
        //     std::shared_ptr<std::deque<iii_drone::types::point_t>> points
        // );

    };

}
}