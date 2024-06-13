#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/quaternion_stamped.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/behavior/action_nodes/maneuver_action_node.hpp>

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>
#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/adapters/reference_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/fly_to_object.hpp>
#include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Fly to object maneuver action node.
     */
    class FlyToObjectManeuverActionNode : public ManeuverActionNode<iii_drone_interfaces::action::FlyToObject> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         * @param maneuver_reference_client The maneuver reference client.
         * @param parameter_bundle The parameter bundle.
         * @param tf_buffer The TF buffer.
         */
        FlyToObjectManeuverActionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle,
            tf2_ros::Buffer::SharedPtr tf_buffer
        );

        bool setGoal(Goal & goal) override;

        static BT::PortsList providedPorts();

    private:
        iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        /**
         * @brief Gets the final hover reference from the wrapped result.
         * 
         * @param wr The wrapped result.
         * 
         * @return The final hover reference.
         */
        iii_drone::control::Reference getFinalReference(const typename BT::RosActionNode<iii_drone_interfaces::action::FlyToObject>::WrappedResult & wr) const;

    };

} // namespace behavior
}  // namespace iii_drone