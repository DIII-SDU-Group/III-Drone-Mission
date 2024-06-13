#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/behavior/action_nodes/maneuver_action_node.hpp>

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>
#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/adapters/reference_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// ROS2:

#include <geometry_msgs/msg/point.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Fly to position maneuver action node.
     */
    class FlyToPositionManeuverActionNode : public ManeuverActionNode<iii_drone_interfaces::action::FlyToPosition> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         * @param maneuver_reference_client The maneuver reference client.
         */
        FlyToPositionManeuverActionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
        );

        bool setGoal(Goal & goal) override;

        static BT::PortsList providedPorts();

    private:
        /**
         * @brief Gets the final hover reference from the wrapped result.
         * 
         * @param wr The wrapped result.
         * 
         * @return The final hover reference.
         */
        iii_drone::control::Reference getFinalReference(const typename BT::RosActionNode<iii_drone_interfaces::action::FlyToPosition>::WrappedResult & wr) const;

    };

} // namespace behavior
}  // namespace iii_drone