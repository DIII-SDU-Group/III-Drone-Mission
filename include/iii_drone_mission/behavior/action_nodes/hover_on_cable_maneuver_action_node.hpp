#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/hover_on_cable.hpp>
#include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp>

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
     * @brief Hover on cable maneuver action node.
     */
    class HoverOnCableManeuverActionNode : public ManeuverActionNode<iii_drone_interfaces::action::HoverOnCable> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         * @param maneuver_reference_client The maneuver reference client.
         * @param parameter_bundle The parameter bundle.
         */
        HoverOnCableManeuverActionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle
        );

        bool setGoal(Goal & goal) override;

        static BT::PortsList providedPorts();

    private:
        iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle_;

    };

} // namespace behavior
}  // namespace iii_drone