#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/msg/target.hpp>

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
     * @brief Cable takeoff maneuver action node.
     */
    class CableTakeoffManeuverActionNode : public ManeuverActionNode<iii_drone_interfaces::action::CableTakeoff> {
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
        CableTakeoffManeuverActionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            iii_drone::configuration::Configuration::SharedPtr configuration
        );

        bool setManeuverGoal(Goal & goal) override;

        static BT::PortsList providedPorts();

    protected:
        /**
         * Cable takeoff is the continuous successor to the sustained upward
         * reference used to unload the gripper before release.  Preserve the
         * live reference stream so the first takeoff generation is checked
         * against that commanded motion instead of restarting from a
         * momentary on-cable odometry sample.
         */

    private:
        /**
         * @brief The parameter bundle.
         */
        iii_drone::configuration::Configuration::SharedPtr configuration_;

    };

} // namespace behavior
}  // namespace iii_drone
