#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/behavior/action_nodes/maneuver_action_node.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/hover_by_object.hpp>
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
     * @brief Hover by object maneuver action node.
     */
    // class HoverByObjectManeuverActionNode : public BT::RosActionNode<iii_drone_interfaces::action::HoverByObject> {
    class HoverByObjectManeuverActionNode : public ManeuverActionNode<iii_drone_interfaces::action::HoverByObject> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         * @param maneuver_reference_client The maneuver reference client.
         */
        HoverByObjectManeuverActionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
        );

        bool setGoal(Goal & goal) override;

        // BT::NodeStatus onResultReceived(const WrappedResult & wr) override final { return BT::NodeStatus::SUCCESS; }
        // BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override final { return BT::NodeStatus::FAILURE; }
        // BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override final { return BT::NodeStatus::RUNNING; }
    
        static BT::PortsList providedPorts();

    };

} // namespace behavior
}  // namespace iii_drone