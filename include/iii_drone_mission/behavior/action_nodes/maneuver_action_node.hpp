#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <string>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/utils/atomic.hpp>

#include <iii_drone_core/control/reference.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Abstract base class for maneuver action nodes.
     */
    template <typename ActionT>
    class ManeuverActionNode : public BT::RosActionNode<ActionT> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         * @param maneuver_reference_client The maneuver reference client.
         */
        ManeuverActionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
        );

        /**
         * @brief On result received callback. Returns the node status based on the result code.
         * Calls the StopManeuver() method with the maneuver reference client and sets the maneuver running flag to false
         * if the maneuver is running.
         * 
         * @param wr The wrapped result.
         * 
         * @return The node status.
         */
        virtual BT::NodeStatus onResultReceived(const typename BT::RosActionNode<ActionT>::WrappedResult & wr) override final;

        /**
         * @brief On failure callback. Logs the error message and returns the node status.
         * Calls the StopManeuver() method with the maneuver reference client and sets the maneuver running flag to false
         * if the maneuver is running.
         * 
         * @param error The error code.
         * 
         * @return The node status.
         */
        BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override final;

        /**
         * @brief On feedback callback. Returns the node status based on the feedback.
         * Calls the StartManuever() method with the maneuver reference client and sets the maneuver running flag to true
         * if the maneuver is not already running.
         * 
         * @param feedback The feedback.
         * 
         * @return The node status.
         */
        BT::NodeStatus onFeedback(const typename std::shared_ptr<const typename BT::RosActionNode<ActionT>::Feedback> feedback) override final;

        /**
         * @brief On halt callback. Sets the maneuver to not running.
         * 
         * @return void
         */
        void onHalt();

        static BT::PortsList providedManeuverActionNodePorts(BT::PortsList additional_ports={});

    private:
        /**
         * @brief The maneuver reference client.
         */
        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        /**
         * @brief Maneuver running flag.
         */
        iii_drone::utils::Atomic<bool> maneuver_running_ = false;

        /**
         * @brief Sets the maneuver running flag to true and calls the StartManeuver() method with the maneuver reference client
         * if the maneuver is not already running.
         */
        void setManueverRunning();

        /**
         * @brief Sets the maneuver running flag to false and calls the StopManeuver() method with the maneuver reference client
         * if the maneuver is running. If stop_maneuver_after_timeout_ms is provided, the method will call the StopManeuverAfterTimeout() instead.
         * 
         * @param stop_maneuver_after_timeout_ms The timeout in milliseconds.
         */
        void setManueverNotRunning(int stop_maneuver_after_timeout_ms = -1);

        /**
         * @brief Sets the maneuver running flag to false and calls the StopManeuver() method with the maneuver reference client
         * if the maneuver is running with the provided reference. If stop_maneuver_after_timeout_ms is provided, the method will call the StopManeuverAfterTimeout() instead.
         * 
         * @param reference The reference.
         * @param stop_maneuver_after_timeout_ms The timeout in milliseconds.
         */
        void setManueverNotRunning(
            const iii_drone::control::Reference & reference,
            int stop_maneuver_after_timeout_ms = -1
        );

        /**
         * @brief Callback function to get final reference for subsequent hovering given the action result.
         * If provided, the function should return the final reference for subsequent hovering,
         * which will then be used as the hover reference. If not, the current state will be used as the hover reference.
         * 
         * @param wr The wrapped result.
         * 
         * @return The final reference for subsequent hovering.
         */
        std::function<iii_drone::control::Reference(const typename BT::RosActionNode<ActionT>::WrappedResult &)> get_final_reference_callback_ = nullptr;

    protected:
        /**
         * @brief Method to set the get_final_reference_callback_.
         * 
         * @param callback The callback function.
         */
        void setGetFinalReferenceCallback(std::function<iii_drone::control::Reference(const typename BT::RosActionNode<ActionT>::WrappedResult &)> callback);

        /**
         * @brief The name of the maneuver action node.
         */
        const std::string name_;

        /**
         * @brief Node pointer.
         */
        rclcpp::Node::SharedPtr node_ptr_;

    };

} // namespace behavior
}  // namespace iii_drone