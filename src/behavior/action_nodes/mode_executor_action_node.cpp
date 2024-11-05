/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/mode_executor_action_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ModeExecutorActionNode::ModeExecutorActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosActionNode<iii_drone_interfaces::action::ModeExecutorAction>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh) { }

PortsList ModeExecutorActionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<mode_executor_action_request_t>("action_request_type"),
        InputPort<float>("takeoff_altitude")
    });

}

bool ModeExecutorActionNode::setGoal(Goal & goal) {

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "ModeExecutorActionNode::setGoal(): Setting goal."
    );

    mode_executor_action_request_t art;

    if (!getInput("action_request_type", art)) {

        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ModeExecutorActionNode::setGoal(): No action request type provided."
        );

        return false;

    }

    switch(art) {
        
        case MODE_EXECUTOR_ACTION_REQUEST_TAKEOFF:

            if (!getInput("takeoff_altitude", goal.takeoff_altitude)) {

                RCLCPP_ERROR(
                    node_ptr_->get_logger(),
                    "ModeExecutorActionNode::setGoal(): No takeoff altitude provided for action request type takeoff."
                );

                return false;

            }

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorActionNode::setGoal(): Setting action request takeoff."
            );

            goal.request = iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_TAKEOFF;

            return true;

        case MODE_EXECUTOR_ACTION_REQUEST_LAND:

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorActionNode::setGoal(): Setting action request land."
            );

            goal.request = iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_LAND;

            return true;

        case MODE_EXECUTOR_ACTION_REQUEST_ARM:

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorActionNode::setGoal(): Setting action request arm."
            );

            goal.request = iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_ARM;

            return true;

        default:

            RCLCPP_ERROR(
                node_ptr_->get_logger(),
                "ModeExecutorActionNode::setGoal(): Invalid action request type."
            );

            return false;

    }

    return false;

}

NodeStatus ModeExecutorActionNode::onResultReceived(const WrappedResult & wr) {

    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ModeExecutorActionNode::onResultReceived(): Success"
        );
        return NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ModeExecutorActionNode::onResultReceived(): Failure"
        );
        return NodeStatus::FAILURE;
    }

}