/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/gripper_command_action_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GripperCommandActionNode::GripperCommandActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::GripperCommand>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh.lock()) { }

PortsList GripperCommandActionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<gripper_command_t>("gripper_command"),
        OutputPort<gripper_command_response_t>("gripper_command_response")
    });

}

bool GripperCommandActionNode::setRequest(Request::SharedPtr & request) {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "GripperCommandActionNode::setRequest(): Setting request"
    );

    gripper_command_t gripper_command;

    getInput("gripper_command", gripper_command);
    
    switch (gripper_command) {

        case GRIPPER_COMMAND_OPEN:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::setRequest(): Opening gripper"
            );
            request->gripper_command = request->GRIPPER_COMMAND_OPEN;
            break;

        case GRIPPER_COMMAND_CLOSE:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::setRequest(): Closing gripper"
            );
            request->gripper_command = request->GRIPPER_COMMAND_CLOSE;
            break;

        default:
            RCLCPP_ERROR(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::setRequest(): Invalid gripper command"
            );
            return false;

    }

    return true;

}

NodeStatus GripperCommandActionNode::onResponseReceived(const Response::SharedPtr & response) {

    switch (response->gripper_command_response) {

        case iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_SUCCESS:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::onResponseReceived(): Gripper command success"
            );
            setOutput("gripper_command_response", GRIPPER_COMMAND_RESPONSE_SUCCESS);
            return NodeStatus::SUCCESS;

        case iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_INVALID_COMMAND:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::onResponseReceived(): Gripper command invalid"
            );
            setOutput("gripper_command_response", GRIPPER_COMMAND_RESPONSE_INVALID_COMMAND);
            return NodeStatus::FAILURE;

        case iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_ERROR:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::onResponseReceived(): Gripper command error"
            );
            setOutput("gripper_command_response", GRIPPER_COMMAND_RESPONSE_ERROR);
            return NodeStatus::FAILURE;

        case iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_TIMEOUT:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::onResponseReceived(): Gripper command timeout"
            );
            setOutput("gripper_command_response", GRIPPER_COMMAND_RESPONSE_TIMEOUT);
            return NodeStatus::FAILURE;

        default:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "GripperCommandActionNode::onResponseReceived(): Gripper command unknown response"
            );

            return NodeStatus::FAILURE;

    }

}