/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/pl_mapper_command_action_node.hpp>

#include <iii_drone_core/diagnostics/hil_trace.hpp>
#include <iii_drone_mission/mission/mission_exit.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PLMapperCommandActionNode::PLMapperCommandActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::PLMapperCommand>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh.lock()),
    service_endpoint_(params.default_port_value) { }

PortsList PLMapperCommandActionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<pl_mapper_command_t>("pl_mapper_command"),
        InputPort<bool>("reset"),
        OutputPort<pl_mapper_ack_t>("pl_mapper_ack")
    });

}

BT::NodeStatus PLMapperCommandActionNode::tick() {
    const bool dispatching = status() == BT::NodeStatus::IDLE;
    return iii_drone::mission::guardMissionDispatch(
        dispatching,
        [this]() { return RosServiceNode<iii_drone_interfaces::srv::PLMapperCommand>::tick(); },
        [this]() {
            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::tick(): %s: Mission Exit, not sending PL mapper command",
                name().c_str()
            );
            return BT::NodeStatus::FAILURE;
        }
    );
}

bool PLMapperCommandActionNode::setRequest(Request::SharedPtr & request) {

    auto event = iii_drone::diagnostics::HilTrace::event("bt_service_request_attempt");
    event.text("node", name());
    event.text("endpoint", service_endpoint_);
    event.commit();

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "PLMapperCommandActionNode::setRequest(): Setting request"
    );

    pl_mapper_command_t pl_mapper_command;
    bool reset;

    getInput("pl_mapper_command", pl_mapper_command);
    getInput("reset", reset);
    
    switch (pl_mapper_command) {

        case PL_MAPPER_COMMAND_START:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::setRequest(): Starting PL Mapper"
            );
            request->pl_mapper_cmd.command = iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_START;
            break;

        case PL_MAPPER_COMMAND_STOP:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::setRequest(): Stopping PL Mapper"
            );
            request->pl_mapper_cmd.command = iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_STOP;
            break;

        case PL_MAPPER_COMMAND_PAUSE:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::setRequest(): Pausing PL Mapper"
            );
            request->pl_mapper_cmd.command = iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_PAUSE;
            break;

        case PL_MAPPER_COMMAND_FREEZE:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::setRequest(): Freezing PL Mapper"
            );
            request->pl_mapper_cmd.command = iii_drone_interfaces::msg::PLMapperCommand::PL_MAPPER_CMD_FREEZE;
            break;

        default:
            return false;

    }

    request->pl_mapper_cmd.reset = reset;
    // Mission Exit returns the mapper to the state a completed Leave Cable
    // leaves it in when the mission started, paused or froze it.
    iii_drone::mission::MissionControl::Process().RecordPlMapperCommand(
        request->pl_mapper_cmd.command);

    return true;

}

NodeStatus PLMapperCommandActionNode::onResponseReceived(const Response::SharedPtr & response) {

    const bool success = response->pl_mapper_ack ==
        iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS;
    auto event = iii_drone::diagnostics::HilTrace::event("bt_service_response");
    event.text("node", name());
    event.text("endpoint", service_endpoint_);
    event.boolean("success", success);
    event.text("bt_status", success ? "SUCCESS" : "FAILURE");
    event.number("response_code", response->pl_mapper_ack);
    event.commit();

    switch (response->pl_mapper_ack) {

        case iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::onResponseReceived(): PL Mapper command successful"
            );
            setOutput("pl_mapper_ack", PL_MAPPER_ACK_SUCCESS);
            return NodeStatus::SUCCESS;

        case iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_INVALID_CMD:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::onResponseReceived(): PL Mapper command invalid"
            );
            setOutput("pl_mapper_ack", PL_MAPPER_ACK_INVALID_COMMAND);
            return NodeStatus::FAILURE;

        default:
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "PLMapperCommandActionNode::onResponseReceived(): PL Mapper command unknown error"
            );
            setOutput("pl_mapper_ack", PL_MAPPER_ACK_UNKNOWN_ERROR);
            return NodeStatus::FAILURE;

    }

}
