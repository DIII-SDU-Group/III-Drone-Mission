/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/mode_executor_action_node.hpp>

#include <iii_drone_core/diagnostics/hil_trace.hpp>

#include <iomanip>
#include <sstream>

using namespace iii_drone::behavior;
using namespace BT;

namespace {

std::string goalUuidToString(const rclcpp_action::GoalUUID & uuid) {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : uuid) {
        stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
}

std::string actionResultCodeToString(rclcpp_action::ResultCode code) {
    switch (code) {
        case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
        case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
        case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
        case rclcpp_action::ResultCode::UNKNOWN: default: return "UNKNOWN";
    }
}

}  // namespace

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
),  node_ptr_(params.nh.lock()),
    action_endpoint_(params.default_port_value) { }

BT::NodeStatus ModeExecutorActionNode::tick() {
    if (this->status() == BT::NodeStatus::IDLE) {
        auto event = iii_drone::diagnostics::HilTrace::event("bt_action_goal_attempt");
        event.text("node", name());
        event.text("endpoint", action_endpoint_);
        event.commit();
    }
    return RosActionNode<iii_drone_interfaces::action::ModeExecutorAction>::tick();
}

void ModeExecutorActionNode::onGoalAccepted() {
    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_goal_accepted");
    event.text("node", name());
    event.text("endpoint", action_endpoint_);
    event.commit();
}

PortsList ModeExecutorActionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<mode_executor_action_request_t>("action_request_type"),
        InputPort<float>("takeoff_altitude"),
        InputPort<bool>("force_disarm", false, "Force disarm")
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

        case MODE_EXECUTOR_ACTION_REQUEST_DISARM:

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorActionNode::setGoal(): Setting action request disarm."
            );

            goal.request = iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_DISARM;

            getInput("force_disarm", goal.force_disarm);

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

    const NodeStatus returned_status =
        wr.code == rclcpp_action::ResultCode::SUCCEEDED ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_result");
    event.text("node", name());
    event.text("endpoint", action_endpoint_);
    event.text("goal_id", goalUuidToString(wr.goal_id));
    event.text("result_code", actionResultCodeToString(wr.code));
    event.text("bt_status", BT::toStr(returned_status, false));
    event.commit();

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

NodeStatus ModeExecutorActionNode::onFailure(
    BT::ActionNodeErrorCode error,
    const std::optional<WrappedResult> & result
) {
    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_failure");
    event.text("node", name());
    event.text("endpoint", action_endpoint_);
    event.text("error", BT::toStr(error));
    event.text("bt_status", "FAILURE");
    if (result.has_value()) {
        event.text("goal_id", goalUuidToString(result->goal_id));
        event.text("result_code", actionResultCodeToString(result->code));
    }
    event.commit();
    return RosActionNode<iii_drone_interfaces::action::ModeExecutorAction>::onFailure(error, result);
}
