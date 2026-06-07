/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/maneuver_action_status_nodes.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// StringEqualsConditionNode
/*****************************************************************************/

StringEqualsConditionNode::StringEqualsConditionNode(
    const std::string & name,
    const NodeConfig & conf
) : SyncActionNode(name, conf) { }

PortsList StringEqualsConditionNode::providedPorts() {
    return {
        InputPort<std::string>("value"),
        InputPort<std::string>("expected")
    };
}

NodeStatus StringEqualsConditionNode::tick() {
    std::string value;
    std::string expected;

    if (!getInput("value", value) || !getInput("expected", expected)) {
        return NodeStatus::FAILURE;
    }

    return value == expected ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

/*****************************************************************************/
// RetryUntilSuccessfulOnAbortedDecorator
/*****************************************************************************/

RetryUntilSuccessfulOnAbortedDecorator::RetryUntilSuccessfulOnAbortedDecorator(
    const std::string & name,
    const NodeConfig & conf
) : DecoratorNode(name, conf) { }

PortsList RetryUntilSuccessfulOnAbortedDecorator::providedPorts() {
    return {
        InputPort<int>("num_attempts", 1, "Maximum number of aborted physical attempts."),
        InputPort<std::string>("terminal_state", "", "Terminal state of the guarded action.")
    };
}

NodeStatus RetryUntilSuccessfulOnAbortedDecorator::tick() {
    int max_attempts = 1;
    getInput("num_attempts", max_attempts);

    setStatus(NodeStatus::RUNNING);
    const NodeStatus child_status = child_node_->executeTick();

    std::string terminal_state;
    getInput("terminal_state", terminal_state);

    if (terminal_state == "ACCEPTED") {
        guarded_action_accepted_ = true;
    }

    if (child_status == NodeStatus::SUCCESS) {
        attempts_ = 0;
        guarded_action_accepted_ = false;
        resetChild();
        return NodeStatus::SUCCESS;
    }

    if (child_status == NodeStatus::RUNNING) {
        return NodeStatus::RUNNING;
    }

    resetChild();

    const bool retryable_abort =
        guarded_action_accepted_ &&
        (terminal_state == "ABORTED" || terminal_state == "ACTION_ABORTED");

    guarded_action_accepted_ = false;

    if (!retryable_abort) {
        attempts_ = 0;
        return NodeStatus::FAILURE;
    }

    ++attempts_;
    if (attempts_ >= max_attempts) {
        attempts_ = 0;
        return NodeStatus::FAILURE;
    }

    return NodeStatus::RUNNING;
}

void RetryUntilSuccessfulOnAbortedDecorator::halt() {
    attempts_ = 0;
    guarded_action_accepted_ = false;
    DecoratorNode::halt();
}
