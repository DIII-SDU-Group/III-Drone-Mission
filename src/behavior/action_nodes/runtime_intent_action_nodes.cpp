/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/runtime_intent_action_nodes.hpp>

using namespace BT;
using namespace iii_drone::behavior;

/*****************************************************************************/
// Helpers
/*****************************************************************************/

namespace {

    void setBoolInBlackboards(
        const NodeConfig & config,
        const BT::Blackboard::Ptr & global_blackboard,
        const std::string & flag_name,
        bool value
    ) {
        if (global_blackboard) {
            global_blackboard->set(flag_name, value);
        }
        if (config.blackboard) {
            config.blackboard->set(flag_name, value);
        }
    }

    void setStringInBlackboards(
        const NodeConfig & config,
        const BT::Blackboard::Ptr & global_blackboard,
        const std::string & flag_name,
        const std::string & value
    ) {
        if (global_blackboard) {
            global_blackboard->set(flag_name, value);
        }
        if (config.blackboard) {
            config.blackboard->set(flag_name, value);
        }
    }

    bool getBoolFromBlackboards(
        const NodeConfig & config,
        const BT::Blackboard::Ptr & global_blackboard,
        const std::string & flag_name,
        bool & value
    ) {
        if (config.blackboard && config.blackboard->get(flag_name, value)) {
            return true;
        }
        if (global_blackboard && global_blackboard->get(flag_name, value)) {
            return true;
        }
        return false;
    }

    bool getStringFromBlackboards(
        const NodeConfig & config,
        const BT::Blackboard::Ptr & global_blackboard,
        const std::string & flag_name,
        std::string & value
    ) {
        if (config.blackboard && config.blackboard->get(flag_name, value)) {
            return true;
        }
        if (global_blackboard && global_blackboard->get(flag_name, value)) {
            return true;
        }
        return false;
    }

} // namespace

/*****************************************************************************/
// ApplyPendingIntentUpdatesActionNode
/*****************************************************************************/

ApplyPendingIntentUpdatesActionNode::ApplyPendingIntentUpdatesActionNode(
    const std::string & name,
    const NodeConfig & config,
    std::shared_ptr<iii_drone::mission::RuntimeIntentBuffer> runtime_intent_buffer,
    BT::Blackboard::Ptr global_blackboard,
    std::shared_ptr<rclcpp::Node> node
) : SyncActionNode(name, config),
    runtime_intent_buffer_(runtime_intent_buffer),
    global_blackboard_(global_blackboard),
    node_(node) { }

PortsList ApplyPendingIntentUpdatesActionNode::providedPorts() {
    return {};
}

NodeStatus ApplyPendingIntentUpdatesActionNode::tick() {
    if (!runtime_intent_buffer_) {
        return NodeStatus::FAILURE;
    }

    const auto updates = runtime_intent_buffer_->Drain();
    for (const auto & update : updates) {
        setBoolInBlackboards(config(), global_blackboard_, update.flag_name, update.value);
        RCLCPP_INFO(
            node_->get_logger(),
            "ApplyPendingIntentUpdatesActionNode::tick(): Applied runtime intent seq=%llu flag=%s value=%s",
            static_cast<unsigned long long>(update.sequence_id),
            update.flag_name.c_str(),
            update.value ? "true" : "false"
        );
    }

    return NodeStatus::SUCCESS;
}

/*****************************************************************************/
// SetBlackboardBoolActionNode
/*****************************************************************************/

SetBlackboardBoolActionNode::SetBlackboardBoolActionNode(
    const std::string & name,
    const NodeConfig & config,
    BT::Blackboard::Ptr global_blackboard
) : SyncActionNode(name, config),
    global_blackboard_(global_blackboard) { }

PortsList SetBlackboardBoolActionNode::providedPorts() {
    return {
        InputPort<std::string>("flag"),
        InputPort<bool>("value", true, "Value to write.")
    };
}

NodeStatus SetBlackboardBoolActionNode::tick() {
    std::string flag;
    bool value = true;

    if (!getInput("flag", flag) || flag.empty()) {
        return NodeStatus::FAILURE;
    }
    getInput("value", value);

    setBoolInBlackboards(config(), global_blackboard_, flag, value);
    return NodeStatus::SUCCESS;
}

/*****************************************************************************/
// BlackboardBoolConditionNode
/*****************************************************************************/

BlackboardBoolConditionNode::BlackboardBoolConditionNode(
    const std::string & name,
    const NodeConfig & config,
    BT::Blackboard::Ptr global_blackboard
) : SyncActionNode(name, config),
    global_blackboard_(global_blackboard) { }

PortsList BlackboardBoolConditionNode::providedPorts() {
    return {
        InputPort<std::string>("flag"),
        InputPort<bool>("expected", true, "Expected bool value.")
    };
}

NodeStatus BlackboardBoolConditionNode::tick() {
    std::string flag;
    bool expected = true;
    bool value = false;

    if (!getInput("flag", flag) || flag.empty()) {
        return NodeStatus::FAILURE;
    }
    getInput("expected", expected);

    const bool found = getBoolFromBlackboards(config(), global_blackboard_, flag, value);
    return found && value == expected ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

/*****************************************************************************/
// SetBlackboardStringActionNode
/*****************************************************************************/

SetBlackboardStringActionNode::SetBlackboardStringActionNode(
    const std::string & name,
    const NodeConfig & config,
    BT::Blackboard::Ptr global_blackboard
) : SyncActionNode(name, config),
    global_blackboard_(global_blackboard) { }

PortsList SetBlackboardStringActionNode::providedPorts() {
    return {
        InputPort<std::string>("flag"),
        InputPort<std::string>("value")
    };
}

NodeStatus SetBlackboardStringActionNode::tick() {
    std::string flag;
    std::string value;

    if (!getInput("flag", flag) || flag.empty() || !getInput("value", value)) {
        return NodeStatus::FAILURE;
    }

    setStringInBlackboards(config(), global_blackboard_, flag, value);
    return NodeStatus::SUCCESS;
}

/*****************************************************************************/
// BlackboardStringEqualsConditionNode
/*****************************************************************************/

BlackboardStringEqualsConditionNode::BlackboardStringEqualsConditionNode(
    const std::string & name,
    const NodeConfig & config,
    BT::Blackboard::Ptr global_blackboard
) : SyncActionNode(name, config),
    global_blackboard_(global_blackboard) { }

PortsList BlackboardStringEqualsConditionNode::providedPorts() {
    return {
        InputPort<std::string>("flag"),
        InputPort<std::string>("expected")
    };
}

NodeStatus BlackboardStringEqualsConditionNode::tick() {
    std::string flag;
    std::string expected;
    std::string value;

    if (!getInput("flag", flag) || flag.empty() || !getInput("expected", expected)) {
        return NodeStatus::FAILURE;
    }

    const bool found = getStringFromBlackboards(config(), global_blackboard_, flag, value);
    return found && value == expected ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}
