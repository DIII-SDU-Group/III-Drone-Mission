/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/split_point_queue_action_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::types;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

SplitPointQueueActionNode::SplitPointQueueActionNode(
    const std::string & name,
    const NodeConfig & config
) : SyncActionNode(name, config) { }

PortsList SplitPointQueueActionNode::providedPorts() {
    return {
        InputPort<SharedQueue<point_t>>("queue"),
        OutputPort<SharedQueue<point_t>>("prefix"),
        OutputPort<point_t>("last"),
        OutputPort<bool>("has_prefix")
    };
}

NodeStatus SplitPointQueueActionNode::tick() {
    SharedQueue<point_t> queue;
    if (!getInput("queue", queue) || !queue || queue->empty()) {
        return NodeStatus::FAILURE;
    }

    auto prefix = std::make_shared<std::deque<point_t>>(*queue);
    const point_t last = prefix->back();
    prefix->pop_back();

    setOutput("prefix", prefix);
    setOutput("last", last);
    setOutput("has_prefix", !prefix->empty());

    return NodeStatus::SUCCESS;
}
