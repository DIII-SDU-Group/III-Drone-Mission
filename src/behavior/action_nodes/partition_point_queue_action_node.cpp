#include <deque>
#include <cmath>

#include <iii_drone_mission/behavior/action_nodes/partition_point_queue_action_node.hpp>

namespace iii_drone::behavior {

WaypointRouteBoundaryIndices ComputeOutsideBoundaryIndices(
    std::size_t shared_route_size,
    std::size_t shared_route_boundary_index,
    bool boundary_is_laterally_outside
) {
    if (!boundary_is_laterally_outside || shared_route_size < 3 ||
        shared_route_boundary_index == 0 ||
        shared_route_boundary_index + 1 >= shared_route_size) {
        return {};
    }
    return {
        static_cast<int>(shared_route_boundary_index - 1),
        static_cast<int>(shared_route_size - 1 - shared_route_boundary_index),
    };
}

bool IsOutsideCorridorBeyondCompletionTolerance(
    double lateral_distance_m,
    double inside_corridor_threshold_m,
    double completion_tolerance_m
) {
    return std::isfinite(lateral_distance_m) &&
        std::isfinite(inside_corridor_threshold_m) &&
        std::isfinite(completion_tolerance_m) &&
        lateral_distance_m > inside_corridor_threshold_m + completion_tolerance_m;
}

std::optional<WaypointQueuePartition> PartitionWaypointQueue(
    const BT::SharedQueue<types::point_t> & input,
    int boundary_index
) {
    if (!input || boundary_index < -1 ||
        (boundary_index >= 0 && static_cast<std::size_t>(boundary_index) >= input->size())) {
        return std::nullopt;
    }

    WaypointQueuePartition result{
        std::make_shared<std::deque<types::point_t>>(),
        std::make_shared<std::deque<types::point_t>>(),
        std::make_shared<std::deque<types::point_t>>(),
        std::nullopt,
    };
    if (boundary_index < 0) {
        *result.after = *input;
        return result;
    }

    result.before->insert(result.before->end(), input->begin(), input->begin() + boundary_index);
    result.through->insert(
        result.through->end(), input->begin(), input->begin() + boundary_index + 1
    );
    result.after->insert(result.after->end(), input->begin() + boundary_index + 1, input->end());
    result.boundary = input->at(static_cast<std::size_t>(boundary_index));
    return result;
}

void RegisterWaypointQueueNodes(BT::BehaviorTreeFactory & factory) {
    factory.registerNodeType<PartitionPointQueueActionNode>("PartitionPointQueue");
    factory.registerNodeType<QueueHasPointsConditionNode>("QueueHasPoints");
}

PartitionPointQueueActionNode::PartitionPointQueueActionNode(
    const std::string & name, const BT::NodeConfig & config
) : BT::SyncActionNode(name, config) {}

BT::PortsList PartitionPointQueueActionNode::providedPorts() {
    return {
        BT::InputPort<BT::SharedQueue<iii_drone::types::point_t>>("queue"),
        BT::InputPort<int>("boundary_index"),
        BT::OutputPort<BT::SharedQueue<iii_drone::types::point_t>>("before"),
        BT::OutputPort<BT::SharedQueue<iii_drone::types::point_t>>("through"),
        BT::OutputPort<BT::SharedQueue<iii_drone::types::point_t>>("after"),
        BT::OutputPort<BT::SharedQueue<iii_drone::types::point_t>>("boundary"),
        BT::OutputPort<bool>("has_boundary"),
    };
}

BT::NodeStatus PartitionPointQueueActionNode::tick() {
    using iii_drone::types::point_t;
    using BT::SharedQueue;
    SharedQueue<point_t> input;
    int boundary_index = -1;
    if (
        !getInput("queue", input) || !input ||
        !getInput("boundary_index", boundary_index) || boundary_index < -1
    ) {
        return BT::NodeStatus::FAILURE;
    }

    auto partition = PartitionWaypointQueue(input, boundary_index);
    if (!partition) {
        return BT::NodeStatus::FAILURE;
    }

    auto boundary = std::make_shared<std::deque<point_t>>();
    if (partition->boundary) {
        boundary->push_back(*partition->boundary);
    }
    setOutput("before", partition->before);
    setOutput("through", partition->through);
    setOutput("after", partition->after);
    setOutput("boundary", boundary);
    setOutput("has_boundary", partition->boundary.has_value());
    return BT::NodeStatus::SUCCESS;
}

QueueHasPointsConditionNode::QueueHasPointsConditionNode(
    const std::string & name, const BT::NodeConfig & config
) : BT::ConditionNode(name, config) {}

BT::PortsList QueueHasPointsConditionNode::providedPorts() {
    return {BT::InputPort<BT::SharedQueue<iii_drone::types::point_t>>("queue")};
}

BT::NodeStatus QueueHasPointsConditionNode::tick() {
    BT::SharedQueue<iii_drone::types::point_t> queue;
    if (!getInput("queue", queue)) {
        return BT::NodeStatus::FAILURE;
    }
    return queue && !queue->empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace iii_drone::behavior
