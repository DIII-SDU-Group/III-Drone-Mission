#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_mission/behavior/port_types.hpp>

namespace iii_drone::behavior {

struct WaypointQueuePartition {
    BT::SharedQueue<types::point_t> before;
    BT::SharedQueue<types::point_t> through;
    BT::SharedQueue<types::point_t> after;
    std::optional<types::point_t> boundary;
};

struct WaypointRouteBoundaryIndices {
    int departure = -1;
    int return_route = -1;
};

/// Expose indices only for a confirmed outside boundary with route on both sides.
WaypointRouteBoundaryIndices ComputeOutsideBoundaryIndices(
    std::size_t shared_route_size,
    std::size_t shared_route_boundary_index,
    bool boundary_is_laterally_outside
);

/// A FlyToPosition arrival anywhere within tolerance must remain outside the corridor.
bool IsOutsideCorridorBeyondCompletionTolerance(
    double lateral_distance_m,
    double inside_corridor_threshold_m,
    double completion_tolerance_m
);

/// A negative boundary index means the route has no outside-corridor waypoint.
std::optional<WaypointQueuePartition> PartitionWaypointQueue(
    const BT::SharedQueue<types::point_t> & queue,
    int boundary_index
);

/// Register the queue nodes used by waypoint corridor trees.
void RegisterWaypointQueueNodes(BT::BehaviorTreeFactory & factory);

/// Partitions a waypoint queue around an explicit route boundary index.
class PartitionPointQueueActionNode : public BT::SyncActionNode {
public:
    PartitionPointQueueActionNode(const std::string & name, const BT::NodeConfig & config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

/// Returns SUCCESS only when the supplied waypoint queue contains a point.
class QueueHasPointsConditionNode : public BT::ConditionNode {
public:
    QueueHasPointsConditionNode(const std::string & name, const BT::NodeConfig & config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace iii_drone::behavior
