/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/inspection_waypoint_progress_nodes.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::types;
using namespace BT;

/*****************************************************************************/
// Helpers
/*****************************************************************************/

namespace {

    constexpr int kInspectionLoopWaypointCount = 8;

    std::string prefixFromInput(BT::TreeNode & node) {
        std::string prefix = "inspection_demo";
        node.getInput("prefix", prefix);
        return prefix;
    }

    std::string queueKey(const std::string & prefix) {
        return prefix + ".active_waypoints";
    }

    std::string indexKey(const std::string & prefix) {
        return prefix + ".current_waypoint_index";
    }

    std::string initializedKey(const std::string & prefix) {
        return prefix + ".waypoints_initialized";
    }

    std::string loopStartKey(const std::string & prefix) {
        return prefix + ".loop_start_index";
    }

    template <typename T>
    bool getFromBlackboards(
        const BT::NodeConfig & config,
        const BT::Blackboard::Ptr & global_blackboard,
        const std::string & key,
        T & value
    ) {
        if (config.blackboard && config.blackboard->get(key, value)) {
            return true;
        }
        if (global_blackboard && global_blackboard->get(key, value)) {
            return true;
        }
        return false;
    }

    template <typename T>
    void setInBlackboards(
        const BT::NodeConfig & config,
        const BT::Blackboard::Ptr & global_blackboard,
        const std::string & key,
        const T & value
    ) {
        if (global_blackboard) {
            global_blackboard->set(key, value);
        }
        if (config.blackboard) {
            config.blackboard->set(key, value);
        }
    }

} // namespace

/*****************************************************************************/
// Progression
/*****************************************************************************/

std::optional<int> iii_drone::behavior::NextInspectionWaypointIndex(
    int current_index,
    std::size_t waypoint_count,
    int loop_start_index
) {
    if (
        waypoint_count == 0 ||
        current_index < 0 ||
        current_index >= static_cast<int>(waypoint_count) ||
        loop_start_index < 0 ||
        loop_start_index >= static_cast<int>(waypoint_count)
    ) {
        return std::nullopt;
    }

    const int next_index = current_index + 1;
    return next_index < static_cast<int>(waypoint_count)
        ? next_index
        : loop_start_index;
}

std::optional<bool> iii_drone::behavior::InspectionWaypointShouldBlendToNext(
    int current_index,
    std::size_t waypoint_count,
    int loop_start_index
) {
    if (
        waypoint_count == 0 ||
        current_index < 0 ||
        current_index >= static_cast<int>(waypoint_count) ||
        loop_start_index < 0 ||
        loop_start_index >= static_cast<int>(waypoint_count) ||
        waypoint_count - static_cast<std::size_t>(loop_start_index) !=
            kInspectionLoopWaypointCount
    ) {
        return std::nullopt;
    }

    if (current_index < loop_start_index) {
        return current_index != loop_start_index - 1;
    }

    return true;
}

/*****************************************************************************/
// InitializeInspectionWaypointsActionNode
/*****************************************************************************/

InitializeInspectionWaypointsActionNode::InitializeInspectionWaypointsActionNode(
    const std::string & name,
    const NodeConfig & config,
    BT::Blackboard::Ptr global_blackboard
) : SyncActionNode(name, config),
    global_blackboard_(global_blackboard) { }

PortsList InitializeInspectionWaypointsActionNode::providedPorts() {
    return {
        InputPort<SharedQueue<point_t>>("waypoints"),
        InputPort<int>("loop_start_index", 0, "First waypoint of the repeating inspection loop."),
        InputPort<std::string>("prefix", "inspection_demo", "Blackboard key prefix."),
        InputPort<bool>("reset", false, "Force replacement of the persisted waypoint queue.")
    };
}

NodeStatus InitializeInspectionWaypointsActionNode::tick() {
    SharedQueue<point_t> waypoints;
    if (!getInput("waypoints", waypoints) || !waypoints) {
        return NodeStatus::FAILURE;
    }

    const std::string prefix = prefixFromInput(*this);
    bool reset = false;
    getInput("reset", reset);

    bool initialized = false;
    getFromBlackboards(config(), global_blackboard_, initializedKey(prefix), initialized);

    int loop_start_index = 0;
    if (!getInput("loop_start_index", loop_start_index)) {
        return NodeStatus::FAILURE;
    }
    if (
        loop_start_index < 0 ||
        loop_start_index >= static_cast<int>(waypoints->size())
    ) {
        return NodeStatus::FAILURE;
    }

    int index = 0;
    getFromBlackboards(config(), global_blackboard_, indexKey(prefix), index);

    SharedQueue<point_t> existing_waypoints;
    const bool has_existing_queue =
        getFromBlackboards(config(), global_blackboard_, queueKey(prefix), existing_waypoints) && existing_waypoints;
    int existing_loop_start_index = 0;
    const bool has_existing_loop_start = getFromBlackboards(
        config(),
        global_blackboard_,
        loopStartKey(prefix),
        existing_loop_start_index
    );

    const bool exhausted = has_existing_queue && index >= static_cast<int>(existing_waypoints->size());
    if (
        !reset &&
        initialized &&
        has_existing_queue &&
        has_existing_loop_start &&
        !exhausted
    ) {
        return NodeStatus::SUCCESS;
    }

    auto persisted_waypoints = std::make_shared<std::deque<point_t>>(*waypoints);
    setInBlackboards(config(), global_blackboard_, queueKey(prefix), persisted_waypoints);
    setInBlackboards(config(), global_blackboard_, indexKey(prefix), 0);
    setInBlackboards(config(), global_blackboard_, loopStartKey(prefix), loop_start_index);
    setInBlackboards(config(), global_blackboard_, initializedKey(prefix), true);

    return NodeStatus::SUCCESS;
}

/*****************************************************************************/
// GetCurrentInspectionWaypointActionNode
/*****************************************************************************/

GetCurrentInspectionWaypointActionNode::GetCurrentInspectionWaypointActionNode(
    const std::string & name,
    const NodeConfig & config,
    BT::Blackboard::Ptr global_blackboard
) : SyncActionNode(name, config),
    global_blackboard_(global_blackboard) { }

PortsList GetCurrentInspectionWaypointActionNode::providedPorts() {
    return {
        InputPort<std::string>("prefix", "inspection_demo", "Blackboard key prefix."),
        OutputPort<point_t>("waypoint"),
        OutputPort<int>("waypoint_index"),
        OutputPort<int>("waypoint_count"),
        OutputPort<bool>("blend_to_next")
    };
}

NodeStatus GetCurrentInspectionWaypointActionNode::tick() {
    const std::string prefix = prefixFromInput(*this);

    SharedQueue<point_t> waypoints;
    int index = 0;
    if (!getFromBlackboards(config(), global_blackboard_, queueKey(prefix), waypoints) || !waypoints) {
        return NodeStatus::FAILURE;
    }
    getFromBlackboards(config(), global_blackboard_, indexKey(prefix), index);

    if (index < 0 || index >= static_cast<int>(waypoints->size())) {
        return NodeStatus::FAILURE;
    }

    int loop_start_index = 0;
    if (!getFromBlackboards(
        config(),
        global_blackboard_,
        loopStartKey(prefix),
        loop_start_index
    )) {
        return NodeStatus::FAILURE;
    }
    const auto blend_to_next = InspectionWaypointShouldBlendToNext(
        index,
        waypoints->size(),
        loop_start_index
    );
    if (!blend_to_next) {
        return NodeStatus::FAILURE;
    }

    setOutput("waypoint", waypoints->at(static_cast<size_t>(index)));
    setOutput("waypoint_index", index);
    setOutput("waypoint_count", static_cast<int>(waypoints->size()));
    setOutput("blend_to_next", *blend_to_next);
    return NodeStatus::SUCCESS;
}

/*****************************************************************************/
// AdvanceInspectionWaypointActionNode
/*****************************************************************************/

AdvanceInspectionWaypointActionNode::AdvanceInspectionWaypointActionNode(
    const std::string & name,
    const NodeConfig & config,
    BT::Blackboard::Ptr global_blackboard
) : SyncActionNode(name, config),
    global_blackboard_(global_blackboard) { }

PortsList AdvanceInspectionWaypointActionNode::providedPorts() {
    return {
        InputPort<std::string>("prefix", "inspection_demo", "Blackboard key prefix.")
    };
}

NodeStatus AdvanceInspectionWaypointActionNode::tick() {
    const std::string prefix = prefixFromInput(*this);

    SharedQueue<point_t> waypoints;
    int index = 0;
    if (!getFromBlackboards(config(), global_blackboard_, queueKey(prefix), waypoints) || !waypoints) {
        return NodeStatus::FAILURE;
    }
    getFromBlackboards(config(), global_blackboard_, indexKey(prefix), index);
    int loop_start_index = 0;
    getFromBlackboards(
        config(),
        global_blackboard_,
        loopStartKey(prefix),
        loop_start_index
    );
    if (
        loop_start_index < 0 ||
        loop_start_index >= static_cast<int>(waypoints->size())
    ) {
        return NodeStatus::FAILURE;
    }

    const auto next_index = NextInspectionWaypointIndex(
        index,
        waypoints->size(),
        loop_start_index
    );
    if (!next_index) {
        return NodeStatus::FAILURE;
    }
    index = *next_index;
    setInBlackboards(config(), global_blackboard_, indexKey(prefix), index);
    setInBlackboards(config(), global_blackboard_, initializedKey(prefix), true);

    return NodeStatus::SUCCESS;
}
