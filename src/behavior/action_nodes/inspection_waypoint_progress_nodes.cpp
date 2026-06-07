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

    int index = 0;
    getFromBlackboards(config(), global_blackboard_, indexKey(prefix), index);

    SharedQueue<point_t> existing_waypoints;
    const bool has_existing_queue =
        getFromBlackboards(config(), global_blackboard_, queueKey(prefix), existing_waypoints) && existing_waypoints;

    const bool exhausted = has_existing_queue && index >= static_cast<int>(existing_waypoints->size());
    if (!reset && initialized && has_existing_queue && !exhausted) {
        return NodeStatus::SUCCESS;
    }

    auto persisted_waypoints = std::make_shared<std::deque<point_t>>(*waypoints);
    setInBlackboards(config(), global_blackboard_, queueKey(prefix), persisted_waypoints);
    setInBlackboards(config(), global_blackboard_, indexKey(prefix), 0);
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
        OutputPort<bool>(
            "has_next_waypoint",
            "True when the next inspection step is another FTP waypoint in the same route pass."
        )
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

    setOutput("waypoint", waypoints->at(static_cast<size_t>(index)));
    setOutput("waypoint_index", index);
    setOutput("waypoint_count", static_cast<int>(waypoints->size()));
    setOutput("has_next_waypoint", index + 1 < static_cast<int>(waypoints->size()));
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

    ++index;
    if (index >= static_cast<int>(waypoints->size())) {
        index = 0;
    }
    setInBlackboards(config(), global_blackboard_, indexKey(prefix), index);
    setInBlackboards(config(), global_blackboard_, initializedKey(prefix), true);

    return NodeStatus::SUCCESS;
}
