#include <iii_drone_mission/mission/profile_restrictions.hpp>

#include <iii_drone_mission/behavior/behavior_node_registry.hpp>
#include <iii_drone_mission/mission/mission_specification.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace iii_drone::mission
{
namespace
{

struct BehaviorNodeRegistry
{
    std::unordered_map<std::string, BT::NodeType> types;
    std::set<std::string> builtins;
};

// Every node the runtime factory can instantiate: the BehaviorTree.CPP
// built-ins and the authoritative III registry, which
// ValidateRuntimeBehaviorFactory() holds the runtime factory to.
const BehaviorNodeRegistry & RuntimeBehaviorNodeRegistry()
{
    static const BehaviorNodeRegistry registry = [] {
        BehaviorNodeRegistry result;
        const BT::BehaviorTreeFactory factory;
        for (const auto & [id, manifest] : factory.manifests()) {
            result.types.emplace(id, manifest.type);
        }
        result.builtins = factory.builtinNodes();
        for (const auto & manifest : iii_drone::behavior::CustomBehaviorNodeManifests()) {
            result.types.emplace(manifest.registration_ID, manifest.type);
        }
        return result;
    }();
    return registry;
}

const std::set<std::string> * FindAllowlist(
    const std::map<std::string, std::set<std::string>> & allowlists,
    const std::string & profile
)
{
    const auto found = allowlists.find(profile);
    return found == allowlists.end() ? nullptr : &found->second;
}

std::string Trim(const std::string & value)
{
    const auto first = value.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\n\r");
    return value.substr(first, last - first + 1);
}

std::string Join(const std::vector<std::string> & values)
{
    std::string joined;
    for (const auto & value : values) {
        joined += (joined.empty() ? "" : ", ") + value;
    }
    return joined;
}

std::string ReadTree(const std::string & path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::runtime_error("cannot read behavior tree " + path);
    }
    std::ostringstream text;
    text << stream.rdbuf();
    return text.str();
}

}  // namespace

std::string ResolveRuntimeProfile(const std::string & parameter_value)
{
    const auto parameter_profile = Trim(parameter_value);
    if (!parameter_profile.empty()) {
        return parameter_profile;
    }
    const char * environment_profile = std::getenv(kRuntimeProfileEnvironment);
    return environment_profile == nullptr ? "" : Trim(environment_profile);
}

std::string NotAvailableInProfileMessage(const std::string & thing, const std::string & profile)
{
    return thing + " is not available in the " + profile + " profile";
}

const std::map<std::string, std::set<std::string>> & CustomOperationAllowlists()
{
    static const std::map<std::string, std::set<std::string>> allowlists = {
        {kOptiTrackProfile, {"fly_to_position", "follow_waypoint_path", "hover"}},
    };
    return allowlists;
}

bool CustomOperationAllowedInProfile(const std::string & operation, const std::string & profile)
{
    const auto * allowlist = FindAllowlist(CustomOperationAllowlists(), profile);
    return allowlist == nullptr || allowlist->count(operation) != 0;
}

const std::map<std::string, std::set<std::string>> & BehaviorNodeAllowlists()
{
    // No cable, gripper, perception or overview providers run in the OptiTrack
    // lab: only flight maneuvers, mode-executor actions and tree plumbing.
    static const std::map<std::string, std::set<std::string>> allowlists = {
        {
            kOptiTrackProfile,
            {
                "ApplyPendingIntentUpdates",
                "BlackboardBool",
                "BlackboardStringEquals",
                "FlyToPosition",
                "FollowWaypointPath",
                "Hover",
                "LogMessage",
                "LoopPoint",
                "ModeExecutorAction",
                "PartitionPointQueue",
                "QueueHasPoints",
                "RetryUntilSuccessfulOnAborted",
                "RosbagRecordingScope",
                "SetBlackboardBool",
                "SetBlackboardString",
                "SplitPointQueue",
                "StartRosbagRecording",
                "StopRosbagRecording",
                "StoreCurrentState",
                "StringEquals",
                "VerifyDisarmed",
                "WaitForPX4Airborne",
            },
        },
    };
    return allowlists;
}

std::vector<std::string> DisallowedBehaviorNodes(const std::string & tree_xml, const std::string & profile)
{
    const auto * allowlist = FindAllowlist(BehaviorNodeAllowlists(), profile);
    if (allowlist == nullptr) {
        return {};
    }

    // BehaviorTree.CPP's own verifier decides which nodes the tree uses: the
    // runtime loader runs it with the full registry, and it rejects a tree
    // exactly when the tree uses a node the registry lacks.
    const auto & registry = RuntimeBehaviorNodeRegistry();
    try {
        BT::VerifyXML(tree_xml, registry.types);
    } catch (const std::exception & exception) {
        throw std::runtime_error(
            std::string("not a behavior tree the runtime can load: ") + exception.what()
        );
    }

    std::vector<std::string> disallowed;
    for (const auto & [id, type] : registry.types) {
        (void)type;
        if (registry.builtins.count(id) != 0 || allowlist->count(id) != 0) {
            continue;
        }
        auto without_node = registry.types;
        without_node.erase(id);
        try {
            BT::VerifyXML(tree_xml, without_node);
        } catch (const std::exception &) {
            disallowed.push_back(id);
        }
    }
    std::sort(disallowed.begin(), disallowed.end());
    return disallowed;
}

void RequireMissionAllowedInProfile(const MissionSpecification & specification, const std::string & profile)
{
    if (FindAllowlist(BehaviorNodeAllowlists(), profile) == nullptr) {
        return;
    }
    for (const auto & entry : specification.entries()) {
        const std::string tree =
            "mission " + specification.catalog_id() + " (mode " + entry.key + ": behavior tree " +
            entry.behavior_tree_logical_name;
        std::vector<std::string> disallowed;
        try {
            disallowed = DisallowedBehaviorNodes(ReadTree(entry.behavior_tree_xml_file), profile);
        } catch (const std::exception & exception) {
            throw std::runtime_error(tree + ") cannot be checked against the " + profile + " profile: " + exception.what());
        }
        if (!disallowed.empty()) {
            throw std::runtime_error(
                NotAvailableInProfileMessage(tree + " uses " + Join(disallowed) + ")", profile)
            );
        }
    }
}

}  // namespace iii_drone::mission
