#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

namespace iii_drone::mission
{

class MissionSpecification;

/// The OptiTrack-lab profile: flight basics without cable, gripper or perception.
inline constexpr char kOptiTrackProfile[] = "opti_track";

/// ROS parameter that sets a node's runtime profile.
inline constexpr char kRuntimeProfileParameter[] = "iii_runtime_profile";

/// Supervision sets this to the boot profile for every entity process.
inline constexpr char kRuntimeProfileEnvironment[] = "III_SYSTEM_PROFILE";

/**
 * The runtime profile: a non-empty iii_runtime_profile parameter, else
 * III_SYSTEM_PROFILE, else empty. An empty or unknown profile restricts
 * nothing.
 */
std::string ResolveRuntimeProfile(const std::string & parameter_value);

/// "<thing> is not available in the <profile> profile".
std::string NotAvailableInProfileMessage(const std::string & thing, const std::string & profile);

/// Custom operations per restricted profile; profiles not listed allow all.
const std::map<std::string, std::set<std::string>> & CustomOperationAllowlists();

bool CustomOperationAllowedInProfile(const std::string & operation, const std::string & profile);

/**
 * III behavior nodes per restricted profile; profiles not listed allow all.
 * BehaviorTree.CPP built-in nodes are always allowed. The behavior-node
 * contract exporter publishes these lists, so the catalog build enforces the
 * same allowlist as the runtime.
 */
const std::map<std::string, std::set<std::string>> & BehaviorNodeAllowlists();

/**
 * Behavior nodes of the tree XML that the profile does not allow, sorted.
 * Empty when the profile restricts no nodes. Throws std::runtime_error when
 * the XML is not a tree the runtime behavior-node registry can load.
 */
std::vector<std::string> DisallowedBehaviorNodes(const std::string & tree_xml, const std::string & profile);

/**
 * Throws std::runtime_error naming the mission, mode, tree and nodes when a
 * mode's behavior tree uses a node the profile does not allow.
 */
void RequireMissionAllowedInProfile(const MissionSpecification & specification, const std::string & profile);

}  // namespace iii_drone::mission
