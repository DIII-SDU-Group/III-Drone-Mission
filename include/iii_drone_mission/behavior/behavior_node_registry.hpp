#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include <string>
#include <vector>

namespace iii_drone::behavior
{

/// The authoritative IDs, types, and ports for every III behavior node.
std::vector<BT::TreeNodeManifest> CustomBehaviorNodeManifests();

/// Fail closed when a runtime factory differs from the authoritative registry.
void ValidateRuntimeBehaviorFactory(const BT::BehaviorTreeFactory & factory);

/// Stable diagnostic form used by tests and runtime errors.
std::string DescribeBehaviorManifestDifference(
    const BT::TreeNodeManifest & expected,
    const BT::TreeNodeManifest & actual
);

}  // namespace iii_drone::behavior
