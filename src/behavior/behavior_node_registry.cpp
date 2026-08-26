#include <iii_drone_mission/behavior/behavior_node_registry.hpp>
#include <iii_drone_mission/behavior/trees/tree_executor.hpp>

#include <algorithm>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace iii_drone::behavior
{
namespace
{

template<typename T>
BT::TreeNodeManifest Manifest(std::string_view id)
{
    return BT::CreateManifest<T>(std::string(id));
}

std::string Direction(BT::PortDirection direction)
{
    switch (direction) {
        case BT::PortDirection::INPUT:
            return "INPUT";
        case BT::PortDirection::OUTPUT:
            return "OUTPUT";
        case BT::PortDirection::INOUT:
            return "INOUT";
    }
    return "UNKNOWN";
}

std::string PortSignature(const BT::PortsList & ports)
{
    std::map<std::string, BT::PortInfo> ordered(ports.begin(), ports.end());
    std::ostringstream output;
    bool first = true;
    for (const auto & [name, port] : ordered) {
        if (!first) {
            output << ";";
        }
        first = false;
        output << name << ":" << Direction(port.direction()) << ":" << port.typeName()
               << ":" << port.defaultValueString() << ":" << port.description();
    }
    return output.str();
}

}  // namespace

std::vector<BT::TreeNodeManifest> CustomBehaviorNodeManifests()
{
    std::vector<BT::TreeNodeManifest> manifests = {
        Manifest<HoverManeuverActionNode>("Hover"),
        Manifest<HoverOnCableManeuverActionNode>("HoverOnCable"),
        Manifest<HoverByObjectManeuverActionNode>("HoverByObject"),
        Manifest<FlyToObjectManeuverActionNode>("FlyToObject"),
        Manifest<FlyToPositionManeuverActionNode>("FlyToPosition"),
        Manifest<FollowWaypointPathManeuverActionNode>("FollowWaypointPath"),
        Manifest<CableLandingManeuverActionNode>("CableLanding"),
        Manifest<CableTakeoffManeuverActionNode>("CableTakeoff"),
        Manifest<GripperCommandActionNode>("GripperCommand"),
        Manifest<PLMapperCommandActionNode>("PLMapperCommand"),
        Manifest<VerifyPowerlineDetectedConditionNode>("VerifyPowerlineDetected"),
        Manifest<SelectTargetLineConditionNode>("SelectTargetLine"),
        Manifest<TargetProvider>("TargetProvider"),
        Manifest<StoreCurrentStateConditionNode>("StoreCurrentState"),
        Manifest<UpdatePowerlineOverviewActionNode>("UpdatePowerlineOverview"),
        Manifest<GetPowerlineOverviewActionNode>("GetPowerlineOverview"),
        Manifest<GetPylonOverviewActionNode>("GetPylonOverview"),
        Manifest<PowerlineWaypointProviderActionNode>("PowerlineWaypointProvider"),
        Manifest<PhaseWaypointProviderActionNode>("PhaseWaypointProvider"),
        Manifest<BT::LoopNode<iii_drone::types::point_t>>("LoopPoint"),
        Manifest<SplitPointQueueActionNode>("SplitPointQueue"),
        Manifest<PublishPowerlineWaypointsConditionNode>("PublishPowerlineWaypoints"),
        Manifest<VerifyGripperClosedConditionNode>("VerifyGripperClosed"),
        Manifest<ShouldRechargeBatteryLowConditionNode>("ShouldRechargeBatteryLow"),
        Manifest<CableChargingMonitorActionNode>("CableChargingMonitor"),
        Manifest<VerifyDisarmedConditionNode>("VerifyDisarmed"),
        Manifest<GetGripperAlignmentYawConditionNode>("GetGripperAlignmentYaw"),
        Manifest<ModeExecutorActionNode>("ModeExecutorAction"),
        Manifest<LogMessageActionNode>("LogMessage"),
        Manifest<ApplyPendingIntentUpdatesActionNode>("ApplyPendingIntentUpdates"),
        Manifest<SetBlackboardBoolActionNode>("SetBlackboardBool"),
        Manifest<BlackboardBoolConditionNode>("BlackboardBool"),
        Manifest<SetBlackboardStringActionNode>("SetBlackboardString"),
        Manifest<BlackboardStringEqualsConditionNode>("BlackboardStringEquals"),
        Manifest<InitializeInspectionWaypointsActionNode>("InitializeInspectionWaypoints"),
        Manifest<GetCurrentInspectionWaypointActionNode>("GetCurrentInspectionWaypoint"),
        Manifest<AdvanceInspectionWaypointActionNode>("AdvanceInspectionWaypoint"),
        Manifest<StartRosbagRecordingActionNode>("StartRosbagRecording"),
        Manifest<StopRosbagRecordingActionNode>("StopRosbagRecording"),
        Manifest<RosbagRecordingScopeDecorator>("RosbagRecordingScope"),
        Manifest<StringEqualsConditionNode>("StringEquals"),
        Manifest<RetryUntilSuccessfulOnAbortedDecorator>("RetryUntilSuccessfulOnAborted"),
    };
    std::sort(
        manifests.begin(),
        manifests.end(),
        [](const auto & first, const auto & second) {
            return first.registration_ID < second.registration_ID;
        }
    );
    const auto duplicate = std::adjacent_find(
        manifests.begin(),
        manifests.end(),
        [](const auto & first, const auto & second) {
            return first.registration_ID == second.registration_ID;
        }
    );
    if (duplicate != manifests.end()) {
        throw std::logic_error("duplicate behavior-node registry ID: " + duplicate->registration_ID);
    }
    return manifests;
}

std::string DescribeBehaviorManifestDifference(
    const BT::TreeNodeManifest & expected,
    const BT::TreeNodeManifest & actual
)
{
    std::ostringstream output;
    output << "expected type=" << static_cast<int>(expected.type)
           << " ports=" << PortSignature(expected.ports)
           << ", actual type=" << static_cast<int>(actual.type)
           << " ports=" << PortSignature(actual.ports);
    return output.str();
}

void ValidateRuntimeBehaviorFactory(const BT::BehaviorTreeFactory & factory)
{
    const auto expected = CustomBehaviorNodeManifests();
    std::map<std::string, const BT::TreeNodeManifest *> expected_by_id;
    for (const auto & manifest : expected) {
        expected_by_id.emplace(manifest.registration_ID, &manifest);
    }
    for (const auto & [id, expected_manifest] : expected_by_id) {
        const auto found = factory.manifests().find(id);
        if (found == factory.manifests().end()) {
            throw std::runtime_error("runtime behavior factory is missing registered node: " + id);
        }
        if (
            found->second.type != expected_manifest->type ||
            PortSignature(found->second.ports) != PortSignature(expected_manifest->ports)
        ) {
            throw std::runtime_error(
                "runtime behavior factory contract differs for " + id + ": " +
                DescribeBehaviorManifestDifference(*expected_manifest, found->second)
            );
        }
    }
    for (const auto & [id, actual] : factory.manifests()) {
        (void)actual;
        if (factory.builtinNodes().count(id) == 0 && expected_by_id.count(id) == 0) {
            throw std::runtime_error("runtime behavior factory has an undeclared custom node: " + id);
        }
    }
}

}  // namespace iii_drone::behavior
