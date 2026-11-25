#include <iii_drone_mission/behavior/behavior_node_registry.hpp>
#include <iii_drone_mission/mission/mission_catalog.hpp>
#include <iii_drone_mission/mission/mission_specification.hpp>
#include <iii_drone_mission/mission/profile_restrictions.hpp>

#include <behaviortree_cpp/bt_factory.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace mission = iii_drone::mission;

namespace
{

class ScopedProfileEnvironment
{
public:
    explicit ScopedProfileEnvironment(const char * value)
    {
        if (const char * previous = std::getenv(mission::kRuntimeProfileEnvironment); previous != nullptr) {
            previous_ = previous;
        }
        if (value == nullptr) {
            unsetenv(mission::kRuntimeProfileEnvironment);
        } else {
            setenv(mission::kRuntimeProfileEnvironment, value, 1);
        }
    }

    ~ScopedProfileEnvironment()
    {
        if (previous_) {
            setenv(mission::kRuntimeProfileEnvironment, previous_->c_str(), 1);
        } else {
            unsetenv(mission::kRuntimeProfileEnvironment);
        }
    }

private:
    std::optional<std::string> previous_;
};

std::string Tree(const std::string & main_tree, const std::string & other_trees = "")
{
    return "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
        "<BehaviorTree ID=\"Main\">" + main_tree + "</BehaviorTree>" + other_trees + "</root>";
}

}  // namespace

TEST(ProfileRestrictionsTest, ParameterProfileWinsOverTheEnvironment)
{
    const ScopedProfileEnvironment environment("sim");
    EXPECT_EQ(mission::ResolveRuntimeProfile("opti_track"), "opti_track");
    EXPECT_EQ(mission::ResolveRuntimeProfile(" opti_track\n"), "opti_track");
    EXPECT_EQ(mission::ResolveRuntimeProfile(""), "sim");
    EXPECT_EQ(mission::ResolveRuntimeProfile("  "), "sim");
}

TEST(ProfileRestrictionsTest, MissingProfileIsEmptyAndUnrestricted)
{
    const ScopedProfileEnvironment environment(nullptr);
    EXPECT_EQ(mission::ResolveRuntimeProfile(""), "");
    EXPECT_TRUE(mission::CustomOperationAllowedInProfile("cable_landing", ""));
    EXPECT_TRUE(mission::DisallowedBehaviorNodes(Tree("<CableLanding/>"), "").empty());
}

TEST(ProfileRestrictionsTest, RejectionWording)
{
    EXPECT_EQ(
        mission::NotAvailableInProfileMessage("custom operation cable_landing", "opti_track"),
        "custom operation cable_landing is not available in the opti_track profile"
    );
}

TEST(ProfileRestrictionsTest, OptiTrackAllowsOnlyFlightCustomOperations)
{
    for (const std::string operation : {"hover", "fly_to_position", "follow_waypoint_path"}) {
        EXPECT_TRUE(mission::CustomOperationAllowedInProfile(operation, "opti_track")) << operation;
    }
    for (const std::string operation : {
            "cable_aware_fly_to_position", "fly_to_object", "hover_by_object",
            "hover_on_cable", "cable_landing", "cable_takeoff"}) {
        EXPECT_FALSE(mission::CustomOperationAllowedInProfile(operation, "opti_track")) << operation;
        EXPECT_TRUE(mission::CustomOperationAllowedInProfile(operation, "sim")) << operation;
        EXPECT_TRUE(mission::CustomOperationAllowedInProfile(operation, "real")) << operation;
    }
}

TEST(ProfileRestrictionsTest, OptiTrackNodeAllowlistNamesRegisteredIIINodes)
{
    const std::set<std::string> expected = {
        "ApplyPendingIntentUpdates", "BlackboardBool", "BlackboardStringEquals", "FlyToPosition",
        "FollowWaypointPath", "Hover", "LogMessage", "LoopPoint", "ModeExecutorAction",
        "PartitionPointQueue", "QueueHasPoints", "RetryUntilSuccessfulOnAborted",
        "RosbagRecordingScope", "SetBlackboardBool", "SetBlackboardString", "SplitPointQueue",
        "StartRosbagRecording", "StopRosbagRecording", "StoreCurrentState", "StringEquals",
        "VerifyDisarmed", "WaitForPX4Airborne",
    };
    const auto & allowlists = mission::BehaviorNodeAllowlists();
    ASSERT_EQ(allowlists.count("opti_track"), 1U);
    EXPECT_EQ(allowlists.at("opti_track"), expected);

    std::set<std::string> registered;
    for (const auto & manifest : iii_drone::behavior::CustomBehaviorNodeManifests()) {
        registered.insert(manifest.registration_ID);
    }
    const BT::BehaviorTreeFactory factory;
    for (const auto & id : expected) {
        EXPECT_EQ(registered.count(id), 1U) << id;
        EXPECT_EQ(factory.builtinNodes().count(id), 0U) << id;
    }
}

TEST(ProfileRestrictionsTest, AllowedNodesAndBuiltinsPassInEveryTree)
{
    const auto tree = Tree(
        "<Sequence>"
        "<SetBlackboard output_key=\"target\" value=\"0,0,1.2\"/>"
        "<Decorator ID=\"RosbagRecordingScope\" recording_id=\"r\">"
        "<Sequence><Action ID=\"FlyToPosition\" frame_id=\"world\" target_position=\"{target}\" target_yaw=\"0\"/>"
        "<Hover duration_s=\"1\"/><SubTree ID=\"Inner\"/></Sequence>"
        "</Decorator>"
        "</Sequence>",
        "<BehaviorTree ID=\"Inner\"><Fallback><BlackboardBool flag=\"a\"/><Delay delay_msec=\"10\">"
        "<LogMessage message=\"m\" log_level=\"1\"/></Delay></Fallback></BehaviorTree>"
    );
    EXPECT_TRUE(mission::DisallowedBehaviorNodes(tree, "opti_track").empty());
}

TEST(ProfileRestrictionsTest, DisallowedNodesAreFoundInEveryTreeAndElementForm)
{
    const auto tree = Tree(
        "<Sequence><Hover duration_s=\"1\"/><SubTree ID=\"Inner\"/></Sequence>",
        "<BehaviorTree ID=\"Inner\"><Sequence><CableLanding/>"
        "<Action ID=\"GripperCommand\" gripper_command=\"0\"/></Sequence></BehaviorTree>"
        "<BehaviorTree ID=\"Unused\"><Condition ID=\"VerifyPowerlineDetected\"/></BehaviorTree>"
    );
    EXPECT_EQ(
        mission::DisallowedBehaviorNodes(tree, "opti_track"),
        (std::vector<std::string>{"CableLanding", "GripperCommand", "VerifyPowerlineDetected"})
    );
    EXPECT_TRUE(mission::DisallowedBehaviorNodes(tree, "sim").empty());
    EXPECT_TRUE(mission::DisallowedBehaviorNodes(tree, "real").empty());
}

TEST(ProfileRestrictionsTest, TreeTheRuntimeCannotLoadIsRejected)
{
    EXPECT_THROW(
        static_cast<void>(mission::DisallowedBehaviorNodes(Tree("<NoSuchNode/>"), "opti_track")),
        std::runtime_error
    );
    EXPECT_THROW(
        static_cast<void>(mission::DisallowedBehaviorNodes("<root>", "opti_track")),
        std::runtime_error
    );
}

TEST(ProfileRestrictionsTest, ExecutorRefusesInstalledMissionWithCableNodesUnderOptiTrack)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    const mission::MissionSpecification specification(
        catalog,
        catalog->entryForProfile("inspection-production", "real"),
        nullptr
    );
    EXPECT_NO_THROW(mission::RequireMissionAllowedInProfile(specification, "real"));
    EXPECT_NO_THROW(mission::RequireMissionAllowedInProfile(specification, ""));
    try {
        mission::RequireMissionAllowedInProfile(specification, "opti_track");
        FAIL() << "inspection-production loaded under opti_track";
    } catch (const std::runtime_error & error) {
        const std::string message = error.what();
        EXPECT_EQ(message.rfind("mission inspection-production (mode ", 0), 0U) << message;
        EXPECT_NE(message.find("behavior tree behavior_trees/"), std::string::npos) << message;
        EXPECT_NE(message.find(") is not available in the opti_track profile"), std::string::npos) << message;
    }
}
