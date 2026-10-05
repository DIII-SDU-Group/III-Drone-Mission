#include <iii_drone_mission/behavior/behavior_node_registry.hpp>
#include <iii_drone_mission/behavior/port_types.hpp>
#include <iii_drone_mission/mission/mission_catalog.hpp>
#include <iii_drone_mission/mission/mission_specification.hpp>
#include <iii_drone_mission/mission/profile_restrictions.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace mission = iii_drone::mission;

namespace
{

using iii_drone::types::point_t;

constexpr float kMinimumWorldAltitudeM = 1.1F;
// PX4 RegisterExtComponentRequest carries the mode name in char[25].
constexpr std::size_t kMaximumModeNameLength = 24;

const std::vector<std::string> kOptiTrackMissions = {
    "opti-track-hover",
    "opti-track-maneuvers",
    "opti-track-cycle",
};

struct Target
{
    std::string node;
    std::string frame_id;
    point_t position;
};

// Stands in for every III action with its authoritative manifest: the tree is
// parsed exactly as the runtime factory parses it (ports, literal
// conversions, blackboard types). Flags read through BlackboardBool are unset,
// so every maneuver branch of the OptiTrack trees is ticked.
class RecordingAction : public BT::SyncActionNode
{
public:
    RecordingAction(
        const std::string & name,
        const BT::NodeConfig & config,
        std::string id,
        std::vector<Target> * targets
    ) : BT::SyncActionNode(name, config), id_(std::move(id)), targets_(targets) {}

    BT::NodeStatus tick() override
    {
        if (id_ == "BlackboardBool") {
            return BT::NodeStatus::FAILURE;
        }
        if (id_ == "FlyToPosition") {
            const auto frame_id = getInput<std::string>("frame_id");
            const auto position = getInput<point_t>("target_position");
            const auto yaw = getInput<float>("target_yaw");
            if (!frame_id || !position || !yaw) {
                throw std::runtime_error(name() + ": FlyToPosition input is unavailable");
            }
            targets_->push_back({name(), *frame_id, *position});
        }
        if (id_ == "FollowWaypointPath") {
            const auto frame_id = getInput<std::string>("frame_id");
            const auto waypoints = getInput<BT::SharedQueue<point_t>>("waypoints");
            if (!frame_id || !waypoints || !*waypoints || (*waypoints)->empty() || !getInput<float>("target_yaw")) {
                throw std::runtime_error(name() + ": FollowWaypointPath input is unavailable");
            }
            for (const auto & waypoint : **waypoints) {
                targets_->push_back({name(), *frame_id, waypoint});
            }
        }
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::string id_;
    std::vector<Target> * targets_;
};

class PassThroughDecorator : public BT::DecoratorNode
{
public:
    PassThroughDecorator(const std::string & name, const BT::NodeConfig & config)
    : BT::DecoratorNode(name, config) {}

    BT::NodeStatus tick() override
    {
        return child()->executeTick();
    }
};

void RegisterRecordingNodes(BT::BehaviorTreeFactory & factory, std::vector<Target> * targets)
{
    for (const auto & manifest : iii_drone::behavior::CustomBehaviorNodeManifests()) {
        const auto id = manifest.registration_ID;
        if (manifest.type == BT::NodeType::DECORATOR) {
            factory.registerBuilder(manifest, [](const std::string & name, const BT::NodeConfig & config) {
                return std::make_unique<PassThroughDecorator>(name, config);
            });
        } else if (manifest.type == BT::NodeType::ACTION || manifest.type == BT::NodeType::CONDITION) {
            factory.registerBuilder(manifest, [id, targets](const std::string & name, const BT::NodeConfig & config) {
                return std::make_unique<RecordingAction>(name, config, id, targets);
            });
        } else {
            throw std::logic_error("no stand-in for behavior node type of " + id);
        }
    }
}

mission::MissionSpecification Specification(
    const mission::MissionCatalog::SharedPtr & catalog,
    const std::string & id
)
{
    return mission::MissionSpecification(catalog, catalog->entryForProfile(id, "opti_track"), nullptr);
}

}  // namespace

TEST(OptiTrackMissionsTest, RegisteredForOptiTrackSimAndHilWithHoverAsDefault)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    EXPECT_EQ(catalog->defaultEntry("opti_track").id, "opti-track-hover");
    for (const auto & id : kOptiTrackMissions) {
        const auto & entry = catalog->entry(id);
        EXPECT_EQ(entry.profiles, (std::vector<std::string>{"hil", "opti_track", "sim"})) << id;
        EXPECT_EQ(entry.default_for, id == "opti-track-hover"
            ? std::vector<std::string>{"opti_track"}
            : std::vector<std::string>{}) << id;
        EXPECT_EQ(entry.classification, id == "opti-track-hover" ? "production" : "experimental") << id;
    }
}

TEST(OptiTrackMissionsTest, ModeKeysAndNamesAreUniqueAndFitPX4)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    std::set<std::string> keys;
    std::set<std::string> names;
    for (const auto & id : kOptiTrackMissions) {
        for (const auto & entry : Specification(catalog, id).entries()) {
            EXPECT_LE(entry.mode_name.size(), kMaximumModeNameLength) << entry.mode_name;
            EXPECT_TRUE(keys.insert(entry.key).second) << entry.key;
            EXPECT_TRUE(names.insert(entry.mode_name).second) << entry.mode_name;
        }
    }
}

TEST(OptiTrackMissionsTest, ExecutorLoadsThemUnderOptiTrack)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    for (const auto & id : kOptiTrackMissions) {
        EXPECT_NO_THROW(mission::RequireMissionAllowedInProfile(Specification(catalog, id), "opti_track")) << id;
    }
}

TEST(OptiTrackMissionsTest, TreesLoadAndKeepWorldTargetsAboveTheMinimumAltitude)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    for (const auto & id : kOptiTrackMissions) {
        for (const auto & entry : Specification(catalog, id).entries()) {
            SCOPED_TRACE(id + " mode " + entry.key);
            std::vector<Target> targets;
            BT::BehaviorTreeFactory factory;
            RegisterRecordingNodes(factory, &targets);
            auto tree = factory.createTreeFromFile(entry.behavior_tree_xml_file);
            EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);

            const bool lands = entry.key == "ot_cycle_land";
            EXPECT_EQ(targets.empty(), lands);
            for (const auto & target : targets) {
                if (target.frame_id == "drone") {
                    continue;
                }
                EXPECT_EQ(target.frame_id, "world") << target.node;
                EXPECT_GE(target.position[2], kMinimumWorldAltitudeM) << target.node;
            }
        }
    }
}

TEST(OptiTrackMissionsTest, ManeuversPathUsesTheLiteralWaypointList)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    const auto entry = Specification(catalog, "opti-track-maneuvers").GetMissionSpecificationEntry("ot_maneuvers");
    std::vector<Target> targets;
    BT::BehaviorTreeFactory factory;
    RegisterRecordingNodes(factory, &targets);
    auto tree = factory.createTreeFromFile(entry.behavior_tree_xml_file);
    ASSERT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);

    std::vector<point_t> path;
    for (const auto & target : targets) {
        if (target.node == "follow_path_waypoints") {
            path.push_back(target.position);
        }
    }
    ASSERT_EQ(path.size(), 5U);
    EXPECT_FLOAT_EQ(path[1][1], 0.5F);
    EXPECT_FLOAT_EQ(path[1][2], 1.4F);
    EXPECT_FLOAT_EQ(path[3][2], 1.1F);
}
