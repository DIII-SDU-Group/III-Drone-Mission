#include <iii_drone_mission/behavior/behavior_node_registry.hpp>
#include <iii_drone_mission/behavior/port_types.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using iii_drone::types::point_t;
using PointQueue = BT::SharedQueue<point_t>;

void ExpectPoint(const point_t & point, float x, float y, float z)
{
    EXPECT_FLOAT_EQ(point[0], x);
    EXPECT_FLOAT_EQ(point[1], y);
    EXPECT_FLOAT_EQ(point[2], z);
}

BT::TreeNodeManifest AuthoritativeManifest(const std::string & id)
{
    for (const auto & manifest : iii_drone::behavior::CustomBehaviorNodeManifests()) {
        if (manifest.registration_ID == id) {
            return manifest;
        }
    }
    throw std::logic_error("behavior node is not registered: " + id);
}

// Stands in for FollowWaypointPath with its authoritative manifest, so the
// tree is parsed exactly as the runtime factory parses it.
class CaptureWaypoints : public BT::SyncActionNode
{
public:
    CaptureWaypoints(const std::string & name, const BT::NodeConfig & config, PointQueue * captured)
    : BT::SyncActionNode(name, config), captured_(captured) {}

    BT::NodeStatus tick() override
    {
        return getInput("waypoints", *captured_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    PointQueue * captured_;
};

std::string FollowWaypointPathTree(const std::string & waypoints)
{
    return
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
        "<BehaviorTree ID=\"Main\">"
        "<Action ID=\"FollowWaypointPath\" frame_id=\"world\" target_yaw=\"0\" waypoints=\"" + waypoints + "\"/>"
        "</BehaviorTree>"
        "</root>";
}

}  // namespace

TEST(LiteralWaypointsTest, ParsesPointsInOrder)
{
    const auto queue = BT::convertFromString<PointQueue>("1,2,3;-0.5,0,1.25");
    ASSERT_NE(queue, nullptr);
    ASSERT_EQ(queue->size(), 2U);
    ExpectPoint(queue->at(0), 1.0F, 2.0F, 3.0F);
    ExpectPoint(queue->at(1), -0.5F, 0.0F, 1.25F);
}

TEST(LiteralWaypointsTest, ToleratesWhitespaceAroundNumbersAndSeparators)
{
    const auto queue = BT::convertFromString<PointQueue>(
        "  0.5 , 0 ,1.2 ;\n\t 0,0.5, 1.4\t; +1, -1 , 1.1 "
    );
    ASSERT_EQ(queue->size(), 3U);
    ExpectPoint(queue->at(0), 0.5F, 0.0F, 1.2F);
    ExpectPoint(queue->at(1), 0.0F, 0.5F, 1.4F);
    ExpectPoint(queue->at(2), 1.0F, -1.0F, 1.1F);
}

TEST(LiteralWaypointsTest, RejectsMalformedLists)
{
    const std::vector<std::string> malformed = {
        "",
        "  \t",
        "1,2",
        "1,2,3,4",
        "1,2,3;",
        ";1,2,3",
        "1,2,3;;4,5,6",
        "1,,3",
        "1,2,x",
        "1,2,3x",
        "1 2,3,4",
        "1;2;3",
        "nan,0,1",
        "1,inf,2",
        "1e40,0,1",
        "++1,0,1",
    };
    for (const auto & text : malformed) {
        EXPECT_THROW(
            static_cast<void>(BT::convertFromString<PointQueue>(text)),
            std::invalid_argument
        ) << "'" << text << "'";
    }
}

TEST(LiteralWaypointsTest, PointQueuePortsConvertLiterals)
{
    // BT.CPP converts a literal port value with the converter stored in the
    // port's manifest, both when it parses the tree XML and in getInput().
    const std::vector<std::pair<std::string, std::string>> ports = {
        {"FollowWaypointPath", "waypoints"},
        {"SplitPointQueue", "queue"},
        {"PartitionPointQueue", "queue"},
        {"QueueHasPoints", "queue"},
    };
    for (const auto & [node, port_name] : ports) {
        const auto manifest = AuthoritativeManifest(node);
        const auto & port = manifest.ports.at(port_name);
        ASSERT_TRUE(static_cast<bool>(port.converter())) << node << "." << port_name;
        const auto queue = port.converter()("0.5,0,1.2;0,0.5,1.4").cast<PointQueue>();
        ASSERT_EQ(queue->size(), 2U) << node << "." << port_name;
        ExpectPoint(queue->at(1), 0.0F, 0.5F, 1.4F);
        EXPECT_THROW(port.converter()("0.5,0"), std::invalid_argument) << node << "." << port_name;
    }
}

TEST(LiteralWaypointsTest, FollowWaypointPathTreeAcceptsLiteralWaypoints)
{
    PointQueue captured;
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder(
        AuthoritativeManifest("FollowWaypointPath"),
        [&captured](const std::string & name, const BT::NodeConfig & config) {
            return std::make_unique<CaptureWaypoints>(name, config, &captured);
        }
    );

    auto tree = factory.createTreeFromText(FollowWaypointPathTree("0.5,0,1.2; 0,0.5,1.4; -0.5,0,1.2"));
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    ASSERT_NE(captured, nullptr);
    ASSERT_EQ(captured->size(), 3U);
    ExpectPoint(captured->at(2), -0.5F, 0.0F, 1.2F);

    // A malformed literal fails when the tree is parsed, before any tick.
    EXPECT_THROW(
        static_cast<void>(factory.createTreeFromText(FollowWaypointPathTree("0.5,0,1.2;0,0.5"))),
        BT::LogicError
    );
}

TEST(LiteralWaypointsTest, LoopPointIteratesALiteralQueue)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT::LoopNode<point_t>>("LoopPoint");

    auto tree = factory.createTreeFromText(
        "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\">"
        "<BehaviorTree ID=\"Main\">"
        "<LoopPoint queue=\"1,0,1.2;2,0,1.2;3,0,1.5\" value=\"{waypoint}\"><AlwaysSuccess/></LoopPoint>"
        "</BehaviorTree>"
        "</root>"
    );
    EXPECT_EQ(tree.tickWhileRunning(), BT::NodeStatus::SUCCESS);
    ExpectPoint(tree.rootBlackboard()->get<point_t>("waypoint"), 3.0F, 0.0F, 1.5F);
}
