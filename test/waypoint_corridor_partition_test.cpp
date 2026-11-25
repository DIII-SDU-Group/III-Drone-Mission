#include <cmath>
#include <deque>
#include <initializer_list>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <iii_drone_mission/behavior/action_nodes/follow_waypoint_path_maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/partition_point_queue_action_node.hpp>
#include <iii_drone_mission/behavior/powerline_geometry.hpp>

namespace {
using iii_drone::behavior::PartitionWaypointQueue;
using iii_drone::behavior::ComputeOutsideBoundaryIndices;
using iii_drone::behavior::IsOutsideCorridorBeyondCompletionTolerance;
using iii_drone::behavior::FollowWaypointPathTransitionMode;
using iii_drone::types::point_t;
using BT::SharedQueue;

point_t Point(double x, double y, double z) {
    point_t point;
    point << x, y, z;
    return point;
}

point_t Point(float x) {
    point_t point;
    point << x, 0.0F, 0.0F;
    return point;
}

SharedQueue<point_t> Queue(std::initializer_list<float> xs) {
    auto queue = std::make_shared<std::deque<point_t>>();
    for (const auto x : xs) queue->push_back(Point(x));
    return queue;
}

std::vector<float> Values(const SharedQueue<point_t> & queue) {
    std::vector<float> values;
    for (const auto & point : *queue) values.push_back(point.x());
    return values;
}
}  // namespace

TEST(WaypointCorridorPartitionTest, NormalDepartureKeepsExplicitOutsideBoundaryAndEveryPointOnce) {
    const auto route = Queue({10.0F, 20.0F, 30.0F});  // top, outside clearance, under-cable target
    const auto parts = PartitionWaypointQueue(route, 1);
    ASSERT_TRUE(parts);
    ASSERT_TRUE(parts->boundary);
    EXPECT_FLOAT_EQ(parts->boundary->x(), 20.0F);
    EXPECT_EQ(Values(parts->before), (std::vector<float>{10.0F}));
    EXPECT_EQ(Values(parts->through), (std::vector<float>{10.0F, 20.0F}));
    EXPECT_EQ(Values(parts->after), (std::vector<float>{30.0F}));

    auto reconstructed = Values(parts->before);
    reconstructed.push_back(parts->boundary->x());
    const auto after = Values(parts->after);
    reconstructed.insert(reconstructed.end(), after.begin(), after.end());
    EXPECT_EQ(reconstructed, Values(route));
}

TEST(WaypointCorridorPartitionTest, DirectInsideRouteHasNoBoundaryAndRetainsEveryFlyToWaypoint) {
    const auto route = Queue({1.0F, 2.0F});
    const auto parts = PartitionWaypointQueue(route, -1);
    ASSERT_TRUE(parts);
    EXPECT_FALSE(parts->boundary);
    EXPECT_TRUE(parts->before->empty());
    EXPECT_TRUE(parts->through->empty());
    EXPECT_EQ(Values(parts->after), (std::vector<float>{1.0F, 2.0F}));
}

TEST(WaypointCorridorPartitionTest, ReturnBoundarySeparatesInsideStopFromContinuousOutsideSuffix) {
    const auto route = Queue({40.0F, 30.0F, 20.0F, 10.0F});  // target, outside, top, saved start
    const auto parts = PartitionWaypointQueue(route, 1);
    ASSERT_TRUE(parts);
    ASSERT_TRUE(parts->boundary);
    EXPECT_EQ(Values(parts->before), (std::vector<float>{40.0F}));
    EXPECT_FLOAT_EQ(parts->boundary->x(), 30.0F);
    EXPECT_EQ(Values(parts->after), (std::vector<float>{20.0F, 10.0F}));
}

TEST(WaypointCorridorPartitionTest, RejectsBoundaryOutsideQueue) {
    EXPECT_FALSE(PartitionWaypointQueue(Queue({1.0F}), 1));
}

TEST(WaypointCorridorPartitionTest, InsideClearanceCandidateFallsBackToIndividualRoutes) {
    point_t middle = Point(0.0F);
    point_t positive_outer = Point(5.0F);
    point_t negative_outer = Point(-5.0F);
    iii_drone::types::vector_t lateral_axis;
    lateral_axis << 1.0F, 0.0F, 0.0F;
    const auto classification = iii_drone::behavior::powerline_geometry::ClassifyCorridor(
        Point(2.0F), middle, lateral_axis, positive_outer, negative_outer, 3.0
    );
    ASSERT_TRUE(classification.inside_corridor);
    const bool outside_beyond_tolerance = IsOutsideCorridorBeyondCompletionTolerance(
        classification.lateral_distance_to_middle, 3.0, 0.1
    );
    EXPECT_FALSE(outside_beyond_tolerance);
    const auto indices = ComputeOutsideBoundaryIndices(3, 1, outside_beyond_tolerance);
    EXPECT_EQ(indices.departure, -1);
    EXPECT_EQ(indices.return_route, -1);
}

TEST(WaypointCorridorPartitionTest, OutsideCandidateHasNonemptyRouteSuffixes) {
    point_t middle = Point(0.0F);
    point_t positive_outer = Point(5.0F);
    point_t negative_outer = Point(-5.0F);
    iii_drone::types::vector_t lateral_axis;
    lateral_axis << 1.0F, 0.0F, 0.0F;
    const auto classification = iii_drone::behavior::powerline_geometry::ClassifyCorridor(
        Point(4.0F), middle, lateral_axis, positive_outer, negative_outer, 3.0
    );
    ASSERT_FALSE(classification.inside_corridor);
    const bool outside_beyond_tolerance = IsOutsideCorridorBeyondCompletionTolerance(
        classification.lateral_distance_to_middle, 3.0, 0.1
    );
    EXPECT_TRUE(outside_beyond_tolerance);
    const auto indices = ComputeOutsideBoundaryIndices(3, 1, outside_beyond_tolerance);
    EXPECT_EQ(indices.departure, 0);
    EXPECT_EQ(indices.return_route, 1);
    const auto return_suffix_size = 3U - 1U - static_cast<std::size_t>(indices.return_route);
    EXPECT_GT(return_suffix_size, 0U);
}

TEST(WaypointCorridorPartitionTest, CorridorEdgeWithinArrivalToleranceIsNotAcceptedAsBoundary) {
    point_t middle = Point(0.0F);
    point_t positive_outer = Point(5.0F);
    point_t negative_outer = Point(-5.0F);
    iii_drone::types::vector_t lateral_axis;
    lateral_axis << 1.0F, 0.0F, 0.0F;
    const auto classification = iii_drone::behavior::powerline_geometry::ClassifyCorridor(
        Point(3.05F), middle, lateral_axis, positive_outer, negative_outer, 3.0
    );
    ASSERT_FALSE(classification.inside_corridor);
    const bool outside_beyond_tolerance = IsOutsideCorridorBeyondCompletionTolerance(
        classification.lateral_distance_to_middle, 3.0, 0.1
    );
    EXPECT_FALSE(outside_beyond_tolerance);
    const auto indices = ComputeOutsideBoundaryIndices(3, 1, outside_beyond_tolerance);
    EXPECT_EQ(indices.departure, -1);
    EXPECT_EQ(indices.return_route, -1);
}

TEST(PowerlineReturnRoute, SameSideExteriorUsesClearanceRouteAndExactReverseForBothSides) {
    using namespace iii_drone::behavior::powerline_geometry;
    point_t middle = Point(4.0, -3.0, 0.0);
    iii_drone::types::vector_t cross;
    cross << 0.0, 1.0, 0.0;

    for (const bool positive_entry : {false, true}) {
        const double side = positive_entry ? 1.0 : -1.0;
        const point_t positive_outer = Point(4.0, -1.0, 8.0);
        const point_t negative_outer = Point(4.0, -5.0, 8.0);
        const point_t start = Point(4.0, -3.0 + side * 2.75, 7.4);
        const point_t entry_under = Point(4.0, -3.0 + side * 2.0, 2.0);
        const auto route = BuildInsideCorridorReturnRoute(
            start, middle, entry_under, positive_outer, negative_outer, cross,
            positive_entry, 3.0, 0.1, 1.2
        );

        ASSERT_EQ(route.kind, InsideCorridorRouteKind::same_side_exterior_clearance);
        ASSERT_EQ(route.waypoints.size(), 4U);
        ASSERT_EQ(route.outside_boundary_index, 2);
        EXPECT_NEAR(route.waypoints[1].x(), start.x(), 1e-9);
        EXPECT_NEAR(route.waypoints[2].x(), start.x(), 1e-9);
        EXPECT_NEAR(route.waypoints[1].z(), start.z(), 1e-9);
        EXPECT_NEAR(route.waypoints[2].z(), entry_under.z(), 1e-9);
        EXPECT_NEAR(route.waypoints[3].y(), entry_under.y(), 1e-9);

        const double outside_signed_distance = side * (route.waypoints[1].y() - middle.y());
        EXPECT_GE(outside_signed_distance, 3.21);
        EXPECT_GT(outside_signed_distance, 3.2);
        EXPECT_GE(outside_signed_distance, 2.0 + 1.2);
        EXPECT_GE(outside_signed_distance, side * (start.y() - middle.y()));
        EXPECT_GT(std::abs(route.waypoints[1].y() - middle.y()), 2.0);
        EXPECT_GT(std::abs(route.waypoints[2].y() - middle.y()), 2.0);

        const auto classification = ClassifyCorridor(
            route.waypoints[route.outside_boundary_index], middle, cross,
            positive_outer, negative_outer, 3.0
        );
        const auto indices = ComputeOutsideBoundaryIndices(
            route.waypoints.size(), route.outside_boundary_index,
            IsOutsideCorridorBeyondCompletionTolerance(
                classification.lateral_distance_to_middle, 3.0, 0.1
            )
        );
        EXPECT_EQ(indices.departure, 1);
        EXPECT_EQ(indices.return_route, 1);
        EXPECT_LT(indices.departure, static_cast<int>(route.waypoints.size()) - 1);
        EXPECT_LT(indices.return_route, static_cast<int>(route.waypoints.size()) - 1);

        ASSERT_EQ(route.return_waypoints.size(), route.waypoints.size());
        for (std::size_t i = 0; i < route.return_waypoints.size(); ++i) {
            EXPECT_TRUE(route.return_waypoints[i].isApprox(
                route.waypoints[route.waypoints.size() - i - 1]
            ));
        }
    }
}

TEST(PowerlineReturnRoute, RotatedCorridorPreservesStartStationAndKeepsVerticalChangeOutside) {
    using namespace iii_drone::behavior::powerline_geometry;
    iii_drone::types::vector_t cross;
    cross << 0.6, 0.8, 0.0;
    iii_drone::types::vector_t span;
    span << -0.8, 0.6, 0.0;
    const point_t middle = Point(2.0, -1.0, 0.0);
    const double station = 6.5;
    const auto at = [&](double lateral, double z) {
        return middle + station * span + lateral * cross + iii_drone::types::vector_t(0.0, 0.0, z);
    };
    const point_t start = at(-2.7, 7.0);
    const point_t entry_under = at(-2.0, 2.0);
    const point_t positive_outer = at(2.0, 8.0);
    const point_t negative_outer = at(-2.0, 8.0);
    const auto route = BuildInsideCorridorReturnRoute(
        start, middle, entry_under, positive_outer, negative_outer, cross,
        false, 3.0, 0.1, 1.2
    );

    ASSERT_EQ(route.kind, InsideCorridorRouteKind::same_side_exterior_clearance);
    ASSERT_EQ(route.waypoints.size(), 4U);
    for (std::size_t i = 1; i + 1 < route.waypoints.size(); ++i) {
        EXPECT_NEAR(route.waypoints[i].dot(span), start.dot(span), 1e-8);
    }
    EXPECT_NEAR(route.waypoints[2].dot(span), start.dot(span), 1e-8);
    EXPECT_NEAR(route.waypoints[1].z(), route.waypoints[0].z(), 1e-9);
    EXPECT_NEAR(route.waypoints[2].z(), route.waypoints[3].z(), 1e-9);
    EXPECT_LT(route.waypoints[2].dot(cross) - middle.dot(cross), -2.0);
}

TEST(PowerlineReturnRoute, RecordedLeaveClimbsOutsideTheLowerConductor) {
    using namespace iii_drone::behavior::powerline_geometry;
    const point_t start = Point(1.753, -8.643, 7.433);
    const point_t middle = Point(-0.917, -7.971, 2.185);
    const point_t entry_under = Point(-0.130, -8.169, 2.185);
    iii_drone::types::vector_t cross = middle - entry_under;
    cross.normalize();
    const point_t negative_outer = Point(entry_under.x(), entry_under.y(), 4.185);
    const point_t positive_outer = middle + 2.339 * cross;
    const auto route = BuildInsideCorridorReturnRoute(
        start, middle, entry_under, positive_outer, negative_outer, cross,
        false, 3.0, 0.1, 2.0
    );
    ASSERT_EQ(route.kind, InsideCorridorRouteKind::same_side_exterior_clearance);
    ASSERT_EQ(route.return_waypoints.size(), 4U);
    EXPECT_TRUE(route.return_waypoints.front().isApprox(entry_under));
    EXPECT_TRUE(route.return_waypoints.back().isApprox(start));
    // Every segment that changes height must stay laterally outside, including
    // arrival tolerance. The old middle-under to initial-position diagonal
    // crossed the lower conductor at roughly gripper height.
    for (std::size_t i = 1; i < route.return_waypoints.size(); ++i) {
        const auto & from = route.return_waypoints[i - 1];
        const auto & to = route.return_waypoints[i];
        if (std::abs(from.z() - to.z()) > 0.01) {
            EXPECT_LT((from - middle).dot(cross), -3.1);
            EXPECT_LT((to - middle).dot(cross), -3.1);
        }
    }
}

TEST(PowerlineReturnRoute, OppositeExteriorSelectsTopRouteAndInteriorKeepsDirectRoute) {
    using namespace iii_drone::behavior::powerline_geometry;
    const point_t middle = Point(0.0, 0.0, 0.0);
    const point_t positive_outer = Point(0.0, 2.0, 8.0);
    const point_t negative_outer = Point(0.0, -2.0, 8.0);
    iii_drone::types::vector_t cross;
    cross << 0.0, 1.0, 0.0;
    const point_t entry_under = Point(0.0, 2.0, 2.0);

    const auto opposite = BuildInsideCorridorReturnRoute(
        Point(0.0, -2.7, 7.0), middle, entry_under,
        positive_outer, negative_outer, cross, true, 3.0, 0.1, 1.2
    );
    EXPECT_EQ(opposite.kind, InsideCorridorRouteKind::top_clearance);
    EXPECT_TRUE(opposite.waypoints.empty());

    const auto interior = BuildInsideCorridorReturnRoute(
        Point(0.0, 0.5, 7.0), middle, entry_under,
        positive_outer, negative_outer, cross, true, 3.0, 0.1, 1.2
    );
    ASSERT_EQ(interior.kind, InsideCorridorRouteKind::direct);
    ASSERT_EQ(interior.waypoints.size(), 3U);
    ASSERT_EQ(interior.return_waypoints.size(), 3U);
    EXPECT_EQ(interior.outside_boundary_index, -1);
    EXPECT_TRUE(interior.waypoints[0].isApprox(Point(0.0, 0.5, 7.0)));
    EXPECT_TRUE(interior.waypoints[1].isApprox(Point(0.0, 0.0, 2.0)));
    EXPECT_TRUE(interior.waypoints[2].isApprox(entry_under));
}

TEST(WaypointCorridorPartitionTest, NonRepeatingPathStopsAtBoundaryAndSavedReturnEnd) {
    using iii_drone_interfaces::msg::Waypoint;
    EXPECT_EQ(FollowWaypointPathTransitionMode(0, 2, false, 0), Waypoint::TRANSITION_BLEND);
    EXPECT_EQ(FollowWaypointPathTransitionMode(1, 2, false, 0), Waypoint::TRANSITION_STOP);
}

TEST(WaypointCorridorPartitionTest, RepeatingPathStillBlendsExceptAtConfiguredLoopEntry) {
    using iii_drone_interfaces::msg::Waypoint;
    EXPECT_EQ(FollowWaypointPathTransitionMode(0, 3, true, 0), Waypoint::TRANSITION_BLEND);
    EXPECT_EQ(FollowWaypointPathTransitionMode(1, 3, true, 2), Waypoint::TRANSITION_STOP);
}

TEST(WaypointCorridorPartitionTest, RuntimeFactoryLoadsAndTicksRegisteredQueueNodes) {
    BT::BehaviorTreeFactory factory;
    iii_drone::behavior::RegisterWaypointQueueNodes(factory);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("queue", Queue({1.0F, 2.0F, 3.0F}));
    const auto xml = R"(
        <root BTCPP_format="4" main_tree_to_execute="Main">
            <BehaviorTree ID="Main">
                <Sequence>
                    <Action ID="PartitionPointQueue" queue="{queue}" boundary_index="1"
                        before="{before}" through="{through}" after="{after}"
                        boundary="{boundary}" has_boundary="{has_boundary}"/>
                    <Condition ID="QueueHasPoints" queue="{through}"/>
                </Sequence>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml, blackboard);
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(Values(blackboard->get<SharedQueue<point_t>>("through")),
        (std::vector<float>{1.0F, 2.0F}));
    EXPECT_EQ(Values(blackboard->get<SharedQueue<point_t>>("after")),
        (std::vector<float>{3.0F}));
}

TEST(WaypointCorridorPartitionTest, ClearancePointExceedsCorridorAndArrivalToleranceOnEitherSide) {
    namespace geom = iii_drone::behavior::powerline_geometry;
    // A16 had outer conductor + 2 m clearance at 2.812 m, still inside
    // the 3.1 m corridor. Rotate and translate the fixture to cover world axes.
    const point_t middle = Point(20.0, -5.0, 8.0);
    const point_t cross = Point(0.6, 0.8, 0.0);
    const point_t along = Point(-0.8, 0.6, 0.0);
    for (const bool positive : {false, true}) {
        const float sign = positive ? 1.0F : -1.0F;
        const point_t outer = middle + sign * 0.812F * cross + 15.0F * along;
        auto boundary = geom::OutsideCorridorClearancePoint(
            outer, middle, cross, positive, 3.1, 0.1, 2.0);
        ASSERT_TRUE(boundary);
        const double lateral = (*boundary - middle).dot(cross);
        EXPECT_NEAR(sign * lateral, 3.31, 1e-5);
        EXPECT_NEAR((*boundary - outer).dot(along), 0.0, 1e-5);
        EXPECT_FLOAT_EQ(boundary->z(), outer.z());
        EXPECT_TRUE(IsOutsideCorridorBeyondCompletionTolerance(std::abs(lateral), 3.1, 0.1));
        const auto indices = ComputeOutsideBoundaryIndices(4, 2, true);
        EXPECT_EQ(indices.departure, 1);
        EXPECT_EQ(indices.return_route, 1);
    }
}

TEST(WaypointCorridorPartitionTest, ClearancePointRetainsLargerConductorClearance) {
    namespace geom = iii_drone::behavior::powerline_geometry;
    const auto boundary = geom::OutsideCorridorClearancePoint(
        Point(5.0, 17.0, 9.0), Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0),
        true, 3.1, 0.1, 2.0);
    ASSERT_TRUE(boundary);
    EXPECT_TRUE(boundary->isApprox(Point(7.0, 17.0, 9.0)));
}
