#include <gtest/gtest.h>

#include <iii_drone_mission/behavior/action_nodes/inspection_waypoint_progress_nodes.hpp>
#include <iii_drone_mission/behavior/action_nodes/phase_waypoint_provider_action_node.hpp>
#include <iii_drone_mission/behavior/powerline_geometry.hpp>

#include <cmath>
#include <vector>

using iii_drone::behavior::BuildCorridorInspectionRoute;
using iii_drone::behavior::CorridorInspectionResume;
using iii_drone::behavior::EvaluateCorridorInspectionStart;
using iii_drone::behavior::InspectionWaypointShouldBlendToNext;
using iii_drone::behavior::NextInspectionWaypointIndex;
using iii_drone::behavior::powerline_geometry::ComputePowerlineAlignedYaw;
using iii_drone::behavior::powerline_geometry::ComputePylonAlignedAxes;
using iii_drone::types::point_t;
using iii_drone::types::vector_t;

namespace {

    point_t point(double x, double y, double z) {
        point_t value;
        value << x, y, z;
        return value;
    }

    const std::vector<point_t> kConductors{
        point(0.0, -2.0, 5.0),
        point(0.0, -1.0, 6.0),
        point(0.0, 1.0, 6.0),
        point(0.0, 2.0, 5.0),
    };

    const vector_t kPowerlineDirection = point(1.0, 0.0, 0.0);
    const point_t kPylonStart = point(0.0, 0.0, 0.0);
    const point_t kPylonEnd = point(10.0, 0.0, 0.0);

    auto buildRoute(const point_t & start) {
        return BuildCorridorInspectionRoute(
            kConductors,
            kPowerlineDirection,
            kPylonStart,
            kPylonEnd,
            start,
            1.5,
            2.0,
            2.0,
            2.0,
            3.0,
            0.5,
            0.35
        );
    }

    void expectPointNear(
        const point_t & actual,
        double expected_x,
        double expected_y,
        double expected_z
    ) {
        EXPECT_NEAR(actual[0], expected_x, 1e-6);
        EXPECT_NEAR(actual[1], expected_y, 1e-6);
        EXPECT_NEAR(actual[2], expected_z, 1e-6);
    }

} // namespace

TEST(CorridorInspectionYaw, AlignsWithHorizontalPowerlineDirection) {
    const auto yaw = ComputePowerlineAlignedYaw(point(1.0, 1.0, 4.0), 0.0);

    ASSERT_TRUE(yaw);
    EXPECT_NEAR(*yaw, M_PI_4, 1e-6);
}

TEST(CorridorInspectionYaw, SelectsNearestEquivalentAxisHeading) {
    const auto yaw = ComputePowerlineAlignedYaw(kPowerlineDirection, 3.0);

    ASSERT_TRUE(yaw);
    EXPECT_NEAR(*yaw, M_PI, 1e-6);
}

TEST(CorridorInspectionYaw, RejectsVerticalDirection) {
    EXPECT_FALSE(ComputePowerlineAlignedYaw(point(0.0, 0.0, 1.0), 0.0));
}

TEST(CorridorInspectionGeometry, UsesPylonSpanAsLongBaselineCorridorAxis) {
    const auto axes = ComputePylonAlignedAxes(
        point(0.147, 0.989, 0.0),
        point(-5.18, -6.98, 0.0),
        point(1.37, 19.09, 0.0)
    );

    ASSERT_TRUE(axes);
    const vector_t expected = point(6.55, 26.07, 0.0).normalized();
    EXPECT_NEAR(axes->direction_no_z[0], expected[0], 1e-6);
    EXPECT_NEAR(axes->direction_no_z[1], expected[1], 1e-6);
    EXPECT_GT(axes->direction_no_z.dot(point(0.147, 0.989, 0.0)), 0.0);
}

TEST(CorridorInspectionGeometry, UsesPylonAxisDespiteLargeMapperDisagreement) {
    const auto axes = ComputePylonAlignedAxes(
        point(1.0, 0.0, 0.0),
        point(0.0, 0.0, 0.0),
        point(0.0, 10.0, 0.0)
    );

    ASSERT_TRUE(axes);
    EXPECT_NEAR(axes->direction_no_z[0], 0.0, 1e-6);
    EXPECT_NEAR(std::abs(axes->direction_no_z[1]), 1.0, 1e-6);
}

TEST(CorridorInspectionRoute, StopsInsidePylonsAndClimbsOnlyForCrossings) {
    const auto route = buildRoute(point(2.0, 3.5, 6.0));

    ASSERT_TRUE(route);
    EXPECT_EQ(route->selected_route, "positive_start");
    EXPECT_EQ(route->loop_start_index, 0U);
    ASSERT_EQ(route->waypoints.size(), 8U);
    expectPointNear(route->waypoints[0], 4.0, 3.5, 6.0);
    expectPointNear(route->waypoints[1], 4.0, 3.5, 7.5);
    expectPointNear(route->waypoints[2], 4.0, -3.5, 7.5);
    expectPointNear(route->waypoints[3], 4.0, -3.5, 6.0);
    expectPointNear(route->waypoints[4], 6.0, -3.5, 6.0);
    expectPointNear(route->waypoints[5], 6.0, -3.5, 7.5);
    expectPointNear(route->waypoints[6], 6.0, 3.5, 7.5);
    expectPointNear(route->waypoints[7], 6.0, 3.5, 6.0);
}

TEST(CorridorInspectionRoute, UsesHighestConductorOnEachSideIndependently) {
    const std::vector<point_t> asymmetric_conductors{
        point(0.0, -2.0, 4.0),
        point(0.0, -1.0, 5.0),
        point(0.0, 1.0, 7.0),
        point(0.0, 2.0, 6.0),
    };

    const auto route = BuildCorridorInspectionRoute(
        asymmetric_conductors,
        kPowerlineDirection,
        kPylonStart,
        kPylonEnd,
        point(2.0, 3.5, 7.0),
        1.5,
        2.0,
        2.0,
        2.0,
        3.0,
        0.5,
        0.35
    );

    ASSERT_TRUE(route);
    ASSERT_EQ(route->waypoints.size() - route->loop_start_index, 8U);
    expectPointNear(route->waypoints[0], 4.0, 3.5, 7.0);
    expectPointNear(route->waypoints[1], 4.0, 3.5, 8.5);
    expectPointNear(route->waypoints[2], 4.0, -3.5, 8.5);
    expectPointNear(route->waypoints[3], 4.0, -3.5, 5.0);
    expectPointNear(route->waypoints[4], 6.0, -3.5, 5.0);
    expectPointNear(route->waypoints[5], 6.0, -3.5, 8.5);
}

TEST(CorridorInspectionRoute, ExcludesCentralTopConductorFromSideFlightAltitudes) {
    const std::vector<point_t> conductors_with_central_top{
        point(0.0, -2.0, 4.5),
        point(0.0, 0.0, 9.4),
        point(0.0, 2.0, 6.4),
        point(0.0, 2.1, 2.6),
    };

    const auto route = BuildCorridorInspectionRoute(
        conductors_with_central_top,
        kPowerlineDirection,
        kPylonStart,
        kPylonEnd,
        point(2.0, -3.5, 4.5),
        1.5,
        2.0,
        2.0,
        2.0,
        3.0,
        0.5,
        0.35
    );

    ASSERT_TRUE(route);
    ASSERT_EQ(route->waypoints.size() - route->loop_start_index, 8U);
    const auto loop = route->loop_start_index;
    EXPECT_NEAR(route->waypoints[loop + 0][2], 4.5, 1e-6);
    EXPECT_NEAR(route->waypoints[loop + 3][2], 6.4, 1e-6);
    EXPECT_NEAR(route->waypoints[loop + 4][2], 6.4, 1e-6);
    EXPECT_NEAR(route->waypoints[loop + 7][2], 4.5, 1e-6);
    EXPECT_NEAR(route->waypoints[loop + 1][2], 10.9, 1e-6);
    EXPECT_NEAR(route->waypoints[loop + 2][2], 10.9, 1e-6);
}

TEST(CorridorInspectionRoute, SelectsNearestSideAndReverseDirection) {
    const auto route = buildRoute(point(8.0, -3.5, 6.0));

    ASSERT_TRUE(route);
    EXPECT_EQ(route->selected_route, "negative_end");
    EXPECT_EQ(route->loop_start_index, 0U);
    ASSERT_EQ(route->waypoints.size(), 8U);
    expectPointNear(route->waypoints[0], 6.0, -3.5, 6.0);
    expectPointNear(route->waypoints[1], 6.0, -3.5, 7.5);
    expectPointNear(route->waypoints[2], 6.0, 3.5, 7.5);
    expectPointNear(route->waypoints[3], 6.0, 3.5, 6.0);
    expectPointNear(route->waypoints[4], 4.0, 3.5, 6.0);
}

TEST(CorridorInspectionRoute, RejectsFreshStartInsideCorridorBeforeMotion) {
    const auto route = buildRoute(point(5.0, 0.0, 2.0));

    EXPECT_FALSE(route);
}

TEST(CorridorInspectionRoute, RejectsFreshStartInsideClearanceEvenAboveCorridor) {
    const auto route = buildRoute(point(5.0, 2.5, 8.0));

    EXPECT_FALSE(route);
}

TEST(CorridorInspectionRoute, UsesDirectIngressToNearestSameSideLegPoint) {
    const auto route = buildRoute(point(5.0, 4.5, 9.0));

    ASSERT_TRUE(route);
    EXPECT_EQ(route->selected_route, "positive_start");
    EXPECT_EQ(route->loop_start_index, 1U);
    ASSERT_EQ(route->waypoints.size(), 9U);
    expectPointNear(route->waypoints[0], 5.0, 3.5, 6.0);
    expectPointNear(route->waypoints[1], 4.0, 3.5, 6.0);
}

TEST(CorridorInspectionRoute, UsesDirectIngressOnNegativeSideWithoutCrossingCorridor) {
    const auto route = buildRoute(point(5.5, -5.0, 4.0));

    ASSERT_TRUE(route);
    EXPECT_EQ(route->selected_route, "negative_end");
    EXPECT_EQ(route->loop_start_index, 1U);
    expectPointNear(route->waypoints[0], 5.5, -3.5, 6.0);
}

TEST(CorridorInspectionEligibility, ReportsClearanceSpanAndIngress) {
    const auto eligibility = EvaluateCorridorInspectionStart(
        kConductors,
        kPowerlineDirection,
        kPylonStart,
        kPylonEnd,
        point(5.0, 4.0, 30.0),
        1.5,
        2.0,
        2.0,
        0.5,
        0.35
    );

    EXPECT_TRUE(eligibility.evaluable);
    EXPECT_TRUE(eligibility.eligible);
    EXPECT_EQ(eligibility.side, "positive");
    EXPECT_NEAR(eligibility.measured_lateral_clearance_m, 2.0, 1e-6);
    EXPECT_NEAR(eligibility.required_lateral_clearance_m, 1.5, 1e-6);
    EXPECT_TRUE(eligibility.between_pylons);
    EXPECT_NEAR(eligibility.distance_from_start_boundary_m, 5.5, 1e-6);
    EXPECT_NEAR(eligibility.distance_to_end_boundary_m, 5.5, 1e-6);
    EXPECT_TRUE(eligibility.ingress_point_valid);
    expectPointNear(eligibility.ingress_point, 5.0, 3.5, 6.0);
    EXPECT_TRUE(eligibility.failure_reasons.empty());
}

TEST(CorridorInspectionEligibility, RejectsLongitudinalStartBeyondMargin) {
    const auto before = buildRoute(point(-0.6, 4.0, 6.0));
    const auto at_margin = buildRoute(point(-0.5, 4.0, 6.0));
    const auto after = buildRoute(point(10.6, -4.0, 6.0));

    EXPECT_FALSE(before);
    EXPECT_TRUE(at_margin);
    EXPECT_FALSE(after);
}

TEST(CorridorInspectionEligibility, HasNoCorridorRelativeMaximumAltitude) {
    const auto route = buildRoute(point(5.0, -4.0, 100.0));

    ASSERT_TRUE(route);
    expectPointNear(route->waypoints.front(), 5.0, -3.5, 6.0);
}

TEST(CorridorInspectionRoute, ResumesInterruptedSegmentAtItsActiveTarget) {
    const point_t interrupted_position = point(4.0, -3.5, 6.75);
    const CorridorInspectionResume resume{
        "positive_start",
        3,
        0,
        0,
        interrupted_position,
    };

    const auto route = BuildCorridorInspectionRoute(
        kConductors,
        kPowerlineDirection,
        kPylonStart,
        kPylonEnd,
        interrupted_position,
        1.5,
        2.0,
        2.0,
        2.0,
        3.0,
        0.5,
        0.35,
        resume,
        0.75
    );

    ASSERT_TRUE(route);
    EXPECT_EQ(route->selected_route, "positive_start");
    EXPECT_TRUE(route->resumed);
    EXPECT_EQ(route->loop_route_offset, 3U);
    EXPECT_EQ(route->loop_start_index, 0U);
    ASSERT_EQ(route->waypoints.size(), 8U);
    expectPointNear(route->waypoints[0], 4.0, -3.5, 6.0);
    expectPointNear(route->waypoints[1], 6.0, -3.5, 6.0);
    expectPointNear(route->waypoints[7], 4.0, -3.5, 7.5);
}

TEST(CorridorInspectionRoute, AccumulatesResumeOffsetAcrossChargingCycles) {
    const point_t interrupted_position = point(5.0, -3.5, 6.0);
    const CorridorInspectionResume resume{
        "positive_start",
        2,
        0,
        3,
        interrupted_position,
    };

    const auto route = BuildCorridorInspectionRoute(
        kConductors,
        kPowerlineDirection,
        kPylonStart,
        kPylonEnd,
        interrupted_position,
        1.5,
        2.0,
        2.0,
        2.0,
        3.0,
        0.5,
        0.35,
        resume,
        0.75
    );

    ASSERT_TRUE(route);
    EXPECT_TRUE(route->resumed);
    EXPECT_EQ(route->loop_route_offset, 5U);
    expectPointNear(route->waypoints[0], 6.0, -3.5, 7.5);
}

TEST(CorridorInspectionRoute, RejectsPylonsThatDoNotFollowCorridor) {
    const auto route = BuildCorridorInspectionRoute(
        kConductors,
        kPowerlineDirection,
        kPylonStart,
        point(0.0, 10.0, 0.0),
        point(0.0, 0.0, 2.0),
        1.5,
        2.0,
        2.0,
        2.0,
        3.0,
        0.5,
        0.35
    );

    EXPECT_FALSE(route);
}

TEST(CorridorInspectionRoute, UsesLongBaselinePylonDirectionForInspectionLegs) {
    const double heading_error = 0.1;
    const point_t pylon_end = point(
        10.0 * std::cos(heading_error),
        10.0 * std::sin(heading_error),
        0.0
    );
    const point_t valid_positive_start = point(
        5.0 * std::cos(heading_error) - 4.0 * std::sin(heading_error),
        5.0 * std::sin(heading_error) + 4.0 * std::cos(heading_error),
        6.0
    );
    const auto route = BuildCorridorInspectionRoute(
        kConductors,
        kPowerlineDirection,
        kPylonStart,
        pylon_end,
        valid_positive_start,
        1.5,
        2.0,
        2.0,
        2.0,
        3.0,
        0.5,
        0.35
    );

    ASSERT_TRUE(route);
    ASSERT_EQ(route->waypoints.size() - route->loop_start_index, 8U);
    vector_t expected_direction = pylon_end - kPylonStart;
    expected_direction[2] = 0.0;
    expected_direction.normalize();
    vector_t inspection_leg =
        route->waypoints[route->loop_start_index + 4] -
        route->waypoints[route->loop_start_index + 3];
    inspection_leg[2] = 0.0;
    inspection_leg.normalize();
    EXPECT_NEAR(std::abs(inspection_leg.dot(expected_direction)), 1.0, 1e-6);
}

TEST(InspectionWaypointProgress, WrapsToLoopStartAfterOneTimeApproach) {
    EXPECT_EQ(NextInspectionWaypointIndex(0, 7, 3), 1);
    EXPECT_EQ(NextInspectionWaypointIndex(6, 7, 3), 3);
    EXPECT_FALSE(NextInspectionWaypointIndex(6, 7, 7));
    EXPECT_FALSE(NextInspectionWaypointIndex(0, 0, 0));
}

TEST(InspectionWaypointProgress, BlendsEveryRepeatingLoopWaypoint) {
    for (int index = 0; index < 8; ++index) {
        EXPECT_EQ(InspectionWaypointShouldBlendToNext(index, 8, 0), true);
    }

    EXPECT_EQ(InspectionWaypointShouldBlendToNext(0, 11, 3), true);
    EXPECT_EQ(InspectionWaypointShouldBlendToNext(1, 11, 3), true);
    EXPECT_EQ(InspectionWaypointShouldBlendToNext(2, 11, 3), false);
    for (int index = 3; index < 11; ++index) {
        EXPECT_EQ(InspectionWaypointShouldBlendToNext(index, 11, 3), true);
    }
    EXPECT_FALSE(InspectionWaypointShouldBlendToNext(0, 7, 0));
}
