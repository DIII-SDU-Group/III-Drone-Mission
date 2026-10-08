#include <gtest/gtest.h>

#include <px4_msgs/msg/vehicle_status.hpp>

#include <iii_drone_mission/px4/mode_executors/handoff_failsafe_deferral.hpp>

using iii_drone::px4::HandoffFailsafeDeferral;
using VehicleStatus = px4_msgs::msg::VehicleStatus;

namespace {

constexpr uint8_t kMissionMode = VehicleStatus::NAVIGATION_STATE_EXTERNAL2;
constexpr uint8_t kNextMissionMode = VehicleStatus::NAVIGATION_STATE_EXTERNAL3;

}  // namespace

TEST(HandoffFailsafeDeferral, KeepsTheFiveSecondTimeout) {
    EXPECT_EQ(HandoffFailsafeDeferral::kTimeoutS, 5);
}

TEST(HandoffFailsafeDeferral, ReleasesOnceWhenTheScheduledModeRuns) {
    HandoffFailsafeDeferral deferral;
    deferral.Begin(kMissionMode);
    EXPECT_TRUE(deferral.pending());

    // Samples from before the switch do not end the handoff.
    EXPECT_FALSE(deferral.Reached(VehicleStatus::NAVIGATION_STATE_POSCTL, true));
    // PX4 selected the mode, but it does not run yet (e.g. while arming).
    EXPECT_FALSE(deferral.Reached(kMissionMode, false));
    EXPECT_TRUE(deferral.pending());

    EXPECT_TRUE(deferral.Reached(kMissionMode, true));
    EXPECT_FALSE(deferral.pending());

    // While the mode runs nothing is deferred.
    EXPECT_FALSE(deferral.Reached(kMissionMode, true));
    EXPECT_FALSE(deferral.Abandon());
}

TEST(HandoffFailsafeDeferral, EveryModeTransitionIsItsOwnHandoff) {
    HandoffFailsafeDeferral deferral;
    deferral.Begin(kMissionMode);
    ASSERT_TRUE(deferral.Reached(kMissionMode, true));

    deferral.Begin(VehicleStatus::NAVIGATION_STATE_AUTO_LAND);
    EXPECT_FALSE(deferral.Reached(kMissionMode, true));
    EXPECT_TRUE(deferral.Reached(VehicleStatus::NAVIGATION_STATE_AUTO_LAND, true));

    deferral.Begin(kMissionMode);
    EXPECT_TRUE(deferral.Reached(kMissionMode, true));
}

TEST(HandoffFailsafeDeferral, ANewHandoffReplacesAPendingOne) {
    HandoffFailsafeDeferral deferral;
    deferral.Begin(kMissionMode);
    deferral.Begin(kNextMissionMode);

    EXPECT_FALSE(deferral.Reached(kMissionMode, true));
    EXPECT_TRUE(deferral.Reached(kNextMissionMode, true));
}

TEST(HandoffFailsafeDeferral, AbandonReleasesOnlyAPendingHandoff) {
    HandoffFailsafeDeferral deferral;
    EXPECT_FALSE(deferral.Abandon());

    deferral.Begin(kMissionMode);
    EXPECT_TRUE(deferral.Abandon());
    EXPECT_FALSE(deferral.pending());
    EXPECT_FALSE(deferral.Abandon());
    EXPECT_FALSE(deferral.Reached(kMissionMode, true));
}
