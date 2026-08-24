#include <gtest/gtest.h>

#include <iii_drone_mission/behavior/condition_nodes/get_gripper_alignment_yaw_condition_node.hpp>

#include <chrono>
#include <cmath>
#include <optional>

using iii_drone::behavior::CableAxisYawSettler;

namespace {

    using Clock = std::chrono::steady_clock;

    Clock::time_point atSeconds(double seconds) {
        return Clock::time_point(std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(seconds)
        ));
    }

} // namespace

TEST(CableAxisYawSettler, AcceptsStableRecentWindowAfterSlowMapperConvergence) {
    CableAxisYawSettler settler;
    std::optional<double> settled_yaw;

    for (int sample = 0; sample <= 120; ++sample) {
        const double seconds = sample * 0.1;
        const double converging_yaw = 0.16 * (1.0 - std::exp(-seconds / 3.0));
        settled_yaw = settler.addSample(converging_yaw, atSeconds(seconds), 1.5, 0.08, 6);
    }

    ASSERT_TRUE(settled_yaw);
    EXPECT_NEAR(*settled_yaw, 0.156, 0.01);
    EXPECT_LT(settler.sampleCount(), 20U);
    EXPECT_GE(settler.sampleSpanSeconds(), 1.5);
    EXPECT_LE(settler.maxDeviationRadians(), 0.08);
}

TEST(CableAxisYawSettler, RejectsUnstableRecentWindow) {
    CableAxisYawSettler settler;
    std::optional<double> settled_yaw;

    for (int sample = 0; sample <= 30; ++sample) {
        const double seconds = sample * 0.1;
        const double oscillating_yaw = sample % 2 == 0 ? -0.12 : 0.12;
        settled_yaw = settler.addSample(oscillating_yaw, atSeconds(seconds), 1.5, 0.08, 6);
    }

    EXPECT_FALSE(settled_yaw);
    EXPECT_GT(settler.maxDeviationRadians(), 0.08);
}

TEST(CableAxisYawSettler, TreatsOppositeCableDirectionsAsSameAxis) {
    CableAxisYawSettler settler;
    std::optional<double> settled_yaw;

    for (int sample = 0; sample <= 20; ++sample) {
        const double yaw = sample % 2 == 0 ? M_PI_2 - 0.01 : -M_PI_2 + 0.01;
        settled_yaw = settler.addSample(yaw, atSeconds(sample * 0.1), 1.5, 0.08, 6);
    }

    ASSERT_TRUE(settled_yaw);
    EXPECT_NEAR(std::abs(*settled_yaw), M_PI_2, 0.02);
}
