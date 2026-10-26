#include <gtest/gtest.h>

#include <cmath>

#include <iii_drone_mission/px4/stick_takeover.hpp>

using iii_drone::px4::StickTakeoverDetector;

namespace {

constexpr double kThreshold = 0.35;

StickTakeoverDetector::Sample Sticks(float roll, float pitch, float yaw, float throttle, bool valid = true) {
    StickTakeoverDetector::Sample sample;
    sample.valid = valid;
    sample.roll = roll;
    sample.pitch = pitch;
    sample.yaw = yaw;
    sample.throttle = throttle;
    return sample;
}

}  // namespace

TEST(StickTakeoverDetector, StationaryThrottleAtTheBottomIsNoTakeover) {
    StickTakeoverDetector detector;
    detector.Observe(Sticks(0.0F, 0.0F, 0.0F, -1.0F));
    detector.Rebaseline();

    for (int sample = 0; sample < 100; ++sample) {
        EXPECT_FALSE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, -1.0F), kThreshold));
    }
}

TEST(StickTakeoverDetector, NonCentredSticksWithinTheThresholdAreNoTakeover) {
    StickTakeoverDetector detector;
    detector.Observe(Sticks(0.5F, -0.6F, 0.4F, 0.8F));
    detector.Rebaseline();

    EXPECT_FALSE(detector.ObserveActive(Sticks(0.6F, -0.5F, 0.3F, 0.9F), kThreshold));
    EXPECT_FALSE(detector.ObserveActive(Sticks(0.5F, -0.6F, 0.4F, 0.8F), kThreshold));
}

TEST(StickTakeoverDetector, MovementOfAnyAxisBeyondTheThresholdIsATakeover) {
    const StickTakeoverDetector::Sample baseline = Sticks(0.0F, 0.0F, 0.0F, -1.0F);
    const StickTakeoverDetector::Sample moved[] = {
        Sticks(0.4F, 0.0F, 0.0F, -1.0F),
        Sticks(0.0F, -0.4F, 0.0F, -1.0F),
        Sticks(0.0F, 0.0F, 0.4F, -1.0F),
        Sticks(0.0F, 0.0F, 0.0F, -0.6F),
    };
    for (const auto & sample : moved) {
        StickTakeoverDetector detector;
        detector.Observe(baseline);
        detector.Rebaseline();
        EXPECT_TRUE(detector.ObserveActive(sample, kThreshold));
    }
}

TEST(StickTakeoverDetector, EveryActivationTakesANewBaseline) {
    StickTakeoverDetector detector;
    detector.Observe(Sticks(0.0F, 0.0F, 0.0F, -1.0F));
    detector.Rebaseline();
    EXPECT_FALSE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, -1.0F), kThreshold));

    // Between activations the pilot centres the throttle.
    detector.Observe(Sticks(0.0F, 0.0F, 0.0F, 0.0F));
    detector.Rebaseline();
    EXPECT_FALSE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, 0.0F), kThreshold));
    EXPECT_TRUE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, -1.0F), kThreshold));
}

TEST(StickTakeoverDetector, WithoutAnEarlierSampleTheFirstActiveSampleIsTheBaseline) {
    StickTakeoverDetector detector;
    detector.Rebaseline();

    EXPECT_FALSE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, -1.0F), kThreshold));
    EXPECT_FALSE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, -1.0F), kThreshold));
    EXPECT_TRUE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, 0.0F), kThreshold));
}

TEST(StickTakeoverDetector, InvalidSamplesAndMissingAxesAreIgnored) {
    StickTakeoverDetector detector;
    detector.Observe(Sticks(0.0F, 0.0F, 0.0F, -1.0F));
    detector.Rebaseline();

    // An invalid sample neither triggers nor moves the baseline.
    EXPECT_FALSE(detector.ObserveActive(Sticks(1.0F, 1.0F, 1.0F, 1.0F, false), kThreshold));
    EXPECT_FALSE(detector.ObserveActive(Sticks(NAN, NAN, NAN, -1.0F), kThreshold));
    EXPECT_FALSE(detector.ObserveActive(Sticks(0.0F, 0.0F, 0.0F, -1.0F), kThreshold));

    // An axis that had no data at activation is baselined by its first value.
    StickTakeoverDetector partial;
    partial.Observe(Sticks(NAN, NAN, NAN, -1.0F));
    partial.Rebaseline();
    EXPECT_FALSE(partial.ObserveActive(Sticks(0.5F, 0.5F, 0.5F, -1.0F), kThreshold));
    EXPECT_FALSE(partial.ObserveActive(Sticks(0.5F, 0.5F, 0.5F, -1.0F), kThreshold));
    EXPECT_TRUE(partial.ObserveActive(Sticks(0.5F, 0.0F, 0.5F, -1.0F), kThreshold));
}
