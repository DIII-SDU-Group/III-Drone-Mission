#include <string>

#include <gtest/gtest.h>

#include <iii_drone_mission/mission/runtime_intent_buffer.hpp>

using iii_drone::mission::RuntimeIntentBuffer;

namespace {

const builtin_interfaces::msg::Time kStamp;

}  // namespace

// SIM gate 2026-10-05: a recharge request accepted while Inspection Demo was
// already leaving for a battery-low recharge waited in the buffer, the Cable
// Charging tree applied it, and the resumed inspection started a second,
// stay-on-cable recharge. It now expires with the activation that accepted it.
TEST(RuntimeIntentBuffer, AnIntentOfAnEndedActivationExpires) {
    RuntimeIntentBuffer buffer;
    buffer.BeginModeActivation("inspection_demo");
    const auto recharge = buffer.Enqueue(
        "inspection_demo.manual_recharge_requested", true, kStamp, "inspection_demo");
    ASSERT_NE(recharge, 0u);

    const auto expired = buffer.BeginModeActivation("reach_cable");
    ASSERT_EQ(expired.size(), 1u);
    EXPECT_EQ(expired.front().sequence_id, recharge);
    buffer.BeginModeActivation("cable_charging");
    EXPECT_TRUE(buffer.Drain().empty());

    std::string ended_mode;
    ASSERT_TRUE(buffer.Expired(recharge, ended_mode));
    EXPECT_EQ(ended_mode, "inspection_demo");
}

TEST(RuntimeIntentBuffer, ANewActivationOfTheSameModeStartsWithoutTheOldIntents) {
    RuntimeIntentBuffer buffer;
    buffer.BeginModeActivation("inspection_demo");
    ASSERT_NE(buffer.Enqueue("inspection_demo.manual_recharge_requested", true, kStamp, "inspection_demo"), 0u);
    EXPECT_EQ(buffer.BeginModeActivation("inspection_demo").size(), 1u);
    EXPECT_TRUE(buffer.Drain().empty());
}

// The caller validated the intent against a mode whose activation ended
// before it reached the buffer.
TEST(RuntimeIntentBuffer, AnIntentValidatedAgainstAnEndedModeIsNotEnqueued) {
    RuntimeIntentBuffer buffer;
    buffer.BeginModeActivation("cable_charging");
    buffer.BeginModeActivation("leave_cable");
    EXPECT_EQ(buffer.Enqueue("charging.interrupt_requested", true, kStamp, "cable_charging"), 0u);
    EXPECT_TRUE(buffer.Drain().empty());
}

TEST(RuntimeIntentBuffer, AnIntentOfTheCurrentActivationIsApplied) {
    RuntimeIntentBuffer buffer;
    buffer.BeginModeActivation("cable_charging");
    const auto stay = buffer.Enqueue("charging.stay_on_cable", true, kStamp, "cable_charging");
    ASSERT_NE(stay, 0u);
    const auto drained = buffer.Drain();
    ASSERT_EQ(drained.size(), 1u);
    EXPECT_EQ(drained.front().sequence_id, stay);
    EXPECT_EQ(drained.front().flag_name, "charging.stay_on_cable");

    EXPECT_TRUE(buffer.BeginModeActivation("leave_cable").empty());
    std::string ended_mode;
    EXPECT_FALSE(buffer.Expired(stay, ended_mode));
}

TEST(RuntimeIntentBuffer, AnIntentValidInEveryModeOutlivesActivations) {
    RuntimeIntentBuffer buffer;
    buffer.BeginModeActivation("inspection_demo");
    ASSERT_NE(buffer.Enqueue("operator.flag", true, kStamp), 0u);
    EXPECT_TRUE(buffer.BeginModeActivation("reach_cable").empty());
    EXPECT_EQ(buffer.Drain().size(), 1u);
}
