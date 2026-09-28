#include <gtest/gtest.h>

#include <iii_drone_mission/px4/modes/pending_activation_callback.hpp>

using iii_drone::px4::PendingActivationCallback;

TEST(PendingActivationCallback, CallbackRegisteredBeforePartialDeactivateInvokesOnce) {
    PendingActivationCallback pending;
    int invocations = 0;

    pending.Set([&invocations]() { ++invocations; });
    pending.OnDeactivate(true);
    pending.Invoke();
    pending.Invoke();

    EXPECT_EQ(invocations, 1);
}

TEST(PendingActivationCallback, CallbackRegisteredAfterPartialDeactivateInvokesOnce) {
    PendingActivationCallback pending;
    int invocations = 0;

    pending.OnDeactivate(true);
    pending.Set([&invocations]() { ++invocations; });
    pending.Invoke();
    pending.Invoke();

    EXPECT_EQ(invocations, 1);
}

TEST(PendingActivationCallback, FullDeactivateCancelsCallback) {
    PendingActivationCallback pending;
    int invocations = 0;

    pending.Set([&invocations]() { ++invocations; });
    pending.OnDeactivate(false);
    pending.Invoke();

    EXPECT_EQ(invocations, 0);
}

TEST(PendingActivationCallback, ExplicitCancelCancelsCallback) {
    PendingActivationCallback pending;
    int invocations = 0;

    pending.Set([&invocations]() { ++invocations; });
    pending.Cancel();
    pending.Invoke();

    EXPECT_EQ(invocations, 0);
}

TEST(PendingActivationCallback, ReentrantReplacementSurvivesFirstInvoke) {
    PendingActivationCallback pending;
    int first_invocations = 0;
    int second_invocations = 0;

    pending.Set([&]() {
        ++first_invocations;
        pending.Set([&second_invocations]() { ++second_invocations; });
    });

    pending.Invoke();
    EXPECT_EQ(first_invocations, 1);
    EXPECT_EQ(second_invocations, 0);

    pending.Invoke();
    EXPECT_EQ(first_invocations, 1);
    EXPECT_EQ(second_invocations, 1);
}
