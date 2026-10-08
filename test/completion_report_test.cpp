#include <gtest/gtest.h>

#include <iii_drone_mission/px4/modes/completion_report.hpp>

using iii_drone::px4::CompletionReport;
using namespace std::chrono_literals;

namespace {

constexpr uint8_t kSuccess = 0;
constexpr uint8_t kFailure = 3;

}  // namespace

TEST(CompletionReport, LostCompletionIsRepeatedEveryIntervalUntilAcknowledged) {
    CompletionReport report;
    const auto t0 = CompletionReport::Clock::time_point{} + 100s;

    ASSERT_TRUE(report.Report(kSuccess, t0));
    EXPECT_FALSE(report.Due(t0 + 499ms, true, true).has_value());

    const auto first = report.Due(t0 + 500ms, true, true);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->result, kSuccess);
    EXPECT_EQ(first->count, 1u);
    EXPECT_FALSE(first->warn);

    EXPECT_FALSE(report.Due(t0 + 900ms, true, true).has_value());
    const auto second = report.Due(t0 + 1000ms, true, true);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->count, 2u);

    report.Acknowledge();
    EXPECT_FALSE(report.pending());
    EXPECT_FALSE(report.Due(t0 + 10s, true, true).has_value());
}

TEST(CompletionReport, NothingIsRepeatedWithoutAnActiveModeUnderAnExecutor) {
    CompletionReport report;
    const auto t0 = CompletionReport::Clock::time_point{} + 100s;
    ASSERT_TRUE(report.Report(kSuccess, t0));

    // PX4 moved the mode off, or the mode was started standalone (nobody waits).
    EXPECT_FALSE(report.Due(t0 + 1s, false, true).has_value());
    EXPECT_FALSE(report.Due(t0 + 1s, true, false).has_value());
    EXPECT_TRUE(report.pending());
    EXPECT_TRUE(report.Due(t0 + 1s, true, true).has_value());
}

TEST(CompletionReport, OnlyTheActivationsFirstReportCountsUntilReset) {
    CompletionReport report;
    const auto t0 = CompletionReport::Clock::time_point{} + 100s;

    ASSERT_TRUE(report.Report(kFailure, t0));
    EXPECT_FALSE(report.Report(kSuccess, t0 + 100ms));
    const auto repeat = report.Due(t0 + 1s, true, true);
    ASSERT_TRUE(repeat.has_value());
    EXPECT_EQ(repeat->result, kFailure);

    // An acknowledged report is not re-armed by a later report in the same activation.
    report.Acknowledge();
    EXPECT_FALSE(report.Report(kSuccess, t0 + 2s));
    EXPECT_FALSE(report.pending());

    report.Reset();
    EXPECT_FALSE(report.pending());
    ASSERT_TRUE(report.Report(kSuccess, t0 + 3s));
    const auto next = report.Due(t0 + 3500ms, true, true);
    ASSERT_TRUE(next.has_value());
    EXPECT_EQ(next->result, kSuccess);
    EXPECT_EQ(next->count, 1u);
}

TEST(CompletionReport, WarnsOnceWhenTheExecutorStillHasNoCompletionAfterFiveSeconds) {
    CompletionReport report;
    const auto t0 = CompletionReport::Clock::time_point{} + 100s;
    ASSERT_TRUE(report.Report(kSuccess, t0));

    int warnings = 0;
    unsigned repeats = 0;
    for (auto t = t0 + 500ms; t <= t0 + 8s; t += 500ms) {
        const auto repeat = report.Due(t, true, true);
        ASSERT_TRUE(repeat.has_value());
        ++repeats;
        EXPECT_EQ(repeat->count, repeats);
        if (repeat->warn) {
            ++warnings;
            EXPECT_GE(repeat->since_report, 5s);
        } else if (warnings == 0) {
            EXPECT_LT(repeat->since_report, 5s);
        }
    }
    EXPECT_EQ(warnings, 1);
}
