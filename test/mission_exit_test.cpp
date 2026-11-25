#include <atomic>
#include <chrono>
#include <future>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <iii_drone_interfaces/msg/pl_mapper_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <iii_drone_mission/mission/mission_exit.hpp>

namespace {

using iii_drone::mission::MissionControl;
using iii_drone::mission::MissionExitDecision;
using iii_drone::mission::MissionExitReason;
using iii_drone::mission::ModeCompletionHandling;
using iii_drone::mission::VehicleControlSample;
using Status = px4_msgs::msg::VehicleStatus;
using SteadyClock = std::chrono::steady_clock;

constexpr uint8_t kExecutor = 3;

VehicleControlSample sample(uint64_t timestamp_us, uint8_t executor_in_charge,
                            uint8_t nav_state, bool failsafe = false,
                            SteadyClock::time_point receipt = SteadyClock::now()) {
    VehicleControlSample value;
    value.timestamp_us = timestamp_us;
    value.executor_in_charge = executor_in_charge;
    value.nav_state = nav_state;
    value.failsafe = failsafe;
    value.receipt = receipt;
    return value;
}

MissionExitDecision operatorHold() {
    MissionExitDecision decision;
    decision.reason = MissionExitReason::OperatorModeChange;
    decision.px4_nav_state = Status::NAVIGATION_STATE_AUTO_LOITER;
    decision.px4_timestamp_us = 10;
    return decision;
}

}  // namespace

TEST(MissionExit, LatchIsRunScopedFirstObserverWinsAndClosesDispatch) {
    MissionControl control;
    // No run: nothing to exit (e.g. after a completed mission).
    EXPECT_FALSE(control.LatchExit(MissionExitReason::OperatorModeChange, 4, 1));
    EXPECT_FALSE(control.DispatchClosed());

    const auto run = control.BeginRun();
    const auto first = control.LatchExit(MissionExitReason::OperatorModeChange, 4, 1);
    ASSERT_TRUE(first);
    EXPECT_EQ(first->run, run);
    EXPECT_TRUE(control.DispatchClosed());
    EXPECT_FALSE(control.AcquireDispatch().allowed());
    // The px4_ros2 path observing the same transition later is a no-op.
    EXPECT_FALSE(control.LatchExit(MissionExitReason::Failsafe, 4, 2));
    ASSERT_TRUE(control.LastExit());
    EXPECT_EQ(control.LastExit()->reason, MissionExitReason::OperatorModeChange);
    EXPECT_TRUE(control.ExitInProgress());

    // A stale activation inside the exited run cannot start a tree.
    EXPECT_FALSE(control.AdmitModeStart());
    EXPECT_TRUE(control.DispatchClosed());

    control.EndRun();
    EXPECT_FALSE(control.ExitInProgress());
    EXPECT_TRUE(control.ExitLatched());
    // A standalone mode start after the exited run reopens dispatch.
    EXPECT_TRUE(control.AdmitModeStart());
    EXPECT_FALSE(control.DispatchClosed());

    // The next executor run starts clean.
    control.LatchExit(MissionExitReason::OperatorModeChange, 4, 3);
    control.BeginRun();
    EXPECT_FALSE(control.ExitLatched());
    EXPECT_FALSE(control.DispatchClosed());
    EXPECT_TRUE(control.AcquireDispatch().allowed());
}

TEST(MissionExit, GateWaitsForInFlightDispatchTickThenBlocksEveryLaterTick) {
    // Race R2: a tree tick is dispatching exactly when the exit fires.
    MissionControl control;
    control.BeginRun();
    std::promise<void> holding;
    std::promise<void> release;
    auto release_future = release.get_future().share();
    std::atomic<bool> dispatched{false};
    std::thread tree([&]() {
        const auto permit = control.AcquireDispatch();
        ASSERT_TRUE(permit.allowed());
        holding.set_value();
        release_future.wait();
        dispatched = true;  // goal sent inside the permit
    });
    holding.get_future().wait();

    auto latch = std::async(std::launch::async, [&control]() {
        return control.LatchExit(MissionExitReason::OperatorModeChange, 4, 1);
    });
    EXPECT_EQ(latch.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout)
        << "the exit must not close the gate under an in-flight dispatch";
    release.set_value();
    tree.join();
    const auto record = latch.get();
    ASSERT_TRUE(record);
    EXPECT_TRUE(dispatched.load());
    // Every later tick of every tree is refused.
    EXPECT_FALSE(control.AcquireDispatch().allowed());
}

TEST(MissionExit, GuardBlocksOnlyDispatchingTicksOfTheProcess) {
    auto & control = MissionControl::Process();
    control.ResetForTest();
    control.BeginRun();
    int ticks = 0;
    const auto dispatch = [&ticks](bool dispatching) {
        return iii_drone::mission::guardMissionDispatch(
            dispatching, [&ticks]() { ++ticks; return std::string("ticked"); },
            []() { return std::string("blocked"); });
    };
    EXPECT_EQ(dispatch(true), "ticked");
    ASSERT_TRUE(control.LatchExit(MissionExitReason::OperatorModeChange, 4, 1));
    EXPECT_EQ(dispatch(true), "blocked");
    // A RUNNING node keeps handling its result/cancellation.
    EXPECT_EQ(dispatch(false), "ticked");
    EXPECT_EQ(ticks, 2);
    EXPECT_TRUE(iii_drone::mission::missionExitClosedDispatch());
    control.ResetForTest();
}

TEST(MissionExit, MonitorFiresOnlyOnFallingEdgeAfterConfirmedRun) {
    MissionControl control;
    const auto before_run = SteadyClock::now() - std::chrono::seconds(1);
    // Samples before any run never exit.
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(100, 0, Status::NAVIGATION_STATE_AUTO_LOITER), kExecutor));
    control.BeginRun();
    // A not-in-charge sample before an in-charge sample of this run (a stale
    // pre-activation sample) is not an exit.
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(200, 0, Status::NAVIGATION_STATE_AUTO_LOITER), kExecutor));
    // An in-charge sample received before the run began cannot confirm it.
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(300, kExecutor, Status::NAVIGATION_STATE_EXTERNAL1, false, before_run), kExecutor));
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(400, 0, Status::NAVIGATION_STATE_AUTO_LOITER), kExecutor));
    // Confirmation, then mission-driven transitions keep the executor in
    // charge: next-mode handoff (external), executor land, on-cable disarm.
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(500, kExecutor, Status::NAVIGATION_STATE_EXTERNAL1), kExecutor));
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(600, kExecutor, Status::NAVIGATION_STATE_EXTERNAL2), kExecutor));
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(700, kExecutor, Status::NAVIGATION_STATE_AUTO_LAND), kExecutor));
    // Duplicates and older samples are ignored.
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(700, 0, Status::NAVIGATION_STATE_AUTO_LOITER), kExecutor));
    // The operator selects Hold: the executor is no longer in charge.
    const auto exit = control.ObserveVehicleStatus(
        sample(800, 0, Status::NAVIGATION_STATE_AUTO_LOITER), kExecutor);
    ASSERT_TRUE(exit);
    EXPECT_EQ(exit->reason, MissionExitReason::OperatorModeChange);
    EXPECT_EQ(exit->px4_nav_state, Status::NAVIGATION_STATE_AUTO_LOITER);
    EXPECT_EQ(exit->px4_timestamp_us, 800U);
    ASSERT_TRUE(control.LatestNavState());
    EXPECT_EQ(*control.LatestNavState(), Status::NAVIGATION_STATE_AUTO_LOITER);
}

TEST(MissionExit, MonitorClassifiesFailsafeAndNeverReplaysAPreviousRunEdge) {
    MissionControl control;
    control.BeginRun();
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(100, kExecutor, Status::NAVIGATION_STATE_EXTERNAL1), kExecutor));
    const auto failsafe = control.ObserveVehicleStatus(
        sample(200, 0, Status::NAVIGATION_STATE_AUTO_RTL, true), kExecutor);
    ASSERT_TRUE(failsafe);
    EXPECT_EQ(failsafe->reason, MissionExitReason::Failsafe);
    ASSERT_TRUE(control.LatchExit(failsafe->reason, failsafe->px4_nav_state, 200));
    // Latched: later not-in-charge samples are not new exits.
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(300, 0, Status::NAVIGATION_STATE_AUTO_RTL, true), kExecutor));
    control.EndRun();

    // Race R4-like: a new run begins while the monitor still sees samples of
    // the old one. Until an in-charge sample arrives after the new run began,
    // no edge can end it.
    control.BeginRun();
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(400, 0, Status::NAVIGATION_STATE_AUTO_LOITER), kExecutor));
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(500, kExecutor, Status::NAVIGATION_STATE_EXTERNAL1), kExecutor));
    // A PX4 clock regression invalidates confirmation.
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(50, 0, Status::NAVIGATION_STATE_AUTO_LOITER), kExecutor));
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(60, kExecutor, Status::NAVIGATION_STATE_EXTERNAL1), kExecutor));
    EXPECT_TRUE(control.ObserveVehicleStatus(
        sample(70, 0, Status::NAVIGATION_STATE_POSCTL), kExecutor));
}

TEST(MissionExit, OnCableDisarmedExitIsDetectedWithoutArmingState) {
    // Race R5: disarmed on the cable, the ActivateAlways executor stays in
    // charge; an operator mode change still ends the mission.
    MissionControl control;
    control.BeginRun();
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(100, kExecutor, Status::NAVIGATION_STATE_EXTERNAL3), kExecutor));
    const auto exit = control.ObserveVehicleStatus(
        sample(200, 0, Status::NAVIGATION_STATE_POSCTL), kExecutor);
    ASSERT_TRUE(exit);
    EXPECT_EQ(exit->reason, MissionExitReason::OperatorModeChange);
}

TEST(MissionExit, ExitStepsRunOnceInOrderAfterDispatchClosed) {
    MissionControl control;
    control.BeginRun();
    std::vector<std::string> order;
    bool dispatch_closed_first = true;
    iii_drone::mission::MissionExitSteps steps;
    const auto step = [&](const char * name) {
        return [&, name]() {
            dispatch_closed_first = dispatch_closed_first && control.DispatchClosed();
            order.emplace_back(name);
        };
    };
    steps.stop_setpoint_consumption = step("setpoints");
    steps.stop_trees = step("trees");
    steps.complete_executor_action = [&]() {
        order.emplace_back("executor_action");
        throw std::runtime_error("goal already terminal");
    };
    steps.release_consumer_control = step("release");
    steps.stop_side_effects = step("side_effects");
    steps.end_run_bookkeeping = step("bookkeeping");
    std::vector<std::string> errors;
    const auto record = iii_drone::mission::ExecuteMissionExit(
        control, operatorHold(), steps,
        [&errors](const char * name, const std::exception &) { errors.emplace_back(name); });
    ASSERT_TRUE(record);
    EXPECT_TRUE(dispatch_closed_first);
    EXPECT_EQ(order, (std::vector<std::string>{
        "setpoints", "trees", "executor_action", "release", "side_effects", "bookkeeping"}));
    EXPECT_EQ(errors, (std::vector<std::string>{"complete_executor_action"}));
    // A second observer (monitor vs px4_ros2) does not repeat the cleanup.
    order.clear();
    EXPECT_FALSE(iii_drone::mission::ExecuteMissionExit(control, operatorHold(), steps));
    EXPECT_TRUE(order.empty());
}

TEST(MissionExit, ModeCompletionIsIgnoredAfterExitAndDeactivationIsDeferred) {
    MissionControl control;
    control.BeginRun();
    // Normal completions are handled.
    EXPECT_EQ(iii_drone::mission::classifyModeCompletion(control, false, true),
        ModeCompletionHandling::Handle);
    // px4_ros2 reports Deactivated before onDeactivate(reason): defer.
    EXPECT_EQ(iii_drone::mission::classifyModeCompletion(control, true, true),
        ModeCompletionHandling::DeferUntilDeactivation);
    // Not active (completed mission): legacy handling.
    EXPECT_EQ(iii_drone::mission::classifyModeCompletion(control, true, false),
        ModeCompletionHandling::Handle);
    // Race R4: after the exit, handoff completions are echoes; the executor
    // must not schedule the next mode or the mission-done mode over the
    // operator's choice.
    ASSERT_TRUE(control.LatchExit(MissionExitReason::OperatorModeChange, 4, 1));
    EXPECT_EQ(iii_drone::mission::classifyModeCompletion(control, true, true),
        ModeCompletionHandling::IgnoreAfterMissionExit);
    EXPECT_EQ(iii_drone::mission::classifyModeCompletion(control, false, false),
        ModeCompletionHandling::IgnoreAfterMissionExit);
}

TEST(MissionExit, PlMapperIsStoppedOnlyWhenTheMissionLeftItRunning) {
    using Command = iii_drone_interfaces::msg::PLMapperCommand;
    MissionControl control;
    EXPECT_FALSE(control.TakePlMapperExitCommand());
    control.RecordPlMapperCommand(Command::PL_MAPPER_CMD_START);
    ASSERT_TRUE(control.TakePlMapperExitCommand());
    EXPECT_FALSE(control.TakePlMapperExitCommand());
    control.RecordPlMapperCommand(Command::PL_MAPPER_CMD_FREEZE);
    const auto command = control.TakePlMapperExitCommand();
    ASSERT_TRUE(command);
    EXPECT_EQ(*command, Command::PL_MAPPER_CMD_STOP);
    control.RecordPlMapperCommand(Command::PL_MAPPER_CMD_STOP);
    EXPECT_FALSE(control.TakePlMapperExitCommand());
}

TEST(MissionExit, OperatorExitsAreQuietAndFailsafeIsNot) {
    EXPECT_TRUE(iii_drone::mission::isOperatorMissionExit(MissionExitReason::OperatorModeChange));
    EXPECT_TRUE(iii_drone::mission::isOperatorMissionExit(MissionExitReason::OperatorStickOverride));
    EXPECT_FALSE(iii_drone::mission::isOperatorMissionExit(MissionExitReason::Failsafe));
    EXPECT_FALSE(iii_drone::mission::isOperatorMissionExit(MissionExitReason::None));
}

TEST(MissionExit, ObservedHandoverIsRunScopedAndRequestExitRunsTheExecutorProcedure) {
    auto & control = MissionControl::Process();
    control.ResetForTest();
    control.BeginRun();
    EXPECT_FALSE(control.ObservedHandover());
    EXPECT_FALSE(control.ObserveVehicleStatus(
        sample(100, kExecutor, Status::NAVIGATION_STATE_EXTERNAL1), kExecutor));
    ASSERT_TRUE(control.ObserveVehicleStatus(
        sample(200, 0, Status::NAVIGATION_STATE_POSCTL), kExecutor));
    const auto handover = control.ObservedHandover();
    ASSERT_TRUE(handover);
    EXPECT_EQ(handover->px4_nav_state, Status::NAVIGATION_STATE_POSCTL);

    int handled = 0;
    bool gate_free_in_handler = false;
    control.SetExitHandler([&](const MissionExitDecision & decision) {
        ++handled;
        // The dispatching tick released its permit: the handler can latch.
        gate_free_in_handler = !control.DispatchClosed();
        (void)control.LatchExit(decision.reason, decision.px4_nav_state, decision.px4_timestamp_us);
    });
    int ticks = 0;
    const auto result = iii_drone::mission::guardMissionDispatch(
        true, [&ticks]() { ++ticks; return 1; }, []() { return 0; });
    EXPECT_EQ(result, 0);
    EXPECT_EQ(ticks, 0);
    EXPECT_EQ(handled, 1);
    EXPECT_TRUE(gate_free_in_handler);
    EXPECT_TRUE(control.ExitLatched());
    EXPECT_FALSE(control.ObservedHandover());
    // Later ticks are blocked by the closed gate without re-running the exit.
    EXPECT_EQ(iii_drone::mission::guardMissionDispatch(
        true, [&ticks]() { ++ticks; return 1; }, []() { return 0; }), 0);
    EXPECT_EQ(handled, 1);

    // A new run never inherits the previous run's handover.
    control.EndRun();
    control.BeginRun();
    EXPECT_FALSE(control.ObservedHandover());
    EXPECT_EQ(iii_drone::mission::guardMissionDispatch(
        true, [&ticks]() { ++ticks; return 1; }, []() { return 0; }), 1);
    control.ResetForTest();
}
