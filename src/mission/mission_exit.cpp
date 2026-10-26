/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_exit.hpp>

#include <iii_drone_interfaces/msg/pl_mapper_command.hpp>

using namespace iii_drone::mission;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

const char * iii_drone::mission::missionExitReasonLabel(MissionExitReason reason) {
    switch (reason) {
        case MissionExitReason::OperatorModeChange:
            return "operator mode change";
        case MissionExitReason::OperatorStickOverride:
            return "operator stick override";
        case MissionExitReason::Failsafe:
            return "PX4 failsafe";
        case MissionExitReason::None:
        default:
            return "none";
    }
}

bool iii_drone::mission::isOperatorMissionExit(MissionExitReason reason) {
    return reason == MissionExitReason::OperatorModeChange ||
        reason == MissionExitReason::OperatorStickOverride;
}

MissionControl & MissionControl::Process() {
    static MissionControl control;
    return control;
}

uint64_t MissionControl::BeginRun(std::chrono::steady_clock::time_point now) {
    uint64_t run = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        run = ++run_;
        run_active_ = true;
        exit_latched_ = false;
        run_started_ = now;
        observed_handover_.reset();
        observed_handover_run_ = 0;
    }
    std::unique_lock<std::shared_mutex> dispatch_lock(dispatch_mutex_);
    dispatch_closed_ = false;
    return run;
}

void MissionControl::EndRun() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    run_active_ = false;
}

bool MissionControl::RunActive() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return run_active_;
}

std::optional<MissionExitRecord> MissionControl::LatchExit(
    MissionExitReason reason,
    uint8_t px4_nav_state,
    uint64_t px4_timestamp_us
) {
    MissionExitRecord record;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!run_active_ || exit_latched_) {
            return std::nullopt;
        }
        exit_latched_ = true;
        record.reason = reason;
        record.px4_nav_state = px4_nav_state;
        record.px4_timestamp_us = px4_timestamp_us;
        record.run = run_;
        record.stamp = std::chrono::system_clock::now();
        last_exit_ = record;
    }
    // Never hold state_mutex_ here: a dispatching tick holds a shared permit
    // and may record a side effect under state_mutex_.
    std::unique_lock<std::shared_mutex> dispatch_lock(dispatch_mutex_);
    dispatch_closed_ = true;
    return record;
}

bool MissionControl::ExitLatched() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return exit_latched_;
}

bool MissionControl::ExitInProgress() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return run_active_ && exit_latched_;
}

std::optional<MissionExitRecord> MissionControl::LastExit() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return last_exit_;
}

MissionControl::DispatchPermit::DispatchPermit(const MissionControl & control)
: lock_(control.dispatch_mutex_),
  allowed_(!control.dispatch_closed_) {}

MissionControl::DispatchPermit MissionControl::AcquireDispatch() const {
    return DispatchPermit(*this);
}

bool MissionControl::DispatchClosed() const {
    std::shared_lock<std::shared_mutex> lock(dispatch_mutex_);
    return dispatch_closed_;
}

void MissionControl::OpenDispatch() {
    std::unique_lock<std::shared_mutex> lock(dispatch_mutex_);
    dispatch_closed_ = false;
}

std::optional<MissionExitDecision> MissionControl::ObserveVehicleStatus(
    const VehicleControlSample & sample,
    uint8_t executor_id
) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (sample.timestamp_us == 0) {
        return std::nullopt;
    }
    if (sample.timestamp_us < last_sample_timestamp_us_) {
        // PX4 clock regression (reboot/replay): no earlier edge is valid.
        confirmed_run_ = 0;
        confirmed_timestamp_us_ = 0;
    } else if (sample.timestamp_us == last_sample_timestamp_us_) {
        return std::nullopt;
    }
    last_sample_timestamp_us_ = sample.timestamp_us;
    latest_nav_state_ = sample.nav_state;
    if (executor_id == 0) {
        return std::nullopt;
    }

    const bool in_charge = sample.executor_in_charge == executor_id;
    if (in_charge) {
        if (run_active_ && sample.receipt > run_started_) {
            confirmed_run_ = run_;
            confirmed_timestamp_us_ = sample.timestamp_us;
        }
        return std::nullopt;
    }
    if (!run_active_ || exit_latched_ || confirmed_run_ != run_ ||
        sample.timestamp_us <= confirmed_timestamp_us_) {
        return std::nullopt;
    }
    MissionExitDecision decision;
    decision.reason = sample.failsafe
        ? MissionExitReason::Failsafe
        : MissionExitReason::OperatorModeChange;
    decision.px4_nav_state = sample.nav_state;
    decision.px4_timestamp_us = sample.timestamp_us;
    // Visible to every dispatching tick before the exit procedure latches.
    observed_handover_ = decision;
    observed_handover_run_ = run_;
    return decision;
}

std::optional<MissionExitDecision> MissionControl::ObservedHandover() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!run_active_ || exit_latched_ || !observed_handover_ ||
        observed_handover_run_ != run_) {
        return std::nullopt;
    }
    return observed_handover_;
}

void MissionControl::SetExitHandler(ExitHandler handler) {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    exit_handler_ = std::move(handler);
}

void MissionControl::ClearExitHandler() {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    exit_handler_ = nullptr;
}

void MissionControl::RequestExit(const MissionExitDecision & decision) {
    {
        std::lock_guard<std::mutex> lock(handler_mutex_);
        if (exit_handler_) {
            exit_handler_(decision);
        }
    }
    if (!ExitLatched()) {
        (void)LatchExit(decision.reason, decision.px4_nav_state, decision.px4_timestamp_us);
    }
}

std::optional<uint8_t> MissionControl::LatestNavState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_nav_state_;
}

void MissionControl::RecordPlMapperCommand(uint8_t command) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_pl_mapper_command_ = command;
}

std::optional<uint8_t> MissionControl::TakePlMapperExitCommand() {
    using Command = iii_drone_interfaces::msg::PLMapperCommand;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!last_pl_mapper_command_ ||
        *last_pl_mapper_command_ == Command::PL_MAPPER_CMD_STOP) {
        return std::nullopt;
    }
    last_pl_mapper_command_ = Command::PL_MAPPER_CMD_STOP;
    return Command::PL_MAPPER_CMD_STOP;
}

bool MissionControl::AdmitModeStart() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (run_active_ && exit_latched_) {
            return false;
        }
    }
    OpenDispatch();
    return true;
}

void MissionControl::ResetForTest() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        run_ = 0;
        run_active_ = false;
        exit_latched_ = false;
        run_started_ = {};
        last_exit_.reset();
        last_sample_timestamp_us_ = 0;
        latest_nav_state_.reset();
        confirmed_run_ = 0;
        confirmed_timestamp_us_ = 0;
        last_pl_mapper_command_.reset();
        observed_handover_.reset();
        observed_handover_run_ = 0;
    }
    ClearExitHandler();
    std::unique_lock<std::shared_mutex> lock(dispatch_mutex_);
    dispatch_closed_ = false;
}

std::optional<MissionExitRecord> iii_drone::mission::ExecuteMissionExit(
    MissionControl & control,
    const MissionExitDecision & decision,
    const MissionExitSteps & steps,
    const MissionExitStepError & on_step_error
) {
    auto record = control.LatchExit(
        decision.reason, decision.px4_nav_state, decision.px4_timestamp_us);
    if (!record) {
        return std::nullopt;
    }
    const auto run_step = [&on_step_error](const char * name, const std::function<void()> & step) {
        if (!step) {
            return;
        }
        try {
            step();
        } catch (const std::exception & error) {
            if (on_step_error) {
                on_step_error(name, error);
            }
        }
    };
    run_step("stop_setpoint_consumption", steps.stop_setpoint_consumption);
    run_step("stop_trees", steps.stop_trees);
    run_step("complete_executor_action", steps.complete_executor_action);
    run_step("release_consumer_control", steps.release_consumer_control);
    run_step("stop_side_effects", steps.stop_side_effects);
    run_step("end_run_bookkeeping", steps.end_run_bookkeeping);
    return record;
}

ModeCompletionHandling iii_drone::mission::classifyModeCompletion(
    const MissionControl & control,
    bool deactivated_result,
    bool executor_active
) {
    if (control.ExitLatched()) {
        return ModeCompletionHandling::IgnoreAfterMissionExit;
    }
    if (deactivated_result && executor_active && control.RunActive()) {
        return ModeCompletionHandling::DeferUntilDeactivation;
    }
    return ModeCompletionHandling::Handle;
}
