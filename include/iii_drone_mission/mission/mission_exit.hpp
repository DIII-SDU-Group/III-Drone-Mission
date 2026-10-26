#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>

/*****************************************************************************/
// Mission Exit
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    /**
     * Why PX4 took command authority away from the Mission executor.
     *
     * Mission Exit is the one authoritative transition that ends a mission run
     * when PX4 leaves the mission-owned external mode for anything the mission
     * did not initiate itself. Mission-driven transitions (next-mode handoffs,
     * the executor's own land/takeoff/arm/disarm, the mission-done mode) keep
     * the executor in charge and are never a Mission Exit.
     */
    enum class MissionExitReason : uint8_t {
        None = 0,
        // The operator (or a test driver) selected another PX4 mode.
        OperatorModeChange,
        // Stick input made the executor hand control to Position mode.
        OperatorStickOverride,
        // PX4 entered failsafe; a genuine fault that stays loud.
        Failsafe
    };

    const char * missionExitReasonLabel(MissionExitReason reason);

    /** Operator exits are normal operation and log INFO only. */
    bool isOperatorMissionExit(MissionExitReason reason);

    struct MissionExitRecord {
        MissionExitReason reason = MissionExitReason::None;
        uint8_t px4_nav_state = 0;
        uint64_t px4_timestamp_us = 0;
        uint64_t run = 0;
        std::chrono::system_clock::time_point stamp{};
    };

    /** The PX4 vehicle_status fields the Mission Exit monitor needs. */
    struct VehicleControlSample {
        uint64_t timestamp_us = 0;
        uint8_t nav_state = 0;
        uint8_t executor_in_charge = 0;
        bool failsafe = false;
        std::chrono::steady_clock::time_point receipt{};
    };

    struct MissionExitDecision {
        MissionExitReason reason = MissionExitReason::None;
        uint8_t px4_nav_state = 0;
        uint64_t px4_timestamp_us = 0;
    };

    /**
     * Process-wide mission control state shared by the mode executor, the
     * maneuver modes and every behavior-tree node of the Mission process:
     * - the run / Mission Exit latch (first observer wins, once per run);
     * - the dispatch gate: nodes that command the vehicle or its payload hold
     *   a shared permit for their whole dispatching tick, and closing the gate
     *   takes the exclusive lock, so after LatchExit() returns no new goal or
     *   command can leave any tree, and a tick already dispatching completes
     *   first (its goal is then ended through Core's consumer release);
     * - the vehicle_status falling-edge monitor;
     * - the ledger of mission-started side effects that outlive a tree.
     */
    class MissionControl {
    public:
        static MissionControl & Process();

        MissionControl() = default;
        MissionControl(const MissionControl &) = delete;
        MissionControl & operator=(const MissionControl &) = delete;

        /** A new executor run: clears the latch and opens the dispatch gate. */
        uint64_t BeginRun(
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now()
        );

        /** The run ended (normally, by failure, or after a Mission Exit). */
        void EndRun();

        bool RunActive() const;

        /**
         * Latch Mission Exit for the active run and close the dispatch gate.
         * Returns the record only for the first caller of the run.
         */
        std::optional<MissionExitRecord> LatchExit(
            MissionExitReason reason,
            uint8_t px4_nav_state,
            uint64_t px4_timestamp_us
        );

        /** Mission Exit latched since the last BeginRun(). */
        bool ExitLatched() const;

        /** Mission Exit latched and its run not yet ended by the executor. */
        bool ExitInProgress() const;

        std::optional<MissionExitRecord> LastExit() const;

        class DispatchPermit {
        public:
            explicit DispatchPermit(const MissionControl & control);
            bool allowed() const { return allowed_; }
        private:
            std::shared_lock<std::shared_mutex> lock_;
            bool allowed_;
        };

        /** Hold for the whole dispatching tick of a guarded node. */
        DispatchPermit AcquireDispatch() const;

        /** True from a Mission Exit until the next run or standalone mode start. */
        bool DispatchClosed() const;

        /** A mission mode legitimately started outside an exited run. */
        void OpenDispatch();

        /**
         * Falling-edge detector on PX4 vehicle_status. A run is confirmed by
         * an in-charge sample received after the run began; a newer sample
         * whose executor_in_charge is no longer ours is a Mission Exit.
         * Stale samples of a previous run can never end the current one.
         */
        std::optional<MissionExitDecision> ObserveVehicleStatus(
            const VehicleControlSample & sample,
            uint8_t executor_id
        );

        /** Latest PX4 navigation state seen by the monitor, if any. */
        std::optional<uint8_t> LatestNavState() const;

        /**
         * The freshest vehicle_status of this process already shows that PX4
         * took command authority away from the active run, but no observer has
         * latched Mission Exit yet. Consulted by every dispatching tick under
         * the dispatch permit, immediately before a goal would be sent.
         */
        std::optional<MissionExitDecision> ObservedHandover() const;

        using ExitHandler = std::function<void(const MissionExitDecision &)>;

        /** The executor's Mission Exit procedure (run by RequestExit()). */
        void SetExitHandler(ExitHandler handler);
        void ClearExitHandler();

        /**
         * Run the Mission Exit for an observed handover from any thread that
         * does not hold a dispatch permit: the executor's full procedure when
         * registered, otherwise at least the latch (gate closed).
         */
        void RequestExit(const MissionExitDecision & decision);

        /** PL mapper command dispatched by a tree (mission-started side effect). */
        void RecordPlMapperCommand(uint8_t command);

        /**
         * The command that returns the PL mapper to the state a completed
         * Leave Cable leaves it in, if the mission left it otherwise.
         */
        std::optional<uint8_t> TakePlMapperExitCommand();

        /**
         * A mission mode is about to start its tree. Refused while a Mission
         * Exit of the active run is in progress (a stale activation processed
         * after PX4 already left); otherwise allowed, reopening the gate for a
         * standalone mode start after an exited run.
         */
        bool AdmitModeStart();

        /** Tests only: restore the initial state. */
        void ResetForTest();

    private:
        mutable std::shared_mutex dispatch_mutex_;
        bool dispatch_closed_ = false;

        mutable std::mutex state_mutex_;
        uint64_t run_ = 0;
        bool run_active_ = false;
        bool exit_latched_ = false;
        std::chrono::steady_clock::time_point run_started_{};
        std::optional<MissionExitRecord> last_exit_;

        uint64_t last_sample_timestamp_us_ = 0;
        std::optional<uint8_t> latest_nav_state_;
        uint64_t confirmed_run_ = 0;
        uint64_t confirmed_timestamp_us_ = 0;

        std::optional<uint8_t> last_pl_mapper_command_;
        std::optional<MissionExitDecision> observed_handover_;
        uint64_t observed_handover_run_ = 0;

        mutable std::mutex handler_mutex_;
        ExitHandler exit_handler_;
    };

    /**
     * The deterministic Mission Exit cleanup, in order. The dispatch gate is
     * closed (by LatchExit) before any step runs.
     */
    struct MissionExitSteps {
        // Stop setpoint consumption so Core retirement never looks like a
        // reference failure while px4_ros2 has not yet deactivated the mode.
        std::function<void()> stop_setpoint_consumption;
        // Stop every running tree; teardown halts RUNNING nodes (goal cancel).
        std::function<void()> stop_trees;
        // Complete a pending ModeExecutorAction goal (its server rejects cancel).
        std::function<void()> complete_executor_action;
        // Release reference control and ask Core to retire the consumer.
        std::function<void()> release_consumer_control;
        // Stop mission-started side effects that outlive a tree.
        std::function<void()> stop_side_effects;
        // Executor bookkeeping (inactive, schedules, blackboard) and status.
        std::function<void()> end_run_bookkeeping;
    };

    using MissionExitStepError = std::function<void(const char * step, const std::exception & error)>;

    /**
     * Latch Mission Exit and run the steps once per run (first observer wins).
     * A failing step is reported and the remaining steps still run.
     */
    std::optional<MissionExitRecord> ExecuteMissionExit(
        MissionControl & control,
        const MissionExitDecision & decision,
        const MissionExitSteps & steps,
        const MissionExitStepError & on_step_error = {}
    );

    /** How the executor treats a completion callback of a scheduled mode. */
    enum class ModeCompletionHandling {
        // Normal (legacy) handling.
        Handle,
        // A Mission Exit of this run was latched: the completion is its echo.
        IgnoreAfterMissionExit,
        // Deactivated while running: px4_ros2 cancels the scheduled mode before
        // it calls onDeactivate(reason) in the same callback. Decide after it.
        DeferUntilDeactivation
    };

    /**
     * Run a guarded node's tick. A dispatching (IDLE) tick holds the dispatch
     * permit for its whole duration and is replaced by on_blocked() after a
     * Mission Exit; ticks of a RUNNING node (result/halt handling) are never
     * blocked.
     */
    template <typename TickFn, typename BlockedFn>
    auto guardMissionDispatch(bool dispatching_tick, TickFn && tick, BlockedFn && on_blocked)
        -> decltype(tick()) {
        if (!dispatching_tick) {
            return tick();
        }
        auto & control = MissionControl::Process();
        std::optional<MissionExitDecision> handover;
        {
            const auto permit = control.AcquireDispatch();
            if (!permit.allowed()) {
                return on_blocked();
            }
            // Handover check at the last moment before sending: the freshest
            // PX4 vehicle_status of this process may already show the
            // executor out of charge before any observer latched the exit.
            handover = control.ObservedHandover();
            if (!handover) {
                return tick();
            }
        }
        // Outside the permit: the exit closes the gate exclusively.
        control.RequestExit(*handover);
        return on_blocked();
    }

    /** A failure observed by a tree node is the echo of a Mission Exit. */
    inline bool missionExitClosedDispatch() {
        return MissionControl::Process().DispatchClosed();
    }

    ModeCompletionHandling classifyModeCompletion(
        const MissionControl & control,
        bool deactivated_result,
        bool executor_active
    );

} // namespace mission
} // namespace iii_drone
