#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

namespace iii_drone::px4 {

/**
 * PX4 failsafe deferral scoped to mode handoffs.
 *
 * The mode executor defers PX4 failsafes from the moment it schedules a mode
 * until PX4 runs that mode, never across a whole mission: a failsafe raised
 * while a mission mode runs must act at once. This tracks the pending
 * handoff; the executor enables and releases the deferral in PX4.
 */
class HandoffFailsafeDeferral final {
public:
    /// Seconds PX4 may defer each failsafe while a handoff is pending.
    static constexpr int kTimeoutS = 5;

    /// A handoff to the mode with this nav_state starts (or replaces the
    /// pending one); the executor enables the deferral.
    void Begin(uint8_t target_nav_state) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_nav_state_ = target_nav_state;
    }

    /// PX4 reported nav_state; target_running tells whether the mode with
    /// that nav_state runs (a mission mode runs once it activated, e.g. not
    /// while the executor still arms). True once, when the pending handoff's
    /// target runs: the executor releases the deferral.
    bool Reached(uint8_t nav_state, bool target_running) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!target_running || !target_nav_state_ || *target_nav_state_ != nav_state) {
            return false;
        }
        target_nav_state_.reset();
        return true;
    }

    /// The handoff ends without reaching its target, e.g. the run ended. True
    /// when a handoff was pending: the executor releases the deferral.
    bool Abandon() {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool pending = target_nav_state_.has_value();
        target_nav_state_.reset();
        return pending;
    }

    bool pending() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return target_nav_state_.has_value();
    }

private:
    mutable std::mutex mutex_;
    std::optional<uint8_t> target_nav_state_;
};

}  // namespace iii_drone::px4
