#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace iii_drone::px4 {

/**
 * Repeats a mode's completion report until its mode executor has it.
 *
 * px4_ros2::ModeBase::completed() publishes one ModeCompleted to PX4, and the
 * executor learns of it only from PX4's echo; both legs cross the best-effort
 * PX4 bridge. One lost datagram left the executor waiting forever (HIL
 * qualification hil-20261002T111516Z: Cable Charging completed, Leave Cable
 * never started). The report is repeated while it is unacknowledged and the
 * mode is still active under an executor. Duplicates are harmless: PX4 only
 * forwards ModeCompleted, and the executor matches only the mode it has
 * scheduled and stops listening once it has the first one.
 */
class CompletionReport final {
public:
    using Clock = std::chrono::steady_clock;

    static constexpr std::chrono::milliseconds kRepeatInterval{500};
    static constexpr std::chrono::seconds kWarnAfter{5};

    struct Repeat {
        uint8_t result;
        unsigned count;
        Clock::duration since_report;
        bool warn;
    };

    /** A new activation: ModeBase accepts a new completion. */
    void Reset() noexcept {
        reported_ = false;
        pending_ = false;
        warned_ = false;
        repeats_ = 0;
    }

    /** Records the activation's first report; later ones are ignored, as by ModeBase. */
    bool Report(uint8_t result, Clock::time_point now) noexcept {
        if (reported_) {
            return false;
        }
        reported_ = true;
        pending_ = true;
        result_ = result;
        reported_at_ = now;
        last_sent_at_ = now;
        return true;
    }

    /** The executor has the completion, or PX4 moved the mode off. */
    void Acknowledge() noexcept { pending_ = false; }

    bool pending() const noexcept { return pending_; }

    /** A repeat is due one interval after the last send while still unacknowledged. */
    std::optional<Repeat> Due(Clock::time_point now, bool mode_active, bool executor_in_charge) noexcept {
        if (!pending_ || !mode_active || !executor_in_charge || now - last_sent_at_ < kRepeatInterval) {
            return std::nullopt;
        }
        last_sent_at_ = now;
        ++repeats_;
        const Clock::duration since_report = now - reported_at_;
        const bool warn = !warned_ && since_report >= kWarnAfter;
        warned_ = warned_ || warn;
        return Repeat{result_, repeats_, since_report, warn};
    }

private:
    bool reported_ = false;
    bool pending_ = false;
    bool warned_ = false;
    uint8_t result_ = 0;
    unsigned repeats_ = 0;
    Clock::time_point reported_at_{};
    Clock::time_point last_sent_at_{};
};

} // namespace iii_drone::px4
