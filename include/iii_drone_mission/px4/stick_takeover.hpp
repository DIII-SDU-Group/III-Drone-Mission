#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <optional>

namespace iii_drone::px4 {

/**
 * Decides when manual stick input is a pilot takeover.
 *
 * A takeover is stick movement: some axis moved by more than the threshold
 * from where it was when the executor became active. A stick that rests away
 * from centre is no takeover; PX4 reports throttle -1 with the stick at the
 * bottom. Samples PX4 marks invalid are ignored, and so are axes without data
 * (NaN).
 */
class StickTakeoverDetector final {
public:
    struct Sample {
        bool valid = false;
        float roll = NAN;
        float pitch = NAN;
        float yaw = NAN;
        float throttle = NAN;
    };

    /**
     * The executor became active: the latest valid stick positions become the
     * baseline. An axis without one takes its first valid value after this.
     */
    void Rebaseline() {
        std::lock_guard<std::mutex> lock(mutex_);
        baseline_ = latest_;
    }

    /** Records a sample while the executor is inactive. */
    void Observe(const Sample & sample) {
        std::lock_guard<std::mutex> lock(mutex_);
        record(sample, std::nullopt);
    }

    /**
     * Records a sample while the executor is active; true when it moved any
     * axis by more than threshold from the baseline.
     */
    bool ObserveActive(const Sample & sample, double threshold) {
        std::lock_guard<std::mutex> lock(mutex_);
        return record(sample, threshold);
    }

private:
    static constexpr std::size_t kAxes = 4;

    std::mutex mutex_;
    std::array<std::optional<float>, kAxes> latest_{};
    std::array<std::optional<float>, kAxes> baseline_{};

    bool record(const Sample & sample, std::optional<double> threshold) {
        if (!sample.valid) {
            return false;
        }
        const std::array<float, kAxes> axes = {sample.roll, sample.pitch, sample.yaw, sample.throttle};
        bool moved = false;
        for (std::size_t axis = 0; axis < kAxes; ++axis) {
            const float value = axes[axis];
            if (!std::isfinite(value)) {
                continue;
            }
            latest_[axis] = value;
            if (!threshold) {
                continue;
            }
            if (!baseline_[axis]) {
                baseline_[axis] = value;
                continue;
            }
            if (std::abs(static_cast<double>(value) - static_cast<double>(*baseline_[axis])) > *threshold) {
                moved = true;
            }
        }
        return moved;
    }
};

}  // namespace iii_drone::px4
