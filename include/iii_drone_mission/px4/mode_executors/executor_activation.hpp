#pragma once

#include <vector>

#include <px4_ros2/components/mode_executor.hpp>

#include <iii_drone_mission/mission/mission_specification.hpp>

namespace iii_drone::px4 {

/**
 * A mission arms the vehicle only where its specification lets a mode run
 * disarmed (allow_activate_when_disarmed).
 *
 * px4_ros2 keeps an ActivateOnlyWhenArmed executor in charge only while the
 * vehicle is armed: selecting the owned mode while disarmed leaves the
 * executor inactive (it activates once something else arms the vehicle), and
 * a disarm deactivates it. A mission none of whose modes may run disarmed
 * registers that way. A mission with a mode that runs disarmed, such as
 * charging on the cable between disarm and re-arm, needs ActivateAlways to
 * stay in charge through that mode; onActivate then decides with
 * DecideActivationArming() whether it may arm.
 */
inline px4_ros2::ModeExecutorBase::Settings::Activation ExecutorActivation(
    const std::vector<iii_drone::mission::mission_specification_entry_t> & entries
) {
    for (const auto & entry : entries) {
        if (entry.allow_activate_when_disarmed) {
            return px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways;
        }
    }
    return px4_ros2::ModeExecutorBase::Settings::Activation::ActivateOnlyWhenArmed;
}

/// What the mode executor does with the vehicle's arming state on activation.
enum class ActivationArming {
    ScheduleOwnedMode,  ///< Armed: schedule the owned mode.
    ArmFirst,           ///< Disarmed, and the owned mode may run disarmed: arm, then schedule it.
    Refuse,             ///< Disarmed, and the owned mode may not run disarmed: never arm.
};

inline ActivationArming DecideActivationArming(bool armed, bool owned_mode_allows_activation_when_disarmed) {
    if (armed) {
        return ActivationArming::ScheduleOwnedMode;
    }
    return owned_mode_allows_activation_when_disarmed ? ActivationArming::ArmFirst : ActivationArming::Refuse;
}

}  // namespace iii_drone::px4
