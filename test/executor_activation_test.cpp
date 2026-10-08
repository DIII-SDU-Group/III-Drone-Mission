#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <iii_drone_mission/mission/mission_catalog.hpp>
#include <iii_drone_mission/mission/mission_specification.hpp>
#include <iii_drone_mission/px4/mode_executors/executor_activation.hpp>

using iii_drone::px4::ActivationArming;
using iii_drone::px4::DecideActivationArming;
using iii_drone::px4::ExecutorActivation;
using Activation = px4_ros2::ModeExecutorBase::Settings::Activation;

namespace {

iii_drone::mission::mission_specification_entry_t Entry(const std::string & key, bool allow_activate_when_disarmed) {
    iii_drone::mission::mission_specification_entry_t entry;
    entry.key = key;
    entry.allow_activate_when_disarmed = allow_activate_when_disarmed;
    return entry;
}

Activation InstalledMissionActivation(const std::string & catalog_id) {
    const auto catalog = iii_drone::mission::MissionCatalog::LoadInstalled();
    const iii_drone::mission::MissionSpecification specification(
        catalog,
        catalog->entry(catalog_id),
        nullptr
    );
    return ExecutorActivation(specification.entries());
}

}  // namespace

TEST(ExecutorActivation, MissionWithoutDisarmedModesActivatesOnlyWhenArmed) {
    EXPECT_EQ(ExecutorActivation({Entry("hover", false)}), Activation::ActivateOnlyWhenArmed);
    EXPECT_EQ(
        ExecutorActivation({Entry("a", false), Entry("b", false)}),
        Activation::ActivateOnlyWhenArmed
    );
}

TEST(ExecutorActivation, AnyModeThatMayRunDisarmedKeepsTheExecutorInChargeWhileDisarmed) {
    EXPECT_EQ(ExecutorActivation({Entry("takeoff", true), Entry("shuttle", false)}), Activation::ActivateAlways);
    // An airborne-only owned mode followed by modes on the cable.
    EXPECT_EQ(
        ExecutorActivation({Entry("inspect", false), Entry("reach_cable", true), Entry("charge", true)}),
        Activation::ActivateAlways
    );
}

TEST(ExecutorActivation, ArmsOnlyWhenTheOwnedModeMayRunDisarmed) {
    EXPECT_EQ(DecideActivationArming(true, false), ActivationArming::ScheduleOwnedMode);
    EXPECT_EQ(DecideActivationArming(true, true), ActivationArming::ScheduleOwnedMode);
    EXPECT_EQ(DecideActivationArming(false, true), ActivationArming::ArmFirst);
    EXPECT_EQ(DecideActivationArming(false, false), ActivationArming::Refuse);
}

TEST(ExecutorActivation, InstalledMissions) {
    // Airborne-only OptiTrack missions: PX4 must be armed by someone else.
    EXPECT_EQ(InstalledMissionActivation("opti-track-hover"), Activation::ActivateOnlyWhenArmed);
    EXPECT_EQ(InstalledMissionActivation("opti-track-maneuvers"), Activation::ActivateOnlyWhenArmed);
    // Missions that take off from the ground themselves.
    EXPECT_EQ(InstalledMissionActivation("opti-track-cycle"), Activation::ActivateAlways);
    EXPECT_EQ(InstalledMissionActivation("opti-track-mode-loop"), Activation::ActivateAlways);
    // The inspection disarms on the cable and re-arms to leave it; its owned
    // mode still refuses a disarmed activation (DecideActivationArming).
    EXPECT_EQ(InstalledMissionActivation("inspection-production"), Activation::ActivateAlways);
    EXPECT_EQ(InstalledMissionActivation("reach-charge-leave-experimental"), Activation::ActivateAlways);
}
