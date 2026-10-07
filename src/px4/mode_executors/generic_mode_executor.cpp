/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/mode_executors/generic_mode_executor.hpp>

#include <chrono>
#include <cmath>

#include <iii_drone_core/diagnostics/hil_trace.hpp>

using namespace iii_drone::px4;
using iii_drone::mission::MissionControl;
using iii_drone::mission::MissionExitDecision;
using iii_drone::mission::MissionExitReason;
using namespace iii_drone::mission;
using namespace iii_drone::configuration;
using namespace iii_drone::adapters;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GenericModeExecutor::GenericModeExecutor(
    px4_ros2::ModeBase & owned_mode,
    std::string mode_executor_name,
    MissionSpecification::SharedPtr mission_specification,
    ModeProvider::SharedPtr mode_provider,
    Configuration::SharedPtr parameters
) : ModeExecutorBase(
    *mode_provider->mode_node(), 
    px4_ros2::ModeExecutorBase::Settings{.activation=ExecutorActivation(mission_specification->entries())}, 
    owned_mode,
    "/"
),  node_(*mode_provider->mode_node()),
    mode_executor_name_(mode_executor_name),
    mission_specification_(mission_specification),
    configuration_(parameters),
    mode_provider_(mode_provider),
    combined_drone_awareness_adapter_history_(1) {

    RCLCPP_DEBUG(node_.get_logger(), "GenericModeExecutor::GenericModeExecutor(): Initializing mode executor %s", mode_executor_name_.c_str());

    if (ExecutorActivation(mission_specification_->entries()) ==
        px4_ros2::ModeExecutorBase::Settings::Activation::ActivateOnlyWhenArmed) {
        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::GenericModeExecutor(): No mode of mission %s may run disarmed; "
            "the mode executor activates only while the vehicle is armed.",
            mission_specification_->catalog_id().c_str()
        );
    }

	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    manual_control_setpoint_sub_ = node_.create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        "/fmu/out/manual_control_setpoint",
        px4_sub_qos,
        std::bind(
            &GenericModeExecutor::manualControlSetpointCallback,
            this,
            std::placeholders::_1
        )
    );

    mode_executor_action_server_ = rclcpp_action::create_server<ModeExecutorAction>(
        &node_,
        "/mission/mode_executor/action",
        std::bind(
            &GenericModeExecutor::modeExecutorActionGoalCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ), 
        [this](const std::shared_ptr<GoalHandleModeExecutorAction>) -> rclcpp_action::CancelResponse {
            return rclcpp_action::CancelResponse::REJECT;
        },
        std::bind(
            &GenericModeExecutor::modeExecutorActionAcceptedCallback,
            this,
            std::placeholders::_1
        )
    );

    // Ends the failsafe deferral of a mode handoff once PX4 runs the scheduled
    // mode; on the default callback group, where the handoffs are scheduled.
    handoff_vehicle_status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        rclcpp::QoS(1).best_effort(),
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            onHandoffVehicleStatus(msg);
        }
    );

    combined_drone_awareness_sub_ = node_.create_subscription<iii_drone_interfaces::msg::CombinedDroneAwareness>(
        "/control/maneuver_controller/combined_drone_awareness",
        10,
        [this](const iii_drone_interfaces::msg::CombinedDroneAwareness::SharedPtr msg) {
            CombinedDroneAwarenessAdapter adapter(*msg);
            combined_drone_awareness_adapter_history_.Store(adapter);
        }

    );

    // Mission Exit monitor: an independent callback group so the falling edge
    // of executor_in_charge is observed even while px4_ros2's own
    // vehicle-status callback is blocked in a synchronous PX4 command.
    mission_exit_callback_group_ = node_.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false
    );
    rclcpp::SubscriptionOptions mission_exit_options;
    mission_exit_options.callback_group = mission_exit_callback_group_;
    mission_exit_vehicle_status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        rclcpp::QoS(1).best_effort().transient_local(),
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            onMissionExitVehicleStatus(msg);
        },
        mission_exit_options
    );
    pl_mapper_command_client_ = node_.create_client<iii_drone_interfaces::srv::PLMapperCommand>(
        "/perception/pl_mapper/pl_mapper_command",
        rclcpp::ServicesQoS(),
        mission_exit_callback_group_
    );

    // Dedicated, never-shared executor thread: the multi-threaded Mission
    // executor can be saturated by blocking PX4 commands, tree service calls
    // and reference callbacks; this sample must be as fresh as Core's.
    mission_exit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    mission_exit_executor_->add_callback_group(
        mission_exit_callback_group_, node_.get_node_base_interface());
    mission_exit_thread_ = std::thread([executor = mission_exit_executor_]() {
        executor->spin();
    });

    MissionControl::Process().SetExitHandler(
        [this](const MissionExitDecision & decision) {
            triggerMissionExit(decision);
        }
    );

}
    
GenericModeExecutor::~GenericModeExecutor() {

    MissionControl::Process().ClearExitHandler();
    if (mission_exit_executor_) {
        mission_exit_executor_->cancel();
    }
    if (mission_exit_thread_.joinable()) {
        mission_exit_thread_.join();
    }
    if (mission_exit_executor_ && mission_exit_callback_group_) {
        mission_exit_executor_->remove_callback_group(mission_exit_callback_group_);
    }

}

void GenericModeExecutor::onActivate() {

    RCLCPP_INFO(node_.get_logger(), "GenericModeExecutor::onActivate(): Activating mode executor %s", mode_executor_name_.c_str());

    // A new mission run: clears any earlier Mission Exit latch and reopens
    // the tree dispatch gate before any mode of this run can start.
    MissionControl::Process().BeginRun();
    stick_takeover_detector_.Rebaseline();
    is_active_ = true;
    triggered_position_control_ = false;
    std::string owned_mode_key = mission_specification_->executor_owned_mode();
    // Before current_mode_ changes: an intent validated against the new mode
    // then always lands in the new activation.
    mode_provider_->BeginModeActivation(owned_mode_key);
    current_mode_ = mode_provider_->GetMode(owned_mode_key);
    current_mode_entry_ = mission_specification_->GetMissionSpecificationEntry(owned_mode_key);

    schedule_next_ = schedule_next_mode;
    schedule_current_ = schedule_next_mode;

    const ActivationArming arming = DecideActivationArming(
        isArmed(),
        (*current_mode_entry_).allow_activate_when_disarmed
    );

    if (arming == ActivationArming::Refuse) {

        // The mission arms the vehicle only where its specification allows.
        RCLCPP_ERROR(
            node_.get_logger(),
            "GenericModeExecutor::onActivate(): The vehicle is disarmed and mode %s does not allow "
            "activation while disarmed; not arming. Arm (and take off) before selecting the mission.",
            (*current_mode_entry_).mode_name.c_str()
        );

        is_active_ = false;
        clearGlobalBlackboard("mission activation refused while disarmed");
        releaseHandoffFailsafeDeferral("mission activation refused while disarmed");

        return;

    }

    deferFailsafesForHandoff((*current_mode_)->id(), "mission activation");

    if (arming == ActivationArming::ScheduleOwnedMode) {

        const bool force_disarmed_activation =
            !isArmed() && (*current_mode_entry_).allow_activate_when_disarmed;

        scheduleMode(
            (*current_mode_)->id(),
            [this](px4_ros2::Result result) {
                onModeCompleted(result);
            },
            force_disarmed_activation
        );

    } else {

        RCLCPP_INFO(
            node_.get_logger(), 
            "GenericModeExecutor::onActivate(): Arming."
        );

        arm(
            [this](px4_ros2::Result result) {

                if (result != px4_ros2::Result::Success) {

                    RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::onActivate(): Arming failed, deactivating mode executor %s", mode_executor_name_.c_str());

                    is_active_ = false;
                    clearGlobalBlackboard("mission executor activation arming failed");
                    releaseHandoffFailsafeDeferral("mission activation arming failed");

                    return;

                }

                scheduleMode(
                    (*current_mode_)->id(),
                    [this](px4_ros2::Result result) {
                        onModeCompleted(result);
                    }
                );

            }
        );

    }
}

void GenericModeExecutor::onDeactivate(DeactivateReason reason) {

    auto & mission_control = MissionControl::Process();
    if (is_active_ && mission_control.RunActive() && !mission_control.ExitLatched()) {
        // px4_ros2 observed the loss of command authority before the Mission
        // Exit monitor did: same transition, reason from px4_ros2.
        MissionExitDecision decision;
        decision.reason = reason == DeactivateReason::FailsafeActivated
            ? MissionExitReason::Failsafe
            : MissionExitReason::OperatorModeChange;
        decision.px4_nav_state = mission_control.LatestNavState().value_or(0);
        triggerMissionExit(decision);
    }
    const bool mission_exit = mission_control.ExitLatched();
    mission_control.EndRun();

    is_active_ = false;
    clearGlobalBlackboard("mission executor deactivated");
    handoff_failsafe_deferral_.Abandon();
    try {
        deferFailsafesSync(false, 0);
    } catch (const std::exception & exception) {
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::onDeactivate(): Failed while clearing PX4 failsafe deferral: %s",
            exception.what()
        );
    }
    if (mission_exit) {
        RCLCPP_INFO(node_.get_logger(), "GenericModeExecutor::onDeactivate(): Deactivating mode executor %s after Mission Exit", mode_executor_name_.c_str());
        return;
    }
    switch(reason) {
        case DeactivateReason::FailsafeActivated:
            RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::onDeactivate(): Deactivating mode executor %s because failsafe activated.", mode_executor_name_.c_str());
            break;
        case DeactivateReason::Other:
            RCLCPP_INFO(node_.get_logger(), "GenericModeExecutor::onDeactivate(): Deactivating mode executor %s", mode_executor_name_.c_str());
            break;
    }

}

bool GenericModeExecutor::active() const {

    return is_active_;

}

std::string GenericModeExecutor::current_mode_key() const {

    const auto current_mode = current_mode_.Load();
    if (current_mode == nullptr) {
        return "";
    }
    return current_mode->mode_key();

}

void GenericModeExecutor::clearGlobalBlackboard(const std::string & reason) {
    if (!mode_provider_) {
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::clearGlobalBlackboard(): Mode provider is not available. Reason: %s",
            reason.c_str()
        );
        return;
    }

    mode_provider_->ClearGlobalBlackboard(reason);
}

void GenericModeExecutor::onModeCompleted(px4_ros2::Result result) {

    // This executor now has the scheduled mode's completion (or its
    // cancellation): the mode stops repeating its completion report.
    const ManeuverMode::SharedPtr completed_mode = current_mode_.Load();
    if (completed_mode != nullptr) {
        completed_mode->AcknowledgeCompletion();
    }

    switch (iii_drone::mission::classifyModeCompletion(
                MissionControl::Process(),
                result == px4_ros2::Result::Deactivated,
                is_active_)) {
        case iii_drone::mission::ModeCompletionHandling::IgnoreAfterMissionExit:
            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onModeCompleted(): %s ended with result %s after Mission Exit; nothing to schedule",
                getModeName(schedule_current_).c_str(),
                px4_ros2::resultToString(result)
            );
            return;
        case iii_drone::mission::ModeCompletionHandling::DeferUntilDeactivation:
            // px4_ros2 cancels the scheduled mode before it calls
            // onDeactivate(reason) in the same vehicle-status callback. Decide
            // once that callback has returned (same callback group): a Mission
            // Exit then owns the cleanup; otherwise the legacy handling runs.
            if (deferred_deactivation_timer_) {
                deferred_deactivation_timer_->cancel();
            }
            deferred_deactivation_timer_ = node_.create_wall_timer(
                std::chrono::milliseconds(0),
                [this, result]() {
                    if (deferred_deactivation_timer_) {
                        deferred_deactivation_timer_->cancel();
                    }
                    if (MissionControl::Process().ExitLatched()) {
                        return;
                    }
                    handleModeCompleted(result);
                }
            );
            return;
        case iii_drone::mission::ModeCompletionHandling::Handle:
        default:
            break;
    }

    handleModeCompleted(result);

}

void GenericModeExecutor::handleModeCompleted(px4_ros2::Result result) {

    if (!checkScheduleAndActionValidity()) return;

    std::string mode_name = getModeName(schedule_current_);

    logModeCompleted(
        mode_name,
        result
    );

    if (checkPositionControlTriggered()) return;

    bool action_succeeded = true;

    if (
        !checkNextModeSucceeded(
            result, 
            action_succeeded
        )
    ) return;

    schedule_t previous_schedule_current;

    if (scheduleActionIfAny(previous_schedule_current)) return;

    if (previous_schedule_current == schedule_next_mode) {

        bool last_mode;
        onNormalModeSuccess(last_mode);

        if (last_mode) return;

    } else {

        (*current_mode_)->RegisterOnNextActivateCallback(
            [this,action_succeeded](){
                if (action_succeeded) {
                    tryCompleteActionGoal(true);
                } else {
                    tryCompleteActionGoal(false);
                }
            }
        );
    }

    // if (!isArmed()) {
    
    //     RCLCPP_INFO(
    //         node_.get_logger(), 
    //         "GenericModeExecutor::onModeCompleted(): Arming before next mode."
    //     );

    //     arm(
    //         [this](px4_ros2::Result result) {

    //             if (result != px4_ros2::Result::Success) {

    //                 RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::onModeCompleted(): Arming failed, deactivating mode executor %s", mode_executor_name_.c_str());

    //                 is_active_ = false;

    //                 return;

    //             }

    //             RCLCPP_INFO(
    //                 node_.get_logger(),
    //                 "GenericModeExecutor::onModeCompleted(): Activating mode %s.",
    //                 (*current_mode_entry_).mode_name.c_str()
    //             );

    //             scheduleMode(
    //                 (*current_mode_)->id(),
    //                 [this](px4_ros2::Result result) {
    //                     onModeCompleted(result);
    //                 }
    //             );
    //         }
    //     );

    // }  else {

        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::onModeCompleted(): Activating mode %s.",
            (*current_mode_entry_).mode_name.c_str()
        );

        const bool force_disarmed_activation =
            !isArmed() && (*current_mode_entry_).allow_activate_when_disarmed;

        deferFailsafesForHandoff((*current_mode_)->id(), "mission mode handoff");

        scheduleMode(
            (*current_mode_)->id(),
            [this](px4_ros2::Result result) {
                onModeCompleted(result);
            },
            force_disarmed_activation
        );

    // }

}

bool GenericModeExecutor::checkScheduleAndActionValidity() {

    RCLCPP_DEBUG(
        node_.get_logger(), 
        "GenericModeExecutor::checkScheduleAndActionValidity()"
    );

    std::string schedule_next_name = getModeName(schedule_next_);
    std::string schedule_current_name = getModeName(schedule_current_);
        
    if (schedule_next_ != schedule_next_mode || schedule_current_ != schedule_next_mode) {

        if ((*current_goal_handle_) == nullptr) {

            RCLCPP_ERROR(
                node_.get_logger(), 
                "GenericModeExecutor::checkScheduleAndActionValidity(): Mode executor action executing but no goal handle found. Schedule next: %s, schedule current: %s",
                schedule_next_name.c_str(),
                schedule_current_name.c_str()
            );

            is_active_ = false;
            clearGlobalBlackboard("mission executor invalid action state: action schedule without goal handle");
            releaseHandoffFailsafeDeferral("invalid mode executor action state");

            stopModeIfWaiting();

            return false;

        }

    } else {

        if ((*current_goal_handle_) != nullptr) {

            RCLCPP_ERROR(
                node_.get_logger(), 
                "GenericModeExecutor::checkScheduleAndActionValidity(): Mode executor action not executing but goal handle found. Schedule next: %s, schedule current: %s",
                schedule_next_name.c_str(),
                schedule_current_name.c_str()
            );

            is_active_ = false;
            clearGlobalBlackboard("mission executor invalid action state: goal handle without action schedule");
            releaseHandoffFailsafeDeferral("invalid mode executor action state");

            stopModeIfWaiting();

            return false;

        }
    }

    return true;

}

void GenericModeExecutor::logModeCompleted(
    std::string mode_name,
    px4_ros2::Result result
) {

    if (schedule_current_ == schedule_next_mode && schedule_next_ != schedule_next_mode) {

        RCLCPP_INFO(
            node_.get_logger(), 
            "GenericModeExecutor::logModeCompleted(): Mode %s temporarily deactivated with result %s to run mode executor action.", 
            mode_name.c_str(),
            px4_ros2::resultToString(result)
        );

    } else {

        RCLCPP_INFO(
            node_.get_logger(), 
            "GenericModeExecutor::logModeCompleted(): Mode %s completed with result %s", 
            mode_name.c_str(),
            px4_ros2::resultToString(result)
        );

    }
}

bool GenericModeExecutor::checkPositionControlTriggered() {

    if (triggered_position_control_) {
        triggered_position_control_ = false;

        RCLCPP_WARN(
            node_.get_logger(), 
            "GenericModeExecutor::checkPositionControlTriggered(): Position control triggered during mode %s, deactivating mode executor %s", 
            (*current_mode_)->mode_name().c_str(),
            mode_executor_name_.c_str()
        );

        is_active_ = false;
        clearGlobalBlackboard("manual position control triggered");
        releaseHandoffFailsafeDeferral("manual position control triggered");

        stopModeIfWaiting();

        return true;
    }

    return false;

}

bool GenericModeExecutor::checkNextModeSucceeded(
    px4_ros2::Result result,
    bool & action_succeeded
) {

    RCLCPP_DEBUG(
        node_.get_logger(), 
        "GenericModeExecutor::checkNextModeSucceeded()"
    );

    action_succeeded = false;

   if (result == px4_ros2::Result::Rejected) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_ERROR(
                node_.get_logger(), 
                "GenericModeExecutor::checkNextModeSucceeded(): Mode %s rejected, deactivating mode executor %s", 
                (*current_mode_)->mode_name().c_str(), 
                mode_executor_name_.c_str()
            );
            is_active_ = false;
            clearGlobalBlackboard("mode rejected");
            releaseHandoffFailsafeDeferral("mode rejected");
            scheduleMode(
                missionDoneSelectModeId(),
                [](px4_ros2::Result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s rejected, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result == px4_ros2::Result::Interrupted) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s interrupted, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), mode_executor_name_.c_str());
            is_active_ = false;
            clearGlobalBlackboard("mode interrupted");
            releaseHandoffFailsafeDeferral("mode interrupted");
            scheduleMode(
                missionDoneSelectModeId(),
                [](px4_ros2::Result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s interrupted, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result == px4_ros2::Result::Timeout) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s timed out, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), mode_executor_name_.c_str());
            is_active_ = false;
            clearGlobalBlackboard("mode timed out");
            releaseHandoffFailsafeDeferral("mode timed out");
            scheduleMode(
                missionDoneSelectModeId(),
                [](px4_ros2::Result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s timed out, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result == px4_ros2::Result::Deactivated) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s deactivated, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), mode_executor_name_.c_str());
            is_active_ = false;
            clearGlobalBlackboard("mode deactivated");
            releaseHandoffFailsafeDeferral("mode deactivated");
            scheduleMode(
                missionDoneSelectModeId(),
                [](px4_ros2::Result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s deactivated, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result != px4_ros2::Result::Success) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s failed with result %s, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), px4_ros2::resultToString(result), mode_executor_name_.c_str());
            is_active_ = false;
            clearGlobalBlackboard("mode failed");
            releaseHandoffFailsafeDeferral("mode failed");
            scheduleMode(
                missionDoneSelectModeId(),
                [](px4_ros2::Result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s failed with result %s, mode executor action failed", (*current_mode_)->mode_name().c_str(), px4_ros2::resultToString(result));

    } else {

        action_succeeded = true;

    }

    return true;

}

int GenericModeExecutor::missionDoneSelectModeId() {

    std::string mission_done_select_mode = configuration_->GetParameter("/mission/mission_done_select_mode").as_string();
    int mission_done_select_mode_id;

    if (mission_done_select_mode == "hold") {
        
        mission_done_select_mode_id = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;

    } else if (mission_done_select_mode == "land") {

        mission_done_select_mode_id = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;

    } else if (mission_done_select_mode == "position") {

        mission_done_select_mode_id = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;

    } else {

        RCLCPP_FATAL(
            node_.get_logger(), 
            "GenericModeExecutor::missionDoneSelectModeId(): Invalid mission_done_select_mode parameter value %s", 
            mission_done_select_mode.c_str()
        );

        throw std::runtime_error("GenericModeExecutor::missionDoneSelectModeId(): Invalid mission_done_select_mode parameter value");

    }

    return mission_done_select_mode_id;

}

std::string GenericModeExecutor::getModeName(schedule_t schedule) {

    switch(schedule) {
        case schedule_next_mode:
            return (*current_mode_)->mode_name();
        case schedule_land:
            return "Landing";
        case schedule_arm:
            return "Arming";
        case schedule_takeoff:
            return "Takeoff";
        case schedule_disarm:
            return "Disarming";
        case schedule_arm_before_takeoff:
            return "Arm Before Takeoff";
    }

    RCLCPP_FATAL(
        node_.get_logger(), 
        "GenericModeExecutor::getModeName(): Invalid schedule value %d", 
        schedule
    );

    throw std::runtime_error("GenericModeExecutor::getModeName(): Invalid schedule value");

}

bool GenericModeExecutor::scheduleActionIfAny(schedule_t & previous_schedule_current) {

    RCLCPP_DEBUG(
        node_.get_logger(), 
        "GenericModeExecutor::scheduleActionIfAny()"
    );

    previous_schedule_current = schedule_current_;

    switch(schedule_next_) {
        case schedule_next_mode:
            schedule_current_ = schedule_next_mode;

            return false;

        case schedule_land:

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleActionIfAny(): Landing."
            );

            schedule_next_ = schedule_next_mode;
            schedule_current_ = schedule_land;

            deferFailsafesForHandoff(px4_ros2::ModeBase::kModeIDLand, "landing handoff");

            land(
                [this](px4_ros2::Result result) {
                    if (result != px4_ros2::Result::Success) {
                        (*current_mode_)->StartControls();
                    }
                    onModeCompleted(result);
                }
            );

            (*current_mode_)->StopControls();

            return true;

        case schedule_arm:

            RCLCPP_FATAL(
                node_.get_logger(),
                "GenericModeExecutor::scheduleActionIfAny(): Arming should not be handled in onModeComplete but in a custom action callback - control should not reach this point."
            );

            throw std::runtime_error("GenericModeExecutor::scheduleActionIfAny(): Arming should not be handled in onModeComplete but in a custom action callback - control should not reach this point.");

        case schedule_disarm:

            RCLCPP_FATAL(
                node_.get_logger(),
                "GenericModeExecutor::scheduleActionIfAny(): Disarming should not be handled in onModeComplete but in a custom action callback - control should not reach this point."
            );

            throw std::runtime_error("GenericModeExecutor::scheduleActionIfAny(): Disarming should not be handled in onModeComplete but in a custom action callback - control should not reach this point.");

        case schedule_takeoff: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleActionIfAny(): Taking off."
            );

            schedule_next_ = schedule_next_mode;
            schedule_current_ = schedule_takeoff;

            // PX4 takes off to an altitude above mean sea level. Without a
            // global position (e.g. indoors) there is no such reference; NaN
            // makes PX4 use its default takeoff altitude (MIS_TAKEOFF_ALT).
            const double ground_altitude_amsl =
                combined_drone_awareness_adapter_history_[0].ground_altitude_estimate_amsl();
            float takeoff_altitude_amsl = NAN;
            if (std::isnan(ground_altitude_amsl)) {
                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::scheduleActionIfAny(): No global ground altitude estimate; "
                    "PX4 takes off to its default takeoff altitude (MIS_TAKEOFF_ALT) instead of %.2f m.",
                    static_cast<double>(takeoff_altitude_.Load())
                );
            } else {
                takeoff_altitude_amsl = static_cast<float>(takeoff_altitude_.Load() + ground_altitude_amsl);
            }

            (*current_mode_)->StartControls();

            deferFailsafesForHandoff(px4_ros2::ModeBase::kModeIDTakeoff, "takeoff handoff");

            takeoff(
                [this](px4_ros2::Result result) {
                    if (result != px4_ros2::Result::Success) {
                        (*current_mode_)->StopControls();
                    }
                    onModeCompleted(result);
                },
                takeoff_altitude_amsl
            );

            return true;

        }

        case schedule_arm_before_takeoff:

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleActionIfAny(): Arming."
            );

            schedule_next_ = schedule_takeoff;
            schedule_current_ = schedule_arm_before_takeoff;

            arm(
                [this](px4_ros2::Result result) {
                    onModeCompleted(result);
                }
            );

            return true;

        default:
            RCLCPP_FATAL(
                node_.get_logger(), 
                "GenericModeExecutor::scheduleActionIfAny(): Invalid schedule_next_ value %d", 
                static_cast<int>(schedule_next_.Load())
            );

            throw std::runtime_error("GenericModeExecutor::scheduleActionIfAny(): Invalid schedule_next_ value");
        
    }

    return false;

}

void GenericModeExecutor::onNormalModeSuccess(bool & last_mode) {

    std::string next_mode_key = (*current_mode_entry_).next_mode;

    if (next_mode_key.empty()) {

        RCLCPP_INFO(
            node_.get_logger(), 
            "GenericModeExecutor::onNormalModeSuccess(): No next mode specified, deactivating mode executor %s", 
            mode_executor_name_.c_str()
        );

        if ((*current_mode_) != nullptr) {
            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onNormalModeSuccess(): Stopping completed terminal mode %s before mission-done mode handoff.",
                (*current_mode_)->mode_name().c_str()
            );
            (*current_mode_)->StopExecution("MISSION_TERMINAL_HANDOFF");
        }

        const int mission_done_mode_id = missionDoneSelectModeId();
        scheduleMode(
            mission_done_mode_id,
            [this, mission_done_mode_id](px4_ros2::Result result) {
                if (result != px4_ros2::Result::Success) {
                    RCLCPP_ERROR(
                        node_.get_logger(),
                        "GenericModeExecutor::onNormalModeSuccess(): Failed to switch to mission-done mode id %d, result=%s.",
                        mission_done_mode_id,
                        px4_ros2::resultToString(result)
                    );
                    return;
                }
                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::onNormalModeSuccess(): Mission-done mode id %d activated.",
                    mission_done_mode_id
                );
            }
        );

        is_active_ = false;
        clearGlobalBlackboard("terminal mission completion");
        releaseHandoffFailsafeDeferral("terminal mission completion");

        last_mode = true;

        return;

    }

    mode_provider_->BeginModeActivation(next_mode_key);
    current_mode_ = mode_provider_->GetMode(next_mode_key);
    current_mode_entry_ = mission_specification_->GetMissionSpecificationEntry(next_mode_key);

    last_mode = false;

}

void GenericModeExecutor::manualControlSetpointCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {

    StickTakeoverDetector::Sample sticks;
    sticks.valid = msg->valid;
    sticks.roll = msg->roll;
    sticks.pitch = msg->pitch;
    sticks.yaw = msg->yaw;
    sticks.throttle = msg->throttle;

    if (!is_active_) {
        stick_takeover_detector_.Observe(sticks);
        return;
    }

    // A takeover is stick movement since activation, not a stick away from
    // centre: PX4 reports throttle -1 with the stick at the bottom.
    const bool switch_to_position_control = stick_takeover_detector_.ObserveActive(
        sticks,
        configuration_->GetParameter("/mission/manual_stick_input_threshold").as_double()
    );

    if (switch_to_position_control) {

        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::manualControlSetpointCallback(): Position control triggered by stick input, ending mission of executor %s",
            mode_executor_name_.c_str()
        );

        MissionExitDecision decision;
        decision.reason = MissionExitReason::OperatorStickOverride;
        decision.px4_nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;
        triggerMissionExit(decision);

        is_active_ = false;
        triggered_position_control_ = true;
        releaseHandoffFailsafeDeferral("pilot stick takeover");

        scheduleMode(
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL,
            [](px4_ros2::Result) { }
        );

    }

}

void GenericModeExecutor::deferFailsafesForHandoff(uint8_t target_nav_state, const char * handoff) {

    handoff_failsafe_deferral_.Begin(target_nav_state);

    bool failsafe_defer_enabled = false;
    try {
        failsafe_defer_enabled = deferFailsafesSync(true, HandoffFailsafeDeferral::kTimeoutS);
    } catch (const std::exception & exception) {
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::deferFailsafesForHandoff(): Failed while confirming PX4 failsafe deferral "
            "for the %s: %s. Continuing because aborting here makes the external mode unresponsive.",
            handoff,
            exception.what()
        );
    }
    if (failsafe_defer_enabled) {
        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::deferFailsafesForHandoff(): Deferring PX4 failsafes (%d s each) during the %s "
            "until PX4 runs mode %u.",
            HandoffFailsafeDeferral::kTimeoutS,
            handoff,
            static_cast<unsigned>(target_nav_state)
        );
    } else {
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::deferFailsafesForHandoff(): Failed to confirm PX4 failsafe deferral during the %s.",
            handoff
        );
    }

}

void GenericModeExecutor::releaseHandoffFailsafeDeferral(const char * reason) {

    // Only a handoff that never reached its mode still defers failsafes.
    if (!handoff_failsafe_deferral_.Abandon()) {
        return;
    }
    try {
        deferFailsafesSync(false, 0);
    } catch (const std::exception & exception) {
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::releaseHandoffFailsafeDeferral(): Failed while clearing PX4 failsafe deferral (%s): %s",
            reason,
            exception.what()
        );
        return;
    }
    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::releaseHandoffFailsafeDeferral(): PX4 failsafes act again: %s.",
        reason
    );

}

void GenericModeExecutor::onHandoffVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {

    if (!handoff_failsafe_deferral_.pending()) {
        return;
    }
    // A mission mode runs once px4_ros2 has activated it (not while the
    // executor still arms); PX4 runs its own modes as soon as it reports them.
    bool target_running = true;
    for (const auto & mode : *mode_provider_) {
        if (mode->mode_id() == msg->nav_state) {
            target_running = mode->active();
            break;
        }
    }
    if (!handoff_failsafe_deferral_.Reached(msg->nav_state, target_running)) {
        return;
    }
    try {
        deferFailsafesSync(false, 0);
    } catch (const std::exception & exception) {
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::onHandoffVehicleStatus(): Failed while clearing PX4 failsafe deferral: %s",
            exception.what()
        );
        return;
    }
    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::onHandoffVehicleStatus(): PX4 runs mode %u; failsafes act again.",
        static_cast<unsigned>(msg->nav_state)
    );

}

rclcpp_action::GoalResponse GenericModeExecutor::modeExecutorActionGoalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ModeExecutorAction::Goal> goal
) {
    (void)uuid;

    if (schedule_current_ != schedule_next_mode) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::modeExecutorActionGoalCallback(): Cannot execute action while another action is executing. Actions must be requested during mode execution."
        );

        return rclcpp_action::GoalResponse::REJECT;

    }

    switch(goal->request) {

        default: {

            RCLCPP_WARN(
                node_.get_logger(),
                "GenericModeExecutor::modeExecutorActionGoalCallback(): Unknown action, rejecting."
            );

            schedule_next_ = schedule_next_mode;

            return rclcpp_action::GoalResponse::REJECT;

        }

        case iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_TAKEOFF: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::modeExecutorActionGoalCallback(): Received takeoff request."
            );

            if (canTakeoff(goal->takeoff_altitude)) {

                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Takeoff request accepted."
                );

                schedule_next_ = schedule_arm_before_takeoff;
                takeoff_altitude_ = goal->takeoff_altitude;

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

            } else {

                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Takeoff request rejected."
                );

                schedule_next_ = schedule_next_mode;

                return rclcpp_action::GoalResponse::REJECT;

            }

            break;

        }

        case iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_LAND: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::modeExecutorActionGoalCallback(): Received landing request."
            );

            if (canLand()) {

                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Landing request accepted."
                );

                schedule_next_ = schedule_land;

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

            } else {

                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Landing request rejected."
                );

                schedule_next_ = schedule_next_mode;

                return rclcpp_action::GoalResponse::REJECT;

            }

            break;

        }

        case iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_ARM: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::modeExecutorActionGoalCallback(): Received arming request."
            );

            if (canArm()) {

                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Arming request accepted."
                );

                schedule_next_ = schedule_arm;

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

            } else {

                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Arming request rejected."
                );

                schedule_next_ = schedule_next_mode;

                return rclcpp_action::GoalResponse::REJECT;

            }

            break;

        }

        case iii_drone_interfaces::action::ModeExecutorAction::Goal::REQUEST_DISARM: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::modeExecutorActionGoalCallback(): Received disarming request."
            );

            if (canDisarm()) {

                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Disarming request accepted."
                );

                schedule_next_ = schedule_disarm;

                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

            } else {

                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::modeExecutorActionGoalCallback(): Disarming request rejected."
                );

                schedule_next_ = schedule_next_mode;

                return rclcpp_action::GoalResponse::REJECT;

            }

            break;

        }
    }

    RCLCPP_ERROR(
        node_.get_logger(),
        "GenericModeExecutor::modeExecutorActionGoalCallback(): Unknown action, rejecting."
    );

    schedule_next_ = schedule_next_mode;

}

void GenericModeExecutor::modeExecutorActionAcceptedCallback(const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle) {

    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::modeExecutorActionAcceptedCallback(): Action accepted."
    );

    if (schedule_next_ == schedule_arm) {

        handleArmAccepted(goal_handle);

        return;

    }

    if (schedule_next_ == schedule_disarm) {

        handleDisarmAccepted(goal_handle);

        return;

    }

    current_goal_handle_ = goal_handle;

    (*current_mode_)->StayAliveOnNextDeactivate();
    (*current_mode_)->ReportCompletion(px4_ros2::Result::Success);

}

void GenericModeExecutor::handleArmAccepted(const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle) {

    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::handleArmAccepted(): Arming."
    );

    (*current_mode_)->StartControls();

    arm(
        [this,goal_handle](px4_ros2::Result result) {
            if (result != px4_ros2::Result::Success) {
                (*current_mode_)->StopControls();
            }
            onArmCompleted(
                result,
                goal_handle
            );
        }
    );

}

void GenericModeExecutor::handleDisarmAccepted(const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle) {

    bool force_disarm = goal_handle->get_goal()->force_disarm;

    if (force_disarm) {
        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::handleDisarmAccepted(): Force disarming."
        );
    } else {
        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::handleDisarmAccepted(): Disarming."
        );
    }

    (*current_mode_)->StopControls();

    px4_ros2::Result res = sendCommandSync(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        0,
        force_disarm ? 21196 : 0
    );

    if (res != px4_ros2::Result::Success) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::handleDisarmAccepted(): Disarming failed."
        );

        (*current_mode_)->StartControls();

        schedule_next_ = schedule_next_mode;

        goal_handle->abort(std::make_shared<ModeExecutorAction::Result>());

        return;

    }

    waitUntilDisarmed(
        [this, goal_handle](px4_ros2::Result result) {
            if (result != px4_ros2::Result::Success) {
                (*current_mode_)->StartControls();
            }
            onDisarmCompleted(
                result,
                goal_handle
            );
        }
    );

}

void GenericModeExecutor::onArmCompleted(
    px4_ros2::Result result,
    const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle
) {

    schedule_next_ = schedule_next_mode;

    if (result != px4_ros2::Result::Success) {

        if (MissionControl::Process().ExitLatched()) {
            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onArmCompleted(): Arming ended by Mission Exit."
            );
        } else {
            RCLCPP_WARN(
                node_.get_logger(),
                "GenericModeExecutor::onArmCompleted(): Arming failed."
            );
        }

        try {
            goal_handle->abort(std::make_shared<ModeExecutorAction::Result>());
        } catch (const std::exception & error) {
            RCLCPP_DEBUG(node_.get_logger(), "GenericModeExecutor::onArmCompleted(): goal already terminal: %s", error.what());
        }

        return;

    }

    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::onArmCompleted(): Arming succeeded."
    );

    goal_handle->succeed(std::make_shared<ModeExecutorAction::Result>());

}

void GenericModeExecutor::onDisarmCompleted(
    px4_ros2::Result result,
    const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle
) {

    schedule_next_ = schedule_next_mode;

    if (result != px4_ros2::Result::Success) {

        if (MissionControl::Process().ExitLatched()) {
            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onDisarmCompleted(): Disarming ended by Mission Exit."
            );
        } else {
            RCLCPP_WARN(
                node_.get_logger(),
                "GenericModeExecutor::onDisarmCompleted(): Disarming failed."
            );
        }

        try {
            goal_handle->abort(std::make_shared<ModeExecutorAction::Result>());
        } catch (const std::exception & error) {
            RCLCPP_DEBUG(node_.get_logger(), "GenericModeExecutor::onDisarmCompleted(): goal already terminal: %s", error.what());
        }

        return;

    }

    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::onDisarmCompleted(): Disarming succeeded."
    );

    goal_handle->succeed(std::make_shared<ModeExecutorAction::Result>());

}

void GenericModeExecutor::stopModeIfWaiting() {

    tryCompleteActionGoal(false);

    if ((*current_mode_) != nullptr) {

        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::stopModeIfWaiting(): Stopping mode %s.",
            (*current_mode_)->mode_name().c_str()
        );

        (*current_mode_)->StopExecution("MISSION_MODE_FAILURE_HANDOFF");

    }

}

void GenericModeExecutor::tryCompleteActionGoal(bool success) {

    RCLCPP_DEBUG(
        node_.get_logger(),
        "GenericModeExecutor::tryCompleteActionGoal()"
    );

    if ((*current_goal_handle_) == nullptr) {

        RCLCPP_DEBUG(
            node_.get_logger(),
            "GenericModeExecutor::tryCompleteActionGoal(): No goal handle found."
        );

        return;

    }

    if (success) {

        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::tryCompleteActionGoal(): Mode executor action succeeded."
        );

        (*current_goal_handle_)->succeed(std::make_shared<ModeExecutorAction::Result>());

    } else {

        if (MissionControl::Process().ExitLatched()) {
            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::tryCompleteActionGoal(): Mode executor action ended by Mission Exit."
            );
        } else {
            RCLCPP_WARN(
                node_.get_logger(),
                "GenericModeExecutor::tryCompleteActionGoal(): Mode executor action failed."
            );
        }

        (*current_goal_handle_)->abort(std::make_shared<ModeExecutorAction::Result>());

    }

    current_goal_handle_ = (std::shared_ptr<GoalHandleModeExecutorAction>)nullptr;

}

bool GenericModeExecutor::canLand() {

    if (!is_active_) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canLand(): Landing after current mode rejected: Mode executor is not active."
        );

        return false;

    }

    if (combined_drone_awareness_adapter_history_.empty()){

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canLand(): Landing after current mode rejected: Drone location history is empty."
        );

        return false;

    }

    if (combined_drone_awareness_adapter_history_[0].drone_location() == DRONE_LOCATION_UNKNOWN) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canLand(): Landing after current mode rejected: Drone location is unknown."
        );

        return false;

    }

    if (combined_drone_awareness_adapter_history_[0].on_ground()) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canLand(): Landing after current mode rejected: Drone is on ground."
        );

        return false;

    }

    return true;

}

bool GenericModeExecutor::canArm() {

    if (!is_active_) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canArm(): Arming after current mode rejected: Mode executor is not active."
        );

        return false;

    }

    // if (combined_drone_awareness_adapter_history_.empty()){

    //     RCLCPP_WARN(
    //         node_.get_logger(),
    //         "GenericModeExecutor::canArm(): Arm after current mode rejected: Armed history is empty."
    //     );

    //     return false;

    // }

    // if (combined_drone_awareness_adapter_history_[0].armed()) {
    if (isArmed()) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canArm(): Armed after current mode rejected: Drone is already armed."
        );

        return false;

    }

    return true;

}

bool GenericModeExecutor::canDisarm() {

    if (!is_active_) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canDisarm(): Disarming after current mode rejected: Mode executor is not active."
        );

        return false;

    }

    if (combined_drone_awareness_adapter_history_.empty()){

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canDisarm(): Disarm after current mode rejected: Combined drone awareness history is empty."
        );

        return false;

    }

    auto drone_location = combined_drone_awareness_adapter_history_[0].drone_location();

    if (drone_location == DRONE_LOCATION_UNKNOWN) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canDisarm(): Disarming after current mode rejected: Drone location is unknown."
        );

        return false;

    }

    if (drone_location == DRONE_LOCATION_IN_FLIGHT) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canDisarm(): Disarming after current mode rejected: Drone is in flight."
        );

        return false;

    }

    // if (combined_drone_awareness_adapter_history_[0].armed()) {
    if (!isArmed()) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canDisarm(): Disarming after current mode rejected: Drone is not armed."
        );

        return false;

    }

    return true;

}

bool GenericModeExecutor::canTakeoff(float altitude) {

    if (!is_active_) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canTakeoff(): Taking off after current mode rejected: Mode executor is not active."
        );

        return false;

    }

    if (combined_drone_awareness_adapter_history_.empty()){

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canTakeoff(): Taking off after current mode rejected: Armed history is empty."
        );

        return false;

    }

    // if (combined_drone_awareness_adapter_history_[0].armed()) {
    // if (isArmed()) {

    //     RCLCPP_WARN(
    //         node_.get_logger(),
    //         "GenericModeExecutor::canTakeoff(): Taking off after current mode rejected: Drone is already armed."
    //     );

    //     return false;

    // }

    const double gae_amsl = combined_drone_awareness_adapter_history_[0].ground_altitude_estimate_amsl();

    if (std::isnan(gae_amsl)) {

        // No global position: the takeoff cannot be given above mean sea
        // level, so PX4 uses its default takeoff altitude.
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canTakeoff(): No global ground altitude estimate; PX4 will take off to "
            "its default takeoff altitude (MIS_TAKEOFF_ALT) instead of %.2f m.",
            static_cast<double>(altitude)
        );

    }

    if (altitude <= 0) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canTakeoff(): Taking off after current mode rejected: Takeoff altitude %f is not positive.",
            altitude
        );

        return false;

    }

    return true;

}

void GenericModeExecutor::onMissionExitVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {

    iii_drone::mission::VehicleControlSample sample;
    sample.timestamp_us = msg->timestamp;
    sample.nav_state = msg->nav_state;
    sample.executor_in_charge = msg->executor_in_charge;
    sample.failsafe = msg->failsafe;
    sample.receipt = std::chrono::steady_clock::now();

    const int executor_id = id();
    const uint8_t observed_executor_id =
        executor_id > 0 && executor_id <= 255 ? static_cast<uint8_t>(executor_id) : 0;
    const auto decision = MissionControl::Process().ObserveVehicleStatus(
        sample, observed_executor_id);
    if (decision && is_active_) {
        triggerMissionExit(*decision);
    }

}

void GenericModeExecutor::triggerMissionExit(const MissionExitDecision & decision) {

    std::lock_guard<std::mutex> lock(mission_exit_mutex_);

    iii_drone::mission::MissionExitSteps steps;
    steps.stop_setpoint_consumption = [this]() {
        for (auto mode : *mode_provider_) {
            mode->PrepareForMissionExit();
        }
    };
    steps.stop_trees = [this]() {
        for (auto mode : *mode_provider_) {
            if (mode->tree_running() || mode->active()) {
                mode->StopExecution("MISSION_EXIT");
            }
        }
    };
    steps.complete_executor_action = [this]() {
        completeActionGoalForMissionExit();
    };
    steps.release_consumer_control = [this, decision]() {
        using Release = iii_drone_interfaces::srv::ReleaseConsumerControl;
        const auto client = mode_provider_->maneuver_reference_client();
        if (!client) {
            return;
        }
        uint8_t reason = Release::Request::REASON_OTHER;
        switch (decision.reason) {
            case MissionExitReason::OperatorModeChange:
                reason = Release::Request::REASON_OPERATOR_MODE_CHANGE;
                break;
            case MissionExitReason::OperatorStickOverride:
                reason = Release::Request::REASON_OPERATOR_STICK_OVERRIDE;
                break;
            case MissionExitReason::Failsafe:
                reason = Release::Request::REASON_FAILSAFE;
                break;
            case MissionExitReason::None:
            default:
                break;
        }
        client->ReleaseConsumerControl(reason, decision.px4_nav_state);
    };
    steps.stop_side_effects = [this]() {
        if (const auto command = MissionControl::Process().TakePlMapperExitCommand()) {
            sendPlMapperExitCommand(*command);
        }
    };
    steps.end_run_bookkeeping = [this, decision]() {
        is_active_ = false;
        schedule_next_ = schedule_next_mode;
        schedule_current_ = schedule_next_mode;
        clearGlobalBlackboard(
            std::string("Mission Exit: ") + iii_drone::mission::missionExitReasonLabel(decision.reason)
        );
    };

    const auto record = iii_drone::mission::ExecuteMissionExit(
        MissionControl::Process(),
        decision,
        steps,
        [this](const char * step, const std::exception & error) {
            RCLCPP_ERROR(
                node_.get_logger(),
                "GenericModeExecutor::triggerMissionExit(): Mission Exit step %s failed: %s",
                step,
                error.what()
            );
        }
    );
    if (!record) {
        return;
    }

    const auto current_mode = current_mode_.Load();
    const std::string mode_name = current_mode != nullptr ? current_mode->mode_name() : "";
    if (iii_drone::mission::isOperatorMissionExit(record->reason)) {
        RCLCPP_INFO(
            node_.get_logger(),
            "GenericModeExecutor::triggerMissionExit(): Mission Exit (%s, PX4 nav_state %u) during %s: "
            "dispatch closed, trees stopped, consumer released",
            iii_drone::mission::missionExitReasonLabel(record->reason),
            static_cast<unsigned>(record->px4_nav_state),
            mode_name.c_str()
        );
    } else {
        RCLCPP_ERROR(
            node_.get_logger(),
            "GenericModeExecutor::triggerMissionExit(): Mission Exit (%s, PX4 nav_state %u) during %s: "
            "dispatch closed, trees stopped, consumer released",
            iii_drone::mission::missionExitReasonLabel(record->reason),
            static_cast<unsigned>(record->px4_nav_state),
            mode_name.c_str()
        );
    }
    auto event = iii_drone::diagnostics::HilTrace::event("mission_exit");
    event.text("reason", iii_drone::mission::missionExitReasonLabel(record->reason));
    event.number("px4_nav_state", record->px4_nav_state);
    event.number("px4_timestamp_us", record->px4_timestamp_us);
    event.number("run", record->run);
    event.text("mode", mode_name);
    event.commit();

}

void GenericModeExecutor::completeActionGoalForMissionExit() {

    const auto goal_handle = current_goal_handle_.Load();
    current_goal_handle_ = (std::shared_ptr<GoalHandleModeExecutorAction>)nullptr;
    if (goal_handle == nullptr) {
        return;
    }
    try {
        if (goal_handle->is_active()) {
            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::completeActionGoalForMissionExit(): Ending mode executor action for Mission Exit."
            );
            goal_handle->abort(std::make_shared<ModeExecutorAction::Result>());
        }
    } catch (const std::exception & error) {
        RCLCPP_DEBUG(
            node_.get_logger(),
            "GenericModeExecutor::completeActionGoalForMissionExit(): goal already terminal: %s",
            error.what()
        );
    }

}

void GenericModeExecutor::sendPlMapperExitCommand(uint8_t command) {

    if (!pl_mapper_command_client_ || !pl_mapper_command_client_->service_is_ready()) {
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::sendPlMapperExitCommand(): PL mapper command service unavailable; mapper left as the mission left it"
        );
        return;
    }
    auto request = std::make_shared<iii_drone_interfaces::srv::PLMapperCommand::Request>();
    request->pl_mapper_cmd.command = command;
    request->pl_mapper_cmd.reset = true;
    auto logger = node_.get_logger();
    pl_mapper_command_client_->async_send_request(
        request,
        [logger](rclcpp::Client<iii_drone_interfaces::srv::PLMapperCommand>::SharedFuture future) {
            if (future.get()->pl_mapper_ack !=
                    iii_drone_interfaces::srv::PLMapperCommand::Response::PL_MAPPER_ACK_SUCCESS) {
                RCLCPP_WARN(logger, "GenericModeExecutor: PL mapper refused the Mission Exit stop command");
            }
        }
    );
    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::sendPlMapperExitCommand(): Stopping the PL mapper started by the exited mission"
    );

}
