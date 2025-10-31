/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/mode_executors/generic_mode_executor.hpp>

using namespace iii_drone::px4;
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
    ParameterBundle::SharedPtr parameters
) : ModeExecutorBase(
    *mode_provider->mode_node(), 
    px4_ros2::ModeExecutorBase::Settings{.activation=px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways}, 
    owned_mode
),  node_(*mode_provider->mode_node()),
    mode_executor_name_(mode_executor_name),
    mission_specification_(mission_specification),
    parameters_(parameters),
    mode_provider_(mode_provider),
    combined_drone_awareness_adapter_history_(1) {

    RCLCPP_DEBUG(node_.get_logger(), "GenericModeExecutor::GenericModeExecutor(): Initializing mode executor %s", mode_executor_name_.c_str());

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

    combined_drone_awareness_sub_ = node_.create_subscription<iii_drone_interfaces::msg::CombinedDroneAwareness>(
        "/control/maneuver_controller/combined_drone_awareness",
        10,
        [this](const iii_drone_interfaces::msg::CombinedDroneAwareness::SharedPtr msg) {
            CombinedDroneAwarenessAdapter adapter(*msg);
            combined_drone_awareness_adapter_history_.Store(adapter);
        }

    );

}
    
void GenericModeExecutor::onActivate() {

    RCLCPP_INFO(node_.get_logger(), "GenericModeExecutor::onActivate(): Activating mode executor %s", mode_executor_name_.c_str());

    is_active_ = true;
    triggered_position_control_ = false;

    std::string owned_mode_key = mission_specification_->executor_owned_mode();
    current_mode_ = mode_provider_->GetMode(owned_mode_key);
    current_mode_entry_ = mission_specification_->GetMissionSpecificationEntry(owned_mode_key);

    schedule_next_ = schedule_next_mode;
    schedule_current_ = schedule_next_mode;

    if (isArmed()) {

        scheduleMode(
            (*current_mode_)->id(),
            [this](px4_ros2::Result result) {
                onModeCompleted(result);
            }
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

    is_active_ = false;

    switch(reason) {
        case DeactivateReason::FailsafeActivated:
            RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::onDeactivate(): Deactivating mode executor %s because failsafe activated.", mode_executor_name_.c_str());
            break;
        case DeactivateReason::Other:
            RCLCPP_INFO(node_.get_logger(), "GenericModeExecutor::onDeactivate(): Deactivating mode executor %s", mode_executor_name_.c_str());
            break;
    }

}

void GenericModeExecutor::onModeCompleted(px4_ros2::Result result) {

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

    auto activate_next_mode = [this]() {

 

    };

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

        scheduleMode(
            (*current_mode_)->id(),
            [this](px4_ros2::Result result) {
                onModeCompleted(result);
            }
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
            scheduleMode(
                missionDoneSelectModeId(),
                [this](px4_ros2::Result result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s rejected, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result == px4_ros2::Result::Interrupted) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s interrupted, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), mode_executor_name_.c_str());
            is_active_ = false;
            scheduleMode(
                missionDoneSelectModeId(),
                [this](px4_ros2::Result result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s interrupted, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result == px4_ros2::Result::Timeout) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s timed out, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), mode_executor_name_.c_str());
            is_active_ = false;
            scheduleMode(
                missionDoneSelectModeId(),
                [this](px4_ros2::Result result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s timed out, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result == px4_ros2::Result::Deactivated) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s deactivated, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), mode_executor_name_.c_str());
            is_active_ = false;
            scheduleMode(
                missionDoneSelectModeId(),
                [this](px4_ros2::Result result) { }
            );

            stopModeIfWaiting();

            return false;

        }

        RCLCPP_WARN(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s deactivated, mode executor action failed", (*current_mode_)->mode_name().c_str());

    } else if (result != px4_ros2::Result::Success) {

        if (schedule_current_ == schedule_next_mode) {

            RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::checkNextModeSucceeded(): Mode %s failed with result %s, deactivating mode executor %s", (*current_mode_)->mode_name().c_str(), px4_ros2::resultToString(result), mode_executor_name_.c_str());
            is_active_ = false;
            scheduleMode(
                missionDoneSelectModeId(),
                [this](px4_ros2::Result result) { }
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

    std::string mission_done_select_mode = parameters_->GetParameter("mission_done_select_mode").as_string();
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

        case schedule_takeoff:

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleActionIfAny(): Taking off."
            );

            schedule_next_ = schedule_next_mode;
            schedule_current_ = schedule_takeoff;

            (*current_mode_)->StartControls();

            takeoff(
                [this](px4_ros2::Result result) {
                    if (result != px4_ros2::Result::Success) {
                        (*current_mode_)->StopControls();
                    }
                    onModeCompleted(result);
                },
                takeoff_altitude_ + combined_drone_awareness_adapter_history_[0].ground_altitude_estimate_amsl()
            );

            return true;

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
                schedule_next_
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

        scheduleMode(
            missionDoneSelectModeId(),
            [this](px4_ros2::Result result) { }
        );

        is_active_ = false;

        last_mode = true;

        return;

    }

    current_mode_ = mode_provider_->GetMode(next_mode_key);
    current_mode_entry_ = mission_specification_->GetMissionSpecificationEntry(next_mode_key);

    last_mode = false;

}

void GenericModeExecutor::manualControlSetpointCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {

    if (!is_active_) {
        return;
    }

    bool switch_to_position_control = false;

    double manual_stick_input_threshold = parameters_->GetParameter("manual_stick_input_threshold").as_double();

    if (abs(msg->throttle) > manual_stick_input_threshold) {

        switch_to_position_control = true;

    } else if (abs(msg->yaw) > manual_stick_input_threshold) {

        switch_to_position_control = true;

    } else if (abs(msg->roll) > manual_stick_input_threshold) {

        switch_to_position_control = true;

    } else if (abs(msg->pitch) > manual_stick_input_threshold) {

        switch_to_position_control = true;

    }

    if (switch_to_position_control) {

        RCLCPP_WARN(
            node_.get_logger(), 
            "GenericModeExecutor::manualControlSetpointCallback(): Position control triggered, deactivating mode executor %s", 
            mode_executor_name_.c_str()
        );

        is_active_ = false;
        triggered_position_control_ = true;

        scheduleMode(
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL,
            [this](px4_ros2::Result result) { }
        );

    }

}

rclcpp_action::GoalResponse GenericModeExecutor::modeExecutorActionGoalCallback(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ModeExecutorAction::Goal> goal
) {

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
    (*current_mode_)->completed(px4_ros2::Result::Success);

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
        RCLCPP_WARN(
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

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::onArmCompleted(): Arming failed."
        );

        goal_handle->abort(std::make_shared<ModeExecutorAction::Result>());

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

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::onDisarmCompleted(): Disarming failed."
        );

        goal_handle->abort(std::make_shared<ModeExecutorAction::Result>());

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

        (*current_mode_)->StopExecution();

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

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::tryCompleteActionGoal(): Mode executor action failed."
        );

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

    float gae_amsl = combined_drone_awareness_adapter_history_[0].ground_altitude_estimate_amsl();

    if (gae_amsl == NAN) {
    
        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canTakeoff(): Taking off after current mode rejected: Does not have global ground altitude estimate."
        );

        return false;
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