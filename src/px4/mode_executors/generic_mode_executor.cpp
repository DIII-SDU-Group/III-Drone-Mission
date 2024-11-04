/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/mode_executors/generic_mode_executor.hpp>

using namespace iii_drone::px4;
using namespace iii_drone::mission;
using namespace iii_drone::configuration;

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
    drone_location_history_(1),
    armed_history_(1) {

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

    schedule_request_srv_ = node_.create_service<iii_drone_interfaces::srv::ModeExecutorScheduleRequest>(
        "/mission/mode_executor/schedule_request",
        std::bind(
            &GenericModeExecutor::scheduleRequestCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    combined_drone_awareness_sub_ = node_.create_subscription<iii_drone_interfaces::msg::CombinedDroneAwareness>(
        "/control/maneuver_controller/combined_drone_awareness",
        10,
        [this](const iii_drone_interfaces::msg::CombinedDroneAwareness::SharedPtr msg) {
            drone_location_history_.Store(msg->drone_location);
            armed_history_.Store(msg->armed);
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

    scheduleMode(
        current_mode_->id(),
        [this](px4_ros2::Result result) {
            onModeCompleted(result);
        }
    );

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

    std::string mode_name;

    switch (schedule_current_) {
        case schedule_next_mode:
            mode_name = current_mode_->mode_name();
            break;
        case schedule_takeoff:
            mode_name = "Takeoff";
            break;
        case schedule_land:
            mode_name = "Landing";
            break;
        case schedule_arm:
            mode_name = "Arming";
            break;
        case schedule_disarm:
            mode_name = "Disarming";
            break;
        case schedule_arm_before_takeoff:
            mode_name = "Arm Before Takeoff";
            break;
    };

    RCLCPP_INFO(
        node_.get_logger(), 
        "GenericModeExecutor::onModeCompleted(): Mode %s completed with result %s", 
        mode_name.c_str(),
        px4_ros2::resultToString(result)
    );

    std::string mission_done_select_mode = parameters_->GetParameter("mission_done_select_mode").as_string();
    int mission_done_select_mode_id;

    if (mission_done_select_mode == "hold") {
        
        mission_done_select_mode_id = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER;

    } else if (mission_done_select_mode == "land") {

        mission_done_select_mode_id = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND;

    } else if (mission_done_select_mode == "position") {

        mission_done_select_mode_id = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL;

    } else {

        RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::onModeCompleted(): Invalid mission_done_select_mode parameter value %s, deactivating mode executor %s", mission_done_select_mode.c_str(), mode_executor_name_.c_str());
        is_active_ = false;
        return;

    }

    if (triggered_position_control_) {
        triggered_position_control_ = false;

        RCLCPP_WARN(
            node_.get_logger(), 
            "GenericModeExecutor::onModeCompleted(): Position control triggered during mode %s, deactivating mode executor %s", 
            current_mode_->mode_name().c_str(),
            mode_executor_name_.c_str()
        );

        is_active_ = false;

        return;
    }

    if (result != px4_ros2::Result::Success) {
        RCLCPP_ERROR(node_.get_logger(), "GenericModeExecutor::onModeCompleted(): Mode %s failed, deactivating mode executor %s", current_mode_->mode_name().c_str(), mode_executor_name_.c_str());
        is_active_ = false;
        scheduleMode(
            mission_done_select_mode_id,
            [this](px4_ros2::Result result) { }
        );
        return;
    }

    if (schedule_next_ == schedule_next_mode) {
    // if (schedule_current_ != schedule_next_mode) {

        // if (schedule_current_ == schedule_land) {
        //     RCLCPP_INFO(
        //         node_.get_logger(),
        //         "GenericModeExecutor::onModeCompleted(): Waiting for disarming."
        //     );

        //     schedule_current_ = schedule_disarm;

        //     waitUntilDisarmed(
        //         [this](px4_ros2::Result result) {
        //             onModeCompleted(result);
        //         }
        //     );

        //     return;
        // }

        schedule_current_ = schedule_next_mode;

    } else {

        if (schedule_next_ == schedule_land) {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onModeCompleted(): Landing."
            );

            schedule_next_ = schedule_next_mode;
            schedule_current_ = schedule_land;

            land(
                [this](px4_ros2::Result result) {
                    onModeCompleted(result);
                }
            );

            return;

        } else if (schedule_next_ == schedule_arm) {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onModeCompleted(): Arming."
            );

            schedule_next_ = schedule_next_mode;
            schedule_current_ = schedule_arm;

            arm(
                [this](px4_ros2::Result result) {
                    onModeCompleted(result);
                }
            );

            return;

        } else if (schedule_next_ == schedule_takeoff) {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onModeCompleted(): Taking off."
            );

            schedule_next_ = schedule_next_mode;
            schedule_current_ = schedule_takeoff;

            takeoff(
                [this](px4_ros2::Result result) {
                    onModeCompleted(result);
                },
                takeoff_altitude_
            );

            return;

        } else if (schedule_next_ == schedule_arm_before_takeoff) {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::onModeCompleted(): Arming."
            );

            schedule_next_ = schedule_takeoff;
            schedule_current_ = schedule_arm_before_takeoff;

            arm(
                [this](px4_ros2::Result result) {
                    onModeCompleted(result);
                }
            );

            return;

        }

    }

    std::string next_mode_key = current_mode_entry_.next_mode;

    if (next_mode_key.empty()) {
        RCLCPP_INFO(node_.get_logger(), "GenericModeExecutor::onModeCompleted(): No next mode specified, deactivating mode executor %s", mode_executor_name_.c_str());
        scheduleMode(
            mission_done_select_mode_id,
            [this](px4_ros2::Result result) {
            }
        );
        is_active_ = false;
        return;
    }

    current_mode_ = mode_provider_->GetMode(next_mode_key);
    current_mode_entry_ = mission_specification_->GetMissionSpecificationEntry(next_mode_key);

    RCLCPP_INFO(
        node_.get_logger(),
        "GenericModeExecutor::onModeCompleted(): Activating mode %s.",
        current_mode_entry_.mode_name.c_str()
    );
    

    scheduleMode(
        current_mode_->id(),
        [this](px4_ros2::Result result) {
            onModeCompleted(result);
        }
    );

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

void GenericModeExecutor::scheduleRequestCallback(
    const std::shared_ptr<iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Response> response
) {

    if (schedule_current_ != schedule_next_mode) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::scheduleRequestCallback(): Cannot schedule action while another action is executing. Scheduling requests must be activated during mode execution."
        );

        response->success = false;

        return;

    }

    switch(request->request) {

        default:
        case iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_CLEAR: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleRequestCallback(): Received clear request, setting schedule next to next mode."
            );

            schedule_next_ = schedule_next_mode;

            response->success = true;

            break;

        }

        case iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_TAKEOFF: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleRequestCallback(): Received request to schedule takeoff."
            );

            if (canScheduleTakeoff(request->takeoff_altitude)) {

                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::scheduleRequestCallback(): Request to schedule takeoff accepted."
                );

                // schedule_next_ = schedule_takeoff;
                schedule_next_ = schedule_arm_before_takeoff;
                takeoff_altitude_ = request->takeoff_altitude;

                response->success = true;

            } else {

                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::scheduleRequestCallback(): Request to schedule takeoff rejected."
                );

                response->success = false;

            }

            break;

        }

        case iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_LAND: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleRequestCallback(): Received request to schedule landing."
            );

            if (canScheduleLand()) {

                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::scheduleRequestCallback(): Request to schedule landing accepted."
                );

                schedule_next_ = schedule_land;

                response->success = true;

            } else {

                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::scheduleRequestCallback(): Request to schedule landing rejected."
                );

                response->success = false;

            }

            break;

        }

        case iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_ARM: {

            RCLCPP_INFO(
                node_.get_logger(),
                "GenericModeExecutor::scheduleRequestCallback(): Received request to schedule arming."
            );

            if (canScheduleArm()) {

                RCLCPP_INFO(
                    node_.get_logger(),
                    "GenericModeExecutor::scheduleRequestCallback(): Request to schedule arming accepted."
                );

                schedule_next_ = schedule_arm;

                response->success = true;

            } else {

                RCLCPP_WARN(
                    node_.get_logger(),
                    "GenericModeExecutor::scheduleRequestCallback(): Request to schedule arming rejected."
                );

                response->success = false;

            }

            break;

        }
    }
}

bool GenericModeExecutor::canScheduleLand() {

    if (!is_active_) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleLand(): Landing after current mode rejected: Mode executor is not active."
        );

        return false;

    }

    if (drone_location_history_.empty()){

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleLand(): Landing after current mode rejected: Drone location history is empty."
        );

        return false;

    }

    if (drone_location_history_[0] == iii_drone_interfaces::msg::CombinedDroneAwareness::DRONE_LOCATION_UNKNOWN) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleLand(): Landing after current mode rejected: Drone location is unknown."
        );

        return false;

    }

    if (drone_location_history_[0] == iii_drone_interfaces::msg::CombinedDroneAwareness::DRONE_LOCATION_ON_GROUND) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleLand(): Landing after current mode rejected: Drone is on ground."
        );

        return false;

    }

    return true;

}

bool GenericModeExecutor::canScheduleArm() {

    if (!is_active_) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleArm(): Arming after current mode rejected: Mode executor is not active."
        );

        return false;

    }

    if (armed_history_.empty()){

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleArm(): Arm after current mode rejected: Armed history is empty."
        );

        return false;

    }

    if (armed_history_[0]) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleArm(): Armed after current mode rejected: Drone is already armed."
        );

        return false;

    }

    return true;

}

bool GenericModeExecutor::canScheduleTakeoff(float altitude) {

    if (!is_active_) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleTakeoff(): Taking off after current mode rejected: Mode executor is not active."
        );

        return false;

    }

    if (armed_history_.empty()){

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleTakeoff(): Taking off after current mode rejected: Armed history is empty."
        );

        return false;

    }

    if (armed_history_[0]) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleTakeoff(): Taking off after current mode rejected: Drone is already armed."
        );

        return false;

    }

    if (altitude <= 0) {

        RCLCPP_WARN(
            node_.get_logger(),
            "GenericModeExecutor::canScheduleTakeoff(): Taking off after current mode rejected: Takeoff altitude %f is not positive.",
            altitude
        );

        return false;

    }

    return true;

}