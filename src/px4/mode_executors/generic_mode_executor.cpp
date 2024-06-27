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
    px4_ros2::ModeExecutorBase::Settings(), 
    owned_mode
),  node_(*mode_provider->mode_node()),
    mode_executor_name_(mode_executor_name),
    mission_specification_(mission_specification),
    parameters_(parameters),
    mode_provider_(mode_provider) {

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

}
    
void GenericModeExecutor::onActivate() {

    RCLCPP_INFO(node_.get_logger(), "GenericModeExecutor::onActivate(): Activating mode executor %s", mode_executor_name_.c_str());

    is_active_ = true;
    triggered_position_control_ = false;

    std::string owned_mode_key = mission_specification_->executor_owned_mode();
    current_mode_ = mode_provider_->GetMode(owned_mode_key);
    current_mode_entry_ = mission_specification_->GetMissionSpecificationEntry(owned_mode_key);

    wait_for_land_ = false;
    wait_for_arm_ = false;

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

    RCLCPP_INFO(
        node_.get_logger(), 
        "GenericModeExecutor::onModeCompleted(): Mode %s completed with result %s", 
        current_mode_->mode_name().c_str(),
        px4_ros2::resultToString(result)
    );

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
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
            [this](px4_ros2::Result result) {
            }
        );
        return;
    }

    if (wait_for_land_) {

        wait_for_land_ = false;

    } else if (wait_for_arm_) {

        wait_for_arm_ = false;

    } else {

        if (current_mode_entry_.land_when_finished) {

            wait_for_land_ = true;

            land(
                [this](px4_ros2::Result result) {
                    onModeCompleted(result);
                }
            );

            return;

        } else if (current_mode_entry_.arm_when_finished) {

            wait_for_arm_ = true;

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
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
            [this](px4_ros2::Result result) {
            }
        );
        is_active_ = false;
        return;
    }

    current_mode_ = mode_provider_->GetMode(next_mode_key);
    current_mode_entry_ = mission_specification_->GetMissionSpecificationEntry(next_mode_key);

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