#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/adapters/combined_drone_awareness_adapter.hpp>

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/history.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp>

#include <iii_drone_mission/mission/mission_specification.hpp>

#include <iii_drone_mission/px4/modes/mode_provider.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/mode_executor_action.hpp>

#include <iii_drone_interfaces/msg/combined_drone_awareness.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

/*****************************************************************************/
// PX4-ROS2:

#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/mode.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace px4 {

    class GenericModeExecutor : public px4_ros2::ModeExecutorBase {

        using ModeExecutorAction = iii_drone_interfaces::action::ModeExecutorAction;
        using GoalHandleModeExecutorAction = rclcpp_action::ServerGoalHandle<ModeExecutorAction>;

    public:
        GenericModeExecutor(
            px4_ros2::ModeBase & owned_mode,
            std::string mode_executor_name,
            iii_drone::mission::MissionSpecification::SharedPtr mission_specification,
            ModeProvider::SharedPtr mode_provider,
            iii_drone::configuration::ParameterBundle::SharedPtr parameters
        ); 
        
        void onActivate() override;

        void onDeactivate(DeactivateReason reason) override;

        typedef std::shared_ptr<GenericModeExecutor> SharedPtr;

        typedef std::unique_ptr<GenericModeExecutor> UniquePtr;

    private:
        rclcpp::Node & node_;

        std::string mode_executor_name_;

        iii_drone::mission::MissionSpecification::SharedPtr mission_specification_;

        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        ModeProvider::SharedPtr mode_provider_;

        utils::Atomic<bool> is_active_ = false;
        utils::Atomic<bool> triggered_position_control_ = false;

        utils::Atomic<ManeuverMode::SharedPtr> current_mode_;
        utils::Atomic<iii_drone::mission::mission_specification_entry_t> current_mode_entry_;

        void onModeCompleted(px4_ros2::Result result);

        bool checkScheduleAndActionValidity();

        void logModeCompleted(
            std::string mode_name, 
            px4_ros2::Result result
        );

        bool checkPositionControlTriggered();

        bool checkNextModeSucceeded(
            px4_ros2::Result result,
            bool & action_succeeded
        );

        int missionDoneSelectModeId();

        enum schedule_t {
            schedule_next_mode,
            schedule_land,
            schedule_arm,
            schedule_takeoff,
            schedule_disarm,
            schedule_arm_before_takeoff
        };

        std::string getModeName(schedule_t schedule);

        bool scheduleActionIfAny(schedule_t & previous_schedule_current);

        void onNormalModeSuccess(bool & last_mode);

        utils::Atomic<schedule_t> schedule_next_ = schedule_next_mode;
        utils::Atomic<schedule_t> schedule_current_ = schedule_next_mode;
        iii_drone::utils::Atomic<float> takeoff_altitude_;

        rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_sub_;
        void manualControlSetpointCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);

        // rclcpp::Service<iii_drone_interfaces::srv::ModeExecutorScheduleRequest>::SharedPtr schedule_request_srv_;
        // void scheduleRequestCallback(
        //     const std::shared_ptr<iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request> request,
        //     std::shared_ptr<iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Response> response
        // );

        rclcpp_action::Server<ModeExecutorAction>::SharedPtr mode_executor_action_server_;
        rclcpp_action::GoalResponse modeExecutorActionGoalCallback(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const ModeExecutorAction::Goal> goal
        );
        void modeExecutorActionAcceptedCallback(const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle);

        void handleArmAccepted(const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle);

        void handleDisarmAccepted(const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle);

        void onArmCompleted(
            px4_ros2::Result result,
            const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle
        );

        void onDisarmCompleted(
            px4_ros2::Result result,
            const std::shared_ptr<GoalHandleModeExecutorAction> goal_handle
        );

        utils::Atomic<std::shared_ptr<GoalHandleModeExecutorAction>> current_goal_handle_ = (std::shared_ptr<GoalHandleModeExecutorAction>)nullptr;

        void stopModeIfWaiting();

        void tryCompleteActionGoal(bool success);

        bool canLand();
        bool canArm();
        bool canDisarm();
        bool canTakeoff(float altitude);

        rclcpp::Subscription<iii_drone_interfaces::msg::CombinedDroneAwareness>::SharedPtr combined_drone_awareness_sub_;
        utils::History<adapters::CombinedDroneAwarenessAdapter> combined_drone_awareness_adapter_history_;

    };

} // namespace px4
} // namespace iii_drone