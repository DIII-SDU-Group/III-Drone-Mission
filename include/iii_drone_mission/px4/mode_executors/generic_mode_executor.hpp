#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <mutex>
#include <thread>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

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

#include <iii_drone_mission/px4/mode_executors/handoff_failsafe_deferral.hpp>

#include <iii_drone_mission/px4/stick_takeover.hpp>

#include <iii_drone_mission/mission/mission_exit.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/action/mode_executor_action.hpp>

#include <iii_drone_interfaces/msg/combined_drone_awareness.hpp>

#include <iii_drone_interfaces/srv/pl_mapper_command.hpp>


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
            iii_drone::configuration::Configuration::SharedPtr parameters
        ); 
        
        ~GenericModeExecutor() override;

        void onActivate() override;

        void onDeactivate(DeactivateReason reason) override;

        bool active() const;
        std::string current_mode_key() const;

        typedef std::shared_ptr<GenericModeExecutor> SharedPtr;

        typedef std::unique_ptr<GenericModeExecutor> UniquePtr;

    private:
        rclcpp::Node & node_;

        std::string mode_executor_name_;

        iii_drone::mission::MissionSpecification::SharedPtr mission_specification_;

        iii_drone::configuration::Configuration::SharedPtr configuration_;

        ModeProvider::SharedPtr mode_provider_;

        utils::Atomic<bool> is_active_ = false;
        utils::Atomic<bool> triggered_position_control_ = false;

        utils::Atomic<ManeuverMode::SharedPtr> current_mode_;
        utils::Atomic<iii_drone::mission::mission_specification_entry_t> current_mode_entry_;

        void onModeCompleted(px4_ros2::Result result);

        void handleModeCompleted(px4_ros2::Result result);

        /**
         * Mission Exit: PX4 took command authority away from this executor
         * for anything the mission did not initiate. Runs the ordered cleanup
         * of MissionExitSteps once per run (first observer wins).
         */
        void triggerMissionExit(const iii_drone::mission::MissionExitDecision & decision);

        void onMissionExitVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

        void completeActionGoalForMissionExit();

        void sendPlMapperExitCommand(uint8_t command);

        std::mutex mission_exit_mutex_;
        rclcpp::CallbackGroup::SharedPtr mission_exit_callback_group_;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mission_exit_vehicle_status_sub_;
        rclcpp::Client<iii_drone_interfaces::srv::PLMapperCommand>::SharedPtr pl_mapper_command_client_;
        rclcpp::TimerBase::SharedPtr deferred_deactivation_timer_;
        // The Mission Exit monitor runs on its own executor thread so no
        // other callback of the Mission process can delay its latest sample.
        std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> mission_exit_executor_;
        std::thread mission_exit_thread_;

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

        StickTakeoverDetector stick_takeover_detector_;

        /**
         * PX4 failsafes are deferred only while a mode handoff is pending:
         * from scheduling a mode until PX4 runs it.
         */
        HandoffFailsafeDeferral handoff_failsafe_deferral_;
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr handoff_vehicle_status_sub_;

        void deferFailsafesForHandoff(uint8_t target_nav_state, const char * handoff);
        void releaseHandoffFailsafeDeferral(const char * reason);
        void onHandoffVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

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

        void clearGlobalBlackboard(const std::string & reason);

    };

} // namespace px4
} // namespace iii_drone
