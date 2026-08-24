#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <functional>
#include <cstdint>
#include <memory>
#include <string>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/utils/atomic.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp>

#include <iii_drone_mission/behavior/trees/tree_executor.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/string_stamped.hpp>

#include <iii_drone_interfaces/srv/register_offboard_mode.hpp>

/*****************************************************************************/
// PX4-ROS2:

#include <px4_ros2/components/mode.hpp>

/*****************************************************************************/
// PX4 messages:

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace px4 {

    class ManeuverMode : public px4_ros2::ModeBase {

    public:
        explicit ManeuverMode(
            rclcpp::Node & node,
            std::string mode_key,
            std::string mode_name,
            float dt,
            bool is_owned_mode,
            bool allow_activate_when_disarmed
        );

        // ~ManeuverMode() override;

        void Register(
            iii_drone::behavior::TreeExecutor::SharedPtr tree_executor,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
        );
        void Unregister(bool force = false);

        void onActivate() override;

        void onDeactivate() override;

        void StayAliveOnNextDeactivate();
        void ClearStayAliveOnNextDeactivate();

        void RegisterOnNextActivateCallback(std::function<void()> callback);

        void StopControls();
        void StartControls();

        void StopExecution();

        void updateSetpoint(float dt) override;

        std::string mode_name() const;
        std::string mode_key() const;
        uint8_t mode_id() const;
        bool is_registered() const;
        bool active() const;
        bool tree_running() const;
        bool tree_finished() const;
        bool tree_success() const;
        bool emergency_reference_hold_active() const;
        std::string degraded_reason() const;

        typedef std::shared_ptr<ManeuverMode> SharedPtr;

        typedef std::unique_ptr<ManeuverMode> UniquePtr;
    
    private:
        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::behavior::TreeExecutor::SharedPtr tree_executor_;

        std::shared_ptr<iii_drone::px4::TrajectorySetpoint> traj_setpoint_;

        std::string mode_name_;

        std::string mode_key_;

        float dt_;

        bool is_owned_mode_;

        utils::Atomic<bool> is_registered_ = false;

        utils::Atomic<bool> offboard_mode_registered_ = false;

        utils::Atomic<bool> active_ = false;

        utils::Atomic<bool> stay_alive_on_next_deactivate_ = false;

        utils::Atomic<bool> stop_controls_ = false;

        utils::Atomic<bool> tree_completion_reported_ = false;

        utils::Atomic<bool> emergency_reference_hold_active_ = false;

        std::function<void()> on_next_activate_callback_ = nullptr;

        rclcpp::Client<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedPtr register_offboard_mode_client_;

        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

        rclcpp::Publisher<iii_drone_interfaces::msg::StringStamped>::SharedPtr status_publisher_;

        rclcpp::TimerBase::SharedPtr status_timer_;

        bool sendRegisterOffboardModeRequest(
            bool deregister,
            bool force = false
        );

        void publishHoldCommand();

        void startExecutionIfReady();

        void publishStatus();

    };


} // namespace px4
} // namespace iii_drone
