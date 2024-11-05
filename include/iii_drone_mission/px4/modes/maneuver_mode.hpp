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

#include <iii_drone_interfaces/srv/register_offboard_mode.hpp>

/*****************************************************************************/
// PX4-ROS2:

#include <px4_ros2/components/mode.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace px4 {

    class ManeuverMode : public px4_ros2::ModeBase {

    public:
        explicit ManeuverMode(
            rclcpp::Node & node,
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

        void StopExecution();

        void updateSetpoint(float dt) override;

        std::string mode_name() const;

        typedef std::shared_ptr<ManeuverMode> SharedPtr;

        typedef std::unique_ptr<ManeuverMode> UniquePtr;
    
    private:
        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::behavior::TreeExecutor::SharedPtr tree_executor_;

        std::shared_ptr<iii_drone::px4::TrajectorySetpoint> traj_setpoint_;

        std::string mode_name_;

        float dt_;

        bool is_owned_mode_;

        bool is_registered_ = false;

        utils::Atomic<bool> stay_alive_on_next_deactivate_ = false;

        std::function<void()> on_next_activate_callback_ = nullptr;

        rclcpp::Client<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedPtr register_offboard_mode_client_;

        void sendRegisterOffboardModeRequest(
            bool deregister,
            bool force = false
        );

    };


} // namespace px4
} // namespace iii_drone