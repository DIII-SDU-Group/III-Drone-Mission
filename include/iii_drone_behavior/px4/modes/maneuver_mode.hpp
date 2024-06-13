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

#include <iii_drone_core/px4/setpoints/trajectory_setpoint.hpp>

#include <iii_drone_core/utils/atomic.hpp>

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
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            float dt
        );

        void RegisterAsOffboardMode();

        void onActivate() override;

        void onDeactivate() override;

        void updateSetpoint(float dt) override;

        typedef std::shared_ptr<ManeuverMode> SharedPtr;

        typedef std::unique_ptr<ManeuverMode> UniquePtr;
    
    private:
        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        std::shared_ptr<iii_drone::px4::TrajectorySetpoint> traj_setpoint_;

        std::string mode_name_;

        float dt_;

    };


} // namespace px4
} // namespace iii_drone