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
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/px4/setpoints/trajectory_setpoint.hpp>

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
    public:
        GenericModeExecutor(
            rclcpp::Node & node, 
            px4_ros2::ModeBase & owned_mode,
            std::string mode_executor_name,
            std::function<void()> on_activate_callback,
            std::function<void(DeactivateReason)> on_deactivate_callback
        ); 
        
        void onActivate() override;

        void onDeactivate(DeactivateReason reason) override;

        typedef std::shared_ptr<GenericModeExecutor> SharedPtr;

        typedef std::unique_ptr<GenericModeExecutor> UniquePtr;

    private:
        rclcpp::Node & node_;

        std::string mode_executor_name_;

        std::function<void()> on_activate_callback_;

        std::function<void(DeactivateReason)> on_deactivate_callback_;

    };

} // namespace px4
} // namespace iii_drone