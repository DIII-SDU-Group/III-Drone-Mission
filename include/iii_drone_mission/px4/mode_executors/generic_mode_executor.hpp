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

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/utils/atomic.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp>

#include <iii_drone_mission/mission/mission_specification.hpp>

#include <iii_drone_mission/px4/modes/mode_provider.hpp>

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

        ManeuverMode::SharedPtr current_mode_;
        iii_drone::mission::mission_specification_entry_t current_mode_entry_;

        void onModeCompleted(px4_ros2::Result result);

        iii_drone::utils::Atomic<bool> wait_for_land_;
        iii_drone::utils::Atomic<bool> wait_for_arm_;

        rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_sub_;
        void manualControlSetpointCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg);


    };

} // namespace px4
} // namespace iii_drone