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
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/mission/mission_specification.hpp>

#include <iii_drone_mission/behavior/trees/tree_provider.hpp>

#include <iii_drone_mission/px4/modes/mode_provider.hpp>
#include <iii_drone_mission/px4/mode_executors/generic_mode_executor.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_odometry.hpp>

/*****************************************************************************/
// PX4-ROS2:

#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/mode.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    class MissionExecutor {
    public:
        explicit MissionExecutor(
            rclcpp_lifecycle::LifecycleNode * node,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            std::string mission_specification_file,
            float dt
        );

        void FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor);
        void Configure(
            iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator
        );
        void Cleanup();
        void Start();
        void Stop();

        typedef std::shared_ptr<MissionExecutor> SharedPtr;

    private:
        rclcpp_lifecycle::LifecycleNode * node_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        MissionSpecification::SharedPtr mission_specification_;

        iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history_;

        rclcpp::CallbackGroup::SharedPtr odometry_sub_callback_group_;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::behavior::TreeProvider::SharedPtr tree_provider_;

        iii_drone::px4::ModeProvider::SharedPtr mode_provider_;

        iii_drone::px4::GenericModeExecutor::SharedPtr generic_mode_executor_;

    };

} // namespace mission
} // namespace iii_drone