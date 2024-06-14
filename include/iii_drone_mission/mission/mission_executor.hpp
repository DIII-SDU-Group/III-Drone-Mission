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

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Core:

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

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_odometry.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    class MissionExecutor {
    public:
        explicit MissionExecutor(
            rclcpp::Node * node,
            iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator,
            tf2_ros::Buffer::SharedPtr tf_buffer
        );

        void FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor);

    private:
        rclcpp::Node * node_;

        iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        MissionSpecification::SharedPtr mission_specification_;

        iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history_;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::behavior::TreeProvider::SharedPtr tree_provider_;

        iii_drone::px4::ModeProvider::SharedPtr mode_provider_;

    };

} // namespace mission
} // namespace iii_drone