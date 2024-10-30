#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Mission:

#include <memory>
#include <thread>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/mission/mission_specification.hpp>

#include <iii_drone_mission/px4/modes/mode_provider.hpp>
#include <iii_drone_mission/behavior/trees/tree_provider.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace px4 {

    class PX4Handler {
    public:
        PX4Handler(
            iii_drone::mission::MissionSpecification::SharedPtr mission_specification,
            iii_drone::behavior::TreeProvider::SharedPtr tree_provider,
            float dt,
            rclcpp_lifecycle::LifecycleNode * node,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            iii_drone::configuration::ParameterBundle::SharedPtr mode_provider_parameters,
            iii_drone::configuration::ParameterBundle::SharedPtr mode_executor_parameters,
            rclcpp::executors::MultiThreadedExecutor & executor
        );

    private:

        iii_drone::mission::MissionSpecification::SharedPtr mission_specification_;

        iii_drone::behavior::TreeProvider::SharedPtr tree_provider_;

        rclcpp_lifecycle::LifecycleNode * node_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::configuration::ParameterBundle::SharedPtr mode_provider_parameters_;
        iii_drone::configuration::ParameterBundle::SharedPtr mode_executor_parameters_;

        iii_drone::px4::ModeProvider::SharedPtr mode_provider_;

    };

}
}