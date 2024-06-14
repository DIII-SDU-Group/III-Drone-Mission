#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <chrono>
#include <memory>
#include <thread>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/trees/tree_executor.hpp>

#include <iii_drone_mission/mission/mission_specification.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class TreeProviderIterator {
    public:
        typedef std::map<std::string, iii_drone::behavior::TreeExecutor::SharedPtr>::iterator iterator;

        TreeProviderIterator(iterator it);

        iii_drone::behavior::TreeExecutor::SharedPtr operator*() const;
        TreeProviderIterator& operator++();
        bool operator!=(const TreeProviderIterator& other) const;

    private:
        iterator it_;

    };

    class TreeProvider : public rclcpp::Node {
    public:
        TreeProvider(
            tf2_ros::Buffer::SharedPtr tf_buffer,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            iii_drone::mission::MissionSpecification::SharedPtr mission_specification
        );

        void FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor);

        TreeExecutor::SharedPtr GetTreeExecutor(const std::string& name) const;

        TreeProviderIterator begin();
        TreeProviderIterator end();

        typedef std::shared_ptr<TreeProvider> SharedPtr;

    private:
        tf2_ros::Buffer::SharedPtr tf_buffer_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::mission::MissionSpecification::SharedPtr mission_specification_;

        iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator_;

        std::map<std::string, iii_drone::behavior::TreeExecutor::SharedPtr> tree_executors_;

        void initializeTreeExecutors();

    };

} // namespace behavior
} // namespace iii_drone