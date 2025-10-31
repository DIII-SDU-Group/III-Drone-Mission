#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/parameter_bundle.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>

#include <iii_drone_mission/behavior/trees/tree_provider.hpp>

#include <iii_drone_mission/mission/mission_specification.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace px4 {

    class ModeProviderIterator {
    public:
        typedef std::map<std::string, iii_drone::px4::ManeuverMode::SharedPtr>::iterator iterator;

        ModeProviderIterator(iterator it);

        iii_drone::px4::ManeuverMode::SharedPtr operator*() const;
        ModeProviderIterator& operator++();
        bool operator!=(const ModeProviderIterator& other) const;

    private:
        iterator it_;

    };

    class ModeProvider {
    public:
        ModeProvider(
            iii_drone::behavior::TreeProvider::SharedPtr tree_provider,
            iii_drone::mission::MissionSpecification::SharedPtr mission_specification,
            rclcpp_lifecycle::LifecycleNode * node,
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
            iii_drone::configuration::ParameterBundle::SharedPtr parameters
        );

        void Register();

        void Cleanup();
        void Stop();

        iii_drone::px4::ManeuverMode::SharedPtr GetMode(const std::string& name) const;

        ModeProviderIterator begin();
        ModeProviderIterator end();

        rclcpp::Node::SharedPtr mode_node() const;

        typedef std::shared_ptr<ModeProvider> SharedPtr;

    private:
        iii_drone::behavior::TreeProvider::SharedPtr tree_provider_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::mission::MissionSpecification::SharedPtr mission_specification_;

        iii_drone::configuration::ParameterBundle::SharedPtr parameters_;

        rclcpp_lifecycle::LifecycleNode * node_;
        rclcpp::Node::SharedPtr mode_node_;

        std::map<std::string, iii_drone::px4::ManeuverMode::SharedPtr> modes_;

        void initializeModes();
        void deinitializeModes();

    };

} // namespace behavior
} // namespace iii_drone