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
#include <iii_drone_mission/mission/runtime_intent_buffer.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/blackboard.h>

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
            iii_drone::mission::MissionSpecification::SharedPtr mission_specification,
            std::shared_ptr<iii_drone::mission::RuntimeIntentBuffer> runtime_intent_buffer
        );

        void Configure(
            iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client
        );
        void Cleanup();
        void SetMissionSpecification(
            iii_drone::mission::MissionSpecification::SharedPtr mission_specification
        );

        TreeExecutor::SharedPtr GetTreeExecutor(const std::string& name) const;
        void ClearGlobalBlackboard(const std::string & reason);

        const BT::BehaviorTreeFactory & factory() const {
            return tree_executors_.begin()->second->factory();
        }

        TreeProviderIterator begin();
        TreeProviderIterator end();

        typedef std::shared_ptr<TreeProvider> SharedPtr;

    private:
        tf2_ros::Buffer::SharedPtr tf_buffer_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::mission::MissionSpecification::SharedPtr mission_specification_;
        std::shared_ptr<iii_drone::mission::RuntimeIntentBuffer> runtime_intent_buffer_;

        iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator_;

        BT::Blackboard::Ptr global_blackboard_;

        std::map<std::string, iii_drone::behavior::TreeExecutor::SharedPtr> tree_executors_;

        void initializeTreeExecutors();

        bool is_configured_ = false;

    };

} // namespace behavior
} // namespace iii_drone
