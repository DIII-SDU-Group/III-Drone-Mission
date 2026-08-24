#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <string>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/mission/runtime_intent_buffer.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class ApplyPendingIntentUpdatesActionNode : public BT::SyncActionNode {
    public:
        ApplyPendingIntentUpdatesActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            std::shared_ptr<iii_drone::mission::RuntimeIntentBuffer> runtime_intent_buffer,
            BT::Blackboard::Ptr global_blackboard,
            std::shared_ptr<rclcpp::Node> node
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        std::shared_ptr<iii_drone::mission::RuntimeIntentBuffer> runtime_intent_buffer_;
        BT::Blackboard::Ptr global_blackboard_;
        std::shared_ptr<rclcpp::Node> node_;
    };

    class SetBlackboardBoolActionNode : public BT::SyncActionNode {
    public:
        SetBlackboardBoolActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            BT::Blackboard::Ptr global_blackboard
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        BT::Blackboard::Ptr global_blackboard_;
    };

    class BlackboardBoolConditionNode : public BT::SyncActionNode {
    public:
        BlackboardBoolConditionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            BT::Blackboard::Ptr global_blackboard
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        BT::Blackboard::Ptr global_blackboard_;
    };

    class SetBlackboardStringActionNode : public BT::SyncActionNode {
    public:
        SetBlackboardStringActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            BT::Blackboard::Ptr global_blackboard
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        BT::Blackboard::Ptr global_blackboard_;
    };

    class BlackboardStringEqualsConditionNode : public BT::SyncActionNode {
    public:
        BlackboardStringEqualsConditionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            BT::Blackboard::Ptr global_blackboard
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        BT::Blackboard::Ptr global_blackboard_;
    };

} // namespace behavior
} // namespace iii_drone
