#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorator_node.h>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class StringEqualsConditionNode : public BT::SyncActionNode {
    public:
        StringEqualsConditionNode(
            const std::string & name,
            const BT::NodeConfig & conf
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
    };

    class RetryUntilSuccessfulOnAbortedDecorator : public BT::DecoratorNode {
    public:
        RetryUntilSuccessfulOnAbortedDecorator(
            const std::string & name,
            const BT::NodeConfig & conf
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

        void halt() override;

    private:
        int attempts_ = 0;
        bool guarded_action_accepted_ = false;
    };

} // namespace behavior
} // namespace iii_drone
