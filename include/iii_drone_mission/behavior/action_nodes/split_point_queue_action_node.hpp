#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorators/loop_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class SplitPointQueueActionNode : public BT::SyncActionNode {
    public:
        SplitPointQueueActionNode(
            const std::string & name,
            const BT::NodeConfig & config
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;
    };

} // namespace behavior
} // namespace iii_drone
