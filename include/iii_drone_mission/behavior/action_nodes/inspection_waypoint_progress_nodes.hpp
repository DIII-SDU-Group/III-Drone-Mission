#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/blackboard.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class InitializeInspectionWaypointsActionNode : public BT::SyncActionNode {
    public:
        InitializeInspectionWaypointsActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            BT::Blackboard::Ptr global_blackboard
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        BT::Blackboard::Ptr global_blackboard_;
    };

    class GetCurrentInspectionWaypointActionNode : public BT::SyncActionNode {
    public:
        GetCurrentInspectionWaypointActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            BT::Blackboard::Ptr global_blackboard
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        BT::Blackboard::Ptr global_blackboard_;
    };

    class AdvanceInspectionWaypointActionNode : public BT::SyncActionNode {
    public:
        AdvanceInspectionWaypointActionNode(
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
