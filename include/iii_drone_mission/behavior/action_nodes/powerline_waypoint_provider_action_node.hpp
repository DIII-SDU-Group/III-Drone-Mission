#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <deque>
#include <limits>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/adapters/target_adapter.hpp>
#include <iii_drone_core/adapters/powerline_adapter.hpp>

#include <iii_drone_core/control/state.hpp>

#include <iii_drone_core/utils/math.hpp>
#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/combined_drone_awareness.hpp>
#include <iii_drone_interfaces/msg/pylon_overview.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/latest_message_subscription.hpp>
#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorators/loop_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Powerline waypoint provider action node.
     */
    class PowerlineWaypointProviderActionNode : public BT::SyncActionNode {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param tf_buffer The tf buffer.
         * @param node The ROS2 node.
         * @param params Read-only live configuration view
         * @param combined_drone_awareness Awareness cache that outlives the
         *        tree. This node reads it once, synchronously, right after
         *        its tree is built: a subscription of its own had not always
         *        received its first sample by then, and the ground-relative
         *        waypoint floor silently fell back to an absolute height
         *        (HIL soak run 37).
         */
        typedef LatestMessageSubscription<iii_drone_interfaces::msg::CombinedDroneAwareness>
            CombinedDroneAwarenessCache;

        PowerlineWaypointProviderActionNode(
            const std::string & name, 
            const BT::NodeConfiguration & conf,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            rclcpp::Node * node,
            iii_drone::configuration::Configuration::SharedPtr params,
            std::shared_ptr<CombinedDroneAwarenessCache> combined_drone_awareness
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        int lineIdForPoint(
            const iii_drone::adapters::PowerlineAdapter & powerline_adapter,
            const iii_drone::types::point_t & point
        ) const;

        double minimumWaypointZ() const;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        rclcpp::Node * node_;

        iii_drone::configuration::Configuration::SharedPtr configuration_;

        std::shared_ptr<CombinedDroneAwarenessCache> combined_drone_awareness_;

        // std::shared_ptr<std::deque<iii_drone::types::point_t>> applyLinearInterpolation(
        //     std::shared_ptr<std::deque<iii_drone::types::point_t>> points
        // );

    };

}
}
