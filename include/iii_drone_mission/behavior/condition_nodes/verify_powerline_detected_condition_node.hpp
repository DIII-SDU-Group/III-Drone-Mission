#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/single_line.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Powerline subscription condition node, subscribes to the powerline topic
     * and verifies if a powerline has been detected.
     */
    class VerifyPowerlineDetectedConditionNode : public BT::RosTopicSubNode<iii_drone_interfaces::msg::Powerline> {
    public:
        /**
         * @brief Constructor.
         * 
         * @param name The name of the node.
         * @param conf The node configuration.
         * @param params The ROS node parameters.
         */
        VerifyPowerlineDetectedConditionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        BT::NodeStatus verifyLineMatchesPowerlineOverview(
            const iii_drone_interfaces::msg::Powerline & detected_powerline,
            const iii_drone_interfaces::msg::Powerline & powerline_overview,
            int powerline_overview_required_line_id,
            double line_match_distance_threshold_m,
            double relaxed_line_match_distance_threshold_m,
            double line_match_ambiguity_margin_m,
            int & matched_detected_line_id,
            double & matched_line_distance_m
        ) const;

        static bool getOverviewLinePoint(
            const iii_drone_interfaces::msg::Powerline & powerline_overview,
            int line_id,
            iii_drone::types::point_t & point
        );

        static double distanceInPlaneOrthogonalToDirection(
            const iii_drone::types::point_t & a,
            const iii_drone::types::point_t & b,
            const iii_drone::types::vector_t & direction
        );

        bool transformLinePointToFrame(
            const iii_drone_interfaces::msg::SingleLine & line,
            const std::string & target_frame_id,
            iii_drone::types::point_t & point
        ) const;

    };

} // namespace behavior
}  // namespace iii_drone
