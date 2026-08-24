#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

/*****************************************************************************/
// III-Drone-Core:


/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/single_line_adapter.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
// #include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/plugins.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class CableAxisYawSettler {
    public:
        std::optional<double> addSample(
            double axis_yaw,
            std::chrono::steady_clock::time_point received_at,
            double window_s,
            double max_yaw_deviation_rad,
            std::size_t min_samples
        );

        void reset();

        std::size_t sampleCount() const;

        double sampleSpanSeconds() const;

        double maxDeviationRadians() const;

    private:
        struct YawSample {
            double axis_yaw;
            std::chrono::steady_clock::time_point received_at;
        };

        double meanCableAxisYaw() const;

        std::deque<YawSample> samples_;

        double max_deviation_rad_ = 0.0;
    };

    class GetGripperAlignmentYawConditionNode : public BT::RosTopicSubNode<iii_drone_interfaces::msg::Powerline> {
    public:
        GetGripperAlignmentYawConditionNode(
            const std::string & name, 
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) override;

    private:
        void resetSettlingState();

        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        int settling_target_cable_id_ = -1;

        int64_t last_line_stamp_ns_ = -1;

        CableAxisYawSettler yaw_settler_;

    };

} // namespace behavior
}  // namespace iii_drone
