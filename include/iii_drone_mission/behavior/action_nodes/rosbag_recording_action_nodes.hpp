#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/start_rosbag_recording.hpp>
#include <iii_drone_interfaces/srv/stop_rosbag_recording.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_ros2/bt_service_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <behaviortree_cpp/decorator_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    class StartRosbagRecordingActionNode : public BT::RosServiceNode<iii_drone_interfaces::srv::StartRosbagRecording> {
    public:
        StartRosbagRecordingActionNode(
            const std::string & name,
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params
        );

        static BT::PortsList providedPorts();

        bool setRequest(Request::SharedPtr & request) override;

        BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;
    };

    class StopRosbagRecordingActionNode : public BT::RosServiceNode<iii_drone_interfaces::srv::StopRosbagRecording> {
    public:
        StopRosbagRecordingActionNode(
            const std::string & name,
            const BT::NodeConfig & conf,
            const BT::RosNodeParams & params
        );

        static BT::PortsList providedPorts();

        bool setRequest(Request::SharedPtr & request) override;

        BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override;

    private:
        rclcpp::Node::SharedPtr node_ptr_;
    };

    class RosbagRecordingScopeDecorator : public BT::DecoratorNode {
    public:
        RosbagRecordingScopeDecorator(
            const std::string & name,
            const BT::NodeConfig & conf,
            rclcpp::Node::SharedPtr node,
            std::string start_service_name,
            std::string stop_service_name,
            std::chrono::milliseconds server_timeout,
            std::chrono::milliseconds wait_for_server_timeout
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

        void halt() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<iii_drone_interfaces::srv::StartRosbagRecording>::SharedPtr start_client_;
        rclcpp::Client<iii_drone_interfaces::srv::StopRosbagRecording>::SharedPtr stop_client_;
        std::chrono::milliseconds server_timeout_;
        std::chrono::milliseconds wait_for_server_timeout_;
        bool recording_started_ = false;

        bool stopActiveRecording(bool require_success);
        bool startRecording();
    };

} // namespace behavior
} // namespace iii_drone
