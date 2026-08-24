#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <cstdint>
#include <filesystem>
#include <string>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/get_rosbag_recording_status.hpp>
#include <iii_drone_interfaces/srv/start_rosbag_recording.hpp>
#include <iii_drone_interfaces/srv/stop_rosbag_recording.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace mission {
namespace rosbag_recorder_node {

    class RosbagRecorderNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        RosbagRecorderNode(
            std::string node_name = "rosbag_recorder",
            std::string node_namespace = "/mission/rosbag_recorder",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

        ~RosbagRecorderNode();

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & state
        );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & state
        );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & state
        );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & state
        );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State & state
        );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
            const rclcpp_lifecycle::State & state
        );

    private:
        rclcpp::Service<iii_drone_interfaces::srv::StartRosbagRecording>::SharedPtr start_recording_srv_;
        rclcpp::Service<iii_drone_interfaces::srv::StopRosbagRecording>::SharedPtr stop_recording_srv_;
        rclcpp::Service<iii_drone_interfaces::srv::GetRosbagRecordingStatus>::SharedPtr recording_status_srv_;

        std::filesystem::path artifact_root_;
        std::filesystem::path log_root_;
        double default_stop_timeout_sec_ = 10.0;

        pid_t child_pid_ = -1;
        std::string recording_id_;
        std::filesystem::path output_dir_;
        std::string started_at_;
        std::string owner_;
        std::string last_error_;

        void startRecordingCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::StartRosbagRecording::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::StartRosbagRecording::Response> response
        );

        void stopRecordingCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::StopRosbagRecording::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::StopRosbagRecording::Response> response
        );

        void recordingStatusCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::GetRosbagRecordingStatus::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::GetRosbagRecordingStatus::Response> response
        );

        bool isRecording();
        bool stopRecording(double timeout_sec, std::string & message, bool & was_running);
        void clearRecordingState();
        std::string makeRecordingId(const std::string & prefix) const;
        std::string sanitizeRecordingId(const std::string & recording_id) const;
        std::uint64_t outputSizeBytes() const;
        void fillStatus(iii_drone_interfaces::srv::GetRosbagRecordingStatus::Response & response);
    };

} // namespace rosbag_recorder_node
} // namespace mission
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char ** argv);
