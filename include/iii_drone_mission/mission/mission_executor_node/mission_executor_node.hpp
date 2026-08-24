#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <fstream>
#include <string>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/write_behavior_tree_model_xml.hpp>
#include <iii_drone_interfaces/srv/override_mission_specification.hpp>
#include <iii_drone_interfaces/msg/mission_mode_status.hpp>
#include <iii_drone_interfaces/srv/get_powerline_overview.hpp>
#include <iii_drone_interfaces/srv/get_pylon_overview.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/mission/mission_executor.hpp>

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>

/*****************************************************************************/
// BehaviorTree.CPP:

#include <behaviortree_cpp/xml_parsing.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    class MissionExecutorNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit MissionExecutorNode(
            rclcpp::executors::MultiThreadedExecutor & executor_handle,
            std::string node_name = "mission_executor",
            std::string node_namespace = "/mission/mission_executor",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

        ~MissionExecutorNode();

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
        iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator_;

        MissionExecutor::SharedPtr mission_executor_;

        rclcpp::Service<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML>::SharedPtr write_behavior_tree_model_xml_service_;
        rclcpp::Service<iii_drone_interfaces::srv::OverrideMissionSpecification>::SharedPtr override_mission_specification_service_;
        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::MissionModeStatus>::SharedPtr mission_status_publisher_;
        rclcpp::TimerBase::SharedPtr mission_status_timer_;
        rclcpp::Client<iii_drone_interfaces::srv::GetPowerlineOverview>::SharedPtr powerline_overview_client_;
        rclcpp::Client<iii_drone_interfaces::srv::GetPylonOverview>::SharedPtr pylon_overview_client_;
        iii_drone_interfaces::srv::GetPowerlineOverview::Response::SharedPtr powerline_overview_response_;
        iii_drone_interfaces::srv::GetPylonOverview::Response::SharedPtr pylon_overview_response_;
        std::atomic<bool> powerline_overview_request_pending_{false};
        std::atomic<bool> pylon_overview_request_pending_{false};
        mutable std::mutex inspection_overview_mutex_;
        std::string mission_status_degraded_reason_;
        std::string default_mission_specification_file_;
        std::string mission_specification_file_;

        void writeBehaviorTreeModelXmlService(
            const std::shared_ptr<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML::Response> response
        );
        void overrideMissionSpecificationService(
            const std::shared_ptr<iii_drone_interfaces::srv::OverrideMissionSpecification::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::OverrideMissionSpecification::Response> response
        );

        tf2_ros::Buffer::SharedPtr tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::executors::MultiThreadedExecutor & executor_handle_;

        void cleanup();
        void publishMissionModeStatus();
        void refreshInspectionOverviewCaches();
        void populateInspectionStartEligibility(iii_drone_interfaces::msg::MissionModeStatus & msg);
        std::vector<std::string> requiredMissionModes() const;
        std::vector<std::string> registeredMissionModes() const;
        bool requiredMissionModesRegistered() const;

        rclcpp::CallbackGroup::SharedPtr odometry_sub_callback_group_;
        rclcpp::CallbackGroup::SharedPtr get_reference_cb_group_;

    };

} // namespace mission
} // namespace iii_drone


/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char **argv);
