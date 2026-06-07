#pragma once

#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_interfaces/msg/pylon_overview.hpp>
#include <iii_drone_interfaces/msg/string_stamped.hpp>
#include <iii_drone_interfaces/srv/clear_pylon_overview.hpp>
#include <iii_drone_interfaces/srv/get_pylon_overview.hpp>
#include <iii_drone_interfaces/srv/store_pylon_overview.hpp>
#include <iii_drone_mission/mission/overview_gnss_persistence.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>

namespace iii_drone {
namespace mission {
namespace pylon_overview_provider_node {

    class PylonOverviewProviderNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        PylonOverviewProviderNode(
            std::string node_name = "pylon_overview_provider",
            std::string node_namespace = "/mission/pylon_overview_provider",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & state
        ) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & state
        ) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & state
        ) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & state
        ) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State & state
        ) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
            const rclcpp_lifecycle::State & state
        ) override;

    private:
        mutable std::mutex mutex_;
        std::map<int32_t, iii_drone_interfaces::msg::Pylon> pylons_;
        std::string frame_id_ = "world";
        bool has_persisted_gnss_pylons_ = false;
        std::filesystem::path gnss_persistence_path_;
        iii_drone::utils::Atomic<px4_msgs::msg::VehicleGlobalPosition> latest_global_position_;

        rclcpp::Service<iii_drone_interfaces::srv::StorePylonOverview>::SharedPtr store_pylon_overview_srv_;
        rclcpp::Service<iii_drone_interfaces::srv::GetPylonOverview>::SharedPtr get_pylon_overview_srv_;
        rclcpp::Service<iii_drone_interfaces::srv::ClearPylonOverview>::SharedPtr clear_pylon_overview_srv_;
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp_lifecycle::LifecyclePublisher<iii_drone_interfaces::msg::StringStamped>::SharedPtr status_pub_;
        rclcpp::TimerBase::SharedPtr status_timer_;

        bool validLocked() const;
        iii_drone_interfaces::msg::PylonOverview overviewLocked() const;
        void replaceOverviewLocked(const iii_drone_interfaces::msg::PylonOverview & overview);
        bool persistOverview(const iii_drone_interfaces::msg::PylonOverview & overview);
        bool loadPersistedOverviewToMemoryLocked();

        void storePylonOverviewCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::StorePylonOverview::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::StorePylonOverview::Response> response
        );

        void getPylonOverviewCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::GetPylonOverview::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::GetPylonOverview::Response> response
        );

        void clearPylonOverviewCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::ClearPylonOverview::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::ClearPylonOverview::Response> response
        );
    };

} // namespace pylon_overview_provider_node
} // namespace mission
} // namespace iii_drone

int main(int argc, char ** argv);
