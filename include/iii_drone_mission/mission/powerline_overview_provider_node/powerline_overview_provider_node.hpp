#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <string>
#include <thread>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/callback_group.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/srv/pl_mapper_command.hpp>

#include <iii_drone_interfaces/msg/powerline.hpp>

#include <iii_drone_interfaces/srv/update_powerline_overview.hpp>
#include <iii_drone_interfaces/srv/get_powerline_overview.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/atomic.hpp>
#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_core/utils/math.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/adapters/point_cloud_adapter.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace mission {
namespace powerline_overview_provider_node {

    class PowerlineOverviewProviderNode : public rclcpp_lifecycle::LifecycleNode {
    public:
        PowerlineOverviewProviderNode(
            std::string node_name = "powerline_overview_provider",
            std::string node_namespace = "/mission/powerline_overview_provider",
            const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        );

        ~PowerlineOverviewProviderNode();

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
        rclcpp::Client<iii_drone_interfaces::srv::PLMapperCommand>::SharedPtr pl_mapper_command_client_;

        rclcpp::CallbackGroup::SharedPtr cb_group_1_;

        rclcpp::Subscription<iii_drone_interfaces::msg::Powerline>::SharedPtr powerline_sub_;

        utils::Atomic<iii_drone_interfaces::msg::Powerline> latest_powerline_;
        utils::Atomic<iii_drone_interfaces::msg::Powerline> stored_powerline_;
        utils::Atomic<iii_drone::adapters::PowerlineAdapter> stored_powerline_adapter_;

        utils::Atomic<bool> has_stored_powerline_ = false;

        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr stored_powerline_points_pub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr stored_powerline_pose_pub_;
        rclcpp::TimerBase::SharedPtr stored_powerline_points_timer_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        rclcpp::Service<iii_drone_interfaces::srv::UpdatePowerlineOverview>::SharedPtr update_powerline_overview_srv_;
        rclcpp::Service<iii_drone_interfaces::srv::GetPowerlineOverview>::SharedPtr get_powerline_overview_srv_;

        void updatePowerlineOverviewCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::UpdatePowerlineOverview::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::UpdatePowerlineOverview::Response> response
        );

        void getPowerlineOverviewCallback(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<iii_drone_interfaces::srv::GetPowerlineOverview::Request> request,
            std::shared_ptr<iii_drone_interfaces::srv::GetPowerlineOverview::Response> response
        );

    };

} // namespace powerline_overview_provider_node
} // namespace mission
} // namespace iii_drone


/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char ** argv);