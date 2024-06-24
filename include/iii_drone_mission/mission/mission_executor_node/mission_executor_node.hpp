#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Mission:

#include <memory>
#include <thread>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/mission/mission_executor_node/mission_executor_node.hpp>

#include <iii_drone_mission/mission/mission_executor.hpp>

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

        tf2_ros::Buffer::SharedPtr tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::executors::MultiThreadedExecutor & executor_handle_;

        void cleanup();

    };

} // namespace mission
} // namespace iii_drone


/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char **argv);