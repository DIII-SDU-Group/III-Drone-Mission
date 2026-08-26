#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/history.hpp>
#include <iii_drone_core/utils/types.hpp>

#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/mission/mission_specification.hpp>
#include <iii_drone_mission/mission/runtime_intent_buffer.hpp>

#include <iii_drone_mission/behavior/trees/tree_provider.hpp>

#include <iii_drone_mission/px4/modes/mode_provider.hpp>
#include <iii_drone_mission/px4/mode_executors/generic_mode_executor.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_odometry.hpp>

/*****************************************************************************/
// PX4-ROS2:

#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/mode.hpp>

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>
#include <iii_drone_interfaces/msg/mission_intent_status.hpp>

#include <std_srvs/srv/set_bool.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    class MissionExecutor {
    public:
        explicit MissionExecutor(
            rclcpp_lifecycle::LifecycleNode * node,
            tf2_ros::Buffer::SharedPtr tf_buffer,
            MissionSpecification::SharedPtr mission_specification,
            rclcpp::CallbackGroup::SharedPtr odometry_sub_callback_group,
            rclcpp::executors::MultiThreadedExecutor & executor
        );

        ~MissionExecutor();

        void Configure(
            iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator,
            rclcpp::CallbackGroup::SharedPtr get_reference_cb_group
        );
        void Cleanup();
        void Start(
            iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator
        );
        void Stop();
        bool SelectMissionSpecification(
            MissionSpecification::SharedPtr mission_specification,
            iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator,
            rclcpp::CallbackGroup::SharedPtr get_reference_cb_group,
            std::string & message
        );

        const BT::BehaviorTreeFactory & factory() const {
            return tree_provider_->factory();
        }

        typedef std::shared_ptr<MissionExecutor> SharedPtr;

        MissionSpecification::SharedPtr mission_specification() const {
            return mission_specification_;
        }

        iii_drone::px4::ModeProvider::SharedPtr mode_provider() const {
            return mode_provider_;
        }

        bool mission_active() const {
            return generic_mode_executor_ != nullptr && generic_mode_executor_->active();
        }

        std::shared_ptr<RuntimeIntentBuffer> runtime_intent_buffer() const {
            return runtime_intent_buffer_;
        }

        std::optional<iii_drone::types::point_t> currentPosition() const;
        iii_drone::configuration::Configuration::SharedPtr phaseWaypointConfiguration() const;
        std::vector<iii_drone_interfaces::msg::MissionIntentStatus> intentStatuses() const;

    private:
        rclcpp_lifecycle::LifecycleNode * node_;

        tf2_ros::Buffer::SharedPtr tf_buffer_;

        MissionSpecification::SharedPtr mission_specification_;

        iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history_;

        rclcpp::CallbackGroup::SharedPtr odometry_sub_callback_group_;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;

        iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

        iii_drone::behavior::TreeProvider::SharedPtr tree_provider_;

        iii_drone::px4::ModeProvider::SharedPtr mode_provider_;

        iii_drone::px4::GenericModeExecutor::SharedPtr generic_mode_executor_;

        rclcpp::executors::MultiThreadedExecutor & executor_;

        bool is_started_ = false;
        bool is_configured_ = false;
        mutable std::mutex lifecycle_mutex_;

        std::shared_ptr<RuntimeIntentBuffer> runtime_intent_buffer_;
        std::map<std::string, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> intent_services_;
        mutable std::mutex intent_status_mutex_;
        std::map<std::string, iii_drone_interfaces::msg::MissionIntentStatus> intent_statuses_;

        void registerIntentServices();
        void unregisterIntentServices();
        bool intentServiceValidForCurrentMode(const mission_intent_service_t & intent_service) const;
        std::string activeModeKey() const;
        bool rebuildWithMissionSpecification(
            MissionSpecification::SharedPtr mission_specification,
            bool configure_after_rebuild,
            bool start_after_rebuild,
            iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator,
            rclcpp::CallbackGroup::SharedPtr get_reference_cb_group,
            std::string & message
        );

    };

} // namespace mission
} // namespace iii_drone
