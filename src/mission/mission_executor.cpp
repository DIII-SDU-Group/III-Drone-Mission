/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_executor.hpp>

using namespace iii_drone::utils;
using namespace iii_drone::mission;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::adapters::px4;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MissionExecutor::MissionExecutor(
    rclcpp::Node * node,
    iii_drone::configuration::Configurator<rclcpp::Node>::SharedPtr configurator,
    tf2_ros::Buffer::SharedPtr tf_buffer
) : node_(node),
    configurator_(configurator),
    tf_buffer_(tf_buffer)
{

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initializing.");

    // Get the mission specification:
    std::string mission_specification_file = configurator_->GetParameter("mission_specification_file").as_string();

    mission_specification_ = std::make_shared<MissionSpecification>(mission_specification_file);

    // Subscription
    vehicle_odometry_adapter_history_ = std::make_shared<History<VehicleOdometryAdapter>>();

	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    odometry_sub_ = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        px4_sub_qos,
        [&](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            vehicle_odometry_adapter_history_->Store(VehicleOdometryAdapter(*msg));
        }
    );

    // Maneuver reference client
    maneuver_reference_client_ = std::make_shared<ManeuverReferenceClient>(
        node_,
        vehicle_odometry_adapter_history_,
        configurator_->GetParameterBundle("maneuver_reference_client"),
        [&]() -> void {
            RCLCPP_ERROR(node->get_logger(), "ManeuverReferenceClient::on_fail_during_maneuver(): Failed to get reference.");
        }
    );

    // Tree provider
    tree_provider_ = std::make_shared<iii_drone::behavior::TreeProvider>(
        tf_buffer_,
        maneuver_reference_client_,
        mission_specification_
    );

    // Modes provider
    mode_provider_ = std::make_shared<iii_drone::px4::ModeProvider>(
        tree_provider_,
        maneuver_reference_client_,
        mission_specification_,
        configurator_->GetParameterBundle("mode_provider"),
        node_
    );

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initialized.");

}

void MissionExecutor::FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor) {

    tree_provider_->FinalizeInitialization(executor);

}