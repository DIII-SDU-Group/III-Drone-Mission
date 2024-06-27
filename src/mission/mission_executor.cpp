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
    rclcpp_lifecycle::LifecycleNode * node,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    std::string mission_specification_file,
    float dt
) : node_(node),
    tf_buffer_(tf_buffer)
{

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initializing.");

    mission_specification_ = std::make_shared<MissionSpecification>(
        mission_specification_file,
        node
    );

    // Subscription
    vehicle_odometry_adapter_history_ = std::make_shared<History<VehicleOdometryAdapter>>();

    odometry_sub_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.callback_group = odometry_sub_callback_group_;

    odometry_sub_ = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        px4_sub_qos,
        [&](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            vehicle_odometry_adapter_history_->Store(VehicleOdometryAdapter(*msg));
        },
        sub_opts
    );

    // Tree provider
    tree_provider_ = std::make_shared<iii_drone::behavior::TreeProvider>(
        tf_buffer_,
        mission_specification_
    );

    // Modes provider
    mode_provider_ = std::make_shared<iii_drone::px4::ModeProvider>(
        tree_provider_,
        mission_specification_,
        node_,
        dt
    );

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initialized.");

}

void MissionExecutor::FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor) {

    tree_provider_->FinalizeInitialization(executor);
    mode_provider_->FinalizeInitialization(executor);

}

void MissionExecutor::Configure(
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator
) {

    // Maneuver reference client
    maneuver_reference_client_ = std::make_shared<ManeuverReferenceClient>(
        node_,
        vehicle_odometry_adapter_history_,
        configurator->GetParameterBundle("maneuver_reference_client")
    );

    tree_provider_->Configure(
        maneuver_reference_client_
    );
    mode_provider_->Configure(
        maneuver_reference_client_, 
        configurator->GetParameterBundle("mode_provider")
    );

    generic_mode_executor_ = std::make_shared<iii_drone::px4::GenericModeExecutor>(
        *mode_provider_->GetMode(mission_specification_->executor_owned_mode()),
        "mode_executor",
        mission_specification_,
        mode_provider_,
        configurator->GetParameterBundle("mode_executor")
    );


}

void MissionExecutor::Cleanup() {

    generic_mode_executor_.reset();
    tree_provider_->Cleanup();
    mode_provider_->Cleanup();

}

void MissionExecutor::Start() {

    generic_mode_executor_->doRegister();
    mode_provider_->Start();

}

void MissionExecutor::Stop() {

    mode_provider_->Stop();

}