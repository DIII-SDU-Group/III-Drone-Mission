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
    rclcpp::CallbackGroup::SharedPtr odometry_sub_callback_group,
    rclcpp::executors::MultiThreadedExecutor & executor
) : node_(node),
    tf_buffer_(tf_buffer),
    odometry_sub_callback_group_(odometry_sub_callback_group),
    executor_(executor)
{

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initializing.");

    mission_specification_ = std::make_shared<MissionSpecification>(
        mission_specification_file,
        node
    );

    // Subscription
    vehicle_odometry_adapter_history_ = std::make_shared<History<VehicleOdometryAdapter>>(2);

    // odometry_sub_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

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

    executor_.add_node(tree_provider_);

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initialized.");

}

MissionExecutor::~MissionExecutor() {

    RCLCPP_INFO(node_->get_logger(), "MissionExecutor::~MissionExecutor(): Removing tree provider node from executor.");

    if (is_started_) Stop();

    if (is_configured_) Cleanup();

    executor_.remove_node(tree_provider_);

    RCLCPP_INFO(node_->get_logger(), "MissionExecutor::~MissionExecutor(): Destroying MissionExecutor.");

}

void MissionExecutor::Configure(
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator,
    rclcpp::CallbackGroup::SharedPtr get_reference_cb_group
) {

    if (is_configured_) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Configure(): Already configured.");
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Configure()");

    // Maneuver reference client
    maneuver_reference_client_ = std::make_shared<ManeuverReferenceClient>(
        node_,
        vehicle_odometry_adapter_history_,
        configurator->GetParameterBundle("maneuver_reference_client"),
        get_reference_cb_group
    );

    tree_provider_->Configure(
        maneuver_reference_client_
    );

    is_configured_ = true;

}

void MissionExecutor::Cleanup() {

    if (!is_configured_) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Cleanup(): Not configured.");
        return;
    }

    tree_provider_->Cleanup();

    maneuver_reference_client_.reset();
    maneuver_reference_client_ = nullptr;

    is_configured_ = false;

}

void MissionExecutor::Start(
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator
) {

    if (is_started_) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Start(): Already started.");
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Initializing mode provider.");

    // Modes provider
    mode_provider_ = std::make_shared<iii_drone::px4::ModeProvider>(
        tree_provider_,
        mission_specification_,
        node_,
        maneuver_reference_client_,
        configurator->GetParameterBundle("mode_provider")
    );

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Initializing mode executor.");

    generic_mode_executor_ = std::make_shared<iii_drone::px4::GenericModeExecutor>(
        *mode_provider_->GetMode(mission_specification_->executor_owned_mode()),
        "mode_executor",
        mission_specification_,
        mode_provider_,
        configurator->GetParameterBundle("mode_executor")
    );

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Registering mode executor.");

    if (!generic_mode_executor_->doRegister()) {
        RCLCPP_FATAL(node_->get_logger(), "MissionExecutor::Start(): Mode executor registration failed.");
        throw std::runtime_error("MissionExecutor::Start(): Mode executor registration failed.");
    }

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Registering modes.");

    mode_provider_->Register();

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Adding node to executor.");

    executor_.add_node(mode_provider_->mode_node());

    is_started_ = true;

}

void MissionExecutor::Stop() {

    if (!is_started_) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Stop(): Already stopped.");
        return;
    }

    executor_.remove_node(
        mode_provider_->mode_node(),
        true
    );

    generic_mode_executor_.reset();
    generic_mode_executor_ = nullptr;

    mode_provider_->Stop();
    mode_provider_->Cleanup();
    mode_provider_.reset();
    mode_provider_ = nullptr;

    is_started_ = false;

}