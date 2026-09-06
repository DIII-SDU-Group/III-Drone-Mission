/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_executor_node/mission_executor_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/phase_waypoint_provider_action_node.hpp>

#include <iii_drone_core/adapters/powerline_adapter.hpp>

#include <chrono>
#include <exception>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>

#include <lifecycle_msgs/msg/state.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace iii_drone::configuration;
using namespace iii_drone::mission;

namespace {

using LifecycleConfigurator = Configurator<rclcpp_lifecycle::LifecycleNode>;
using ParameterType = rclcpp::ParameterType;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;

std::string ConfigurationProfile()
{
    if (const char * profile = std::getenv("III_SYSTEM_PROFILE"); profile != nullptr && *profile != '\0') {
        return profile;
    }
    if (const char * profile = std::getenv("III_DRONE_PROFILE"); profile != nullptr && *profile != '\0') {
        return profile;
    }
    if (const char * simulation = std::getenv("SIMULATION"); simulation != nullptr) {
        return std::string(simulation) == "true" ? "sim" : "real";
    }
    return "unknown";
}

bool WaitForVehicleStatusMessage(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic_name,
    std::chrono::milliseconds timeout
) {
    auto subscription = node.create_subscription<px4_msgs::msg::VehicleStatus>(
        topic_name,
        rclcpp::QoS(1).best_effort(),
        [](px4_msgs::msg::VehicleStatus::UniquePtr) {}
    );

    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(subscription);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();
        const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
        const auto wait_result = wait_set.wait(remaining);

        if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
            px4_msgs::msg::VehicleStatus message;
            rclcpp::MessageInfo message_info;

            if (subscription->take(message, message_info)) {
                wait_set.remove_subscription(subscription);
                return true;
            }
        }
    }

    wait_set.remove_subscription(subscription);
    return false;
}

std::string Trim(const std::string & value)
{
    auto begin = value.begin();
    while (begin != value.end() && std::isspace(static_cast<unsigned char>(*begin))) {
        ++begin;
    }
    auto end = value.end();
    while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
        --end;
    }
    return std::string(begin, end);
}

void DeclareManagedParameters(LifecycleConfigurator & configurator)
{
    const auto bool_t = ParameterType::PARAMETER_BOOL;
    const auto int_t = ParameterType::PARAMETER_INTEGER;
    const auto double_t = ParameterType::PARAMETER_DOUBLE;
    const auto string_t = ParameterType::PARAMETER_STRING;

    configurator.DeclareParameter("/mission/use_nans_when_hovering", bool_t);
    configurator.DeclareParameter("/mission/max_failed_attempts_during_maneuver", int_t);
    configurator.DeclareParameter("/mission/wait_for_maneuver_start_timeout_ms", int_t);
    configurator.DeclareParameter("/control/dt", double_t);
    configurator.DeclareParameter("/mission/get_reference_timeout_ms", int_t);
    configurator.DeclareParameter("/mission/reference_loss_timeout_ms", int_t);
    configurator.DeclareParameter("/mission/reference_rebase_timeout_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/maneuver_execution_period_ms", int_t);
    configurator.DeclareParameter("/control/maneuver_controller/reference_stream_timeout_ms", int_t);
    configurator.DeclareParameter("/mission/reference_continuity_position_tolerance_m", double_t);
    configurator.DeclareParameter("/mission/reference_continuity_velocity_tolerance_m_s", double_t);
    configurator.DeclareParameter("/mission/reference_continuity_acceleration_tolerance_m_s2", double_t);
    configurator.DeclareParameter("/mission/reference_continuity_yaw_tolerance_rad", double_t);
    configurator.DeclareParameter("/mission/reference_continuity_yaw_rate_tolerance_rad_s", double_t);
    configurator.DeclareParameter("/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/controlled_cancel_settle_time_s", double_t);
    configurator.DeclareParameter("/mission/manual_stick_input_threshold", double_t);
    configurator.DeclareParameter("/mission/mission_done_select_mode", string_t);

    configurator.CreateConfiguration("maneuver_reference_client", {
        ConfigurationEntry("/mission/use_nans_when_hovering", bool_t),
        ConfigurationEntry("/mission/max_failed_attempts_during_maneuver", int_t),
        ConfigurationEntry("/mission/wait_for_maneuver_start_timeout_ms", int_t),
        ConfigurationEntry("/mission/get_reference_timeout_ms", int_t),
        ConfigurationEntry("/mission/reference_loss_timeout_ms", int_t),
        ConfigurationEntry("/mission/reference_rebase_timeout_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/maneuver_execution_period_ms", int_t),
        ConfigurationEntry("/control/maneuver_controller/reference_stream_timeout_ms", int_t),
        ConfigurationEntry("/mission/reference_continuity_position_tolerance_m", double_t),
        ConfigurationEntry("/mission/reference_continuity_velocity_tolerance_m_s", double_t),
        ConfigurationEntry("/mission/reference_continuity_acceleration_tolerance_m_s2", double_t),
        ConfigurationEntry("/mission/reference_continuity_yaw_tolerance_rad", double_t),
        ConfigurationEntry("/mission/reference_continuity_yaw_rate_tolerance_rad_s", double_t),
        ConfigurationEntry("/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2", double_t),
        ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", double_t),
        ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", double_t),
        ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", double_t),
        ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", double_t),
        ConfigurationEntry("/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", double_t),
        ConfigurationEntry("/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", double_t),
        ConfigurationEntry("/control/maneuver_controller/controlled_cancel_settle_time_s", double_t),
    });
    configurator.CreateConfiguration("mode_provider", {
        ConfigurationEntry("/control/dt", double_t),
    });
    configurator.CreateConfiguration("mode_executor", {
        ConfigurationEntry("/mission/manual_stick_input_threshold", double_t),
        ConfigurationEntry("/mission/mission_done_select_mode", string_t),
    });
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MissionExecutorNode::MissionExecutorNode(
    rclcpp::executors::MultiThreadedExecutor & executor_handle,
    std::string node_name,
    std::string node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(
    node_name, 
    node_namespace, 
    options
),  executor_handle_(executor_handle) {
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	const char * log_level_env = std::getenv("MISSION_EXECUTOR_LOG_LEVEL");
	std::string log_level = log_level_env == nullptr ? "" : log_level_env;

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			set_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			set_logger_level(RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			set_logger_level(RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			set_logger_level(RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			set_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
		}

	}

    get_mission_catalog_service_ = create_service<iii_drone_interfaces::srv::GetMissionCatalog>(
        "get_mission_catalog",
        std::bind(&MissionExecutorNode::getMissionCatalogService, this, std::placeholders::_1, std::placeholders::_2)
    );
    select_mission_catalog_entry_service_ = create_service<iii_drone_interfaces::srv::SelectMissionCatalogEntry>(
        "select_mission_catalog_entry",
        std::bind(&MissionExecutorNode::selectMissionCatalogEntryService, this, std::placeholders::_1, std::placeholders::_2)
    );
    mission_status_publisher_ = create_publisher<iii_drone_interfaces::msg::MissionModeStatus>(
        "/mission/status",
        rclcpp::QoS(1).reliable().transient_local()
    );
    mission_status_publisher_->on_activate();
    powerline_overview_client_ = create_client<iii_drone_interfaces::srv::GetPowerlineOverview>(
        "/mission/powerline_overview_provider/get_powerline_overview"
    );
    pylon_overview_client_ = create_client<iii_drone_interfaces::srv::GetPylonOverview>(
        "/mission/pylon_overview_provider/get_pylon_overview"
    );
    mission_status_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            publishMissionModeStatus();
        }
    );

    odometry_sub_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    get_reference_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::MissionExecutorNode()");

}

MissionExecutorNode::~MissionExecutorNode() {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::~MissionExecutorNode()");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_configure(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_configure()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_configure(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_configure(): Failed to configure parent class"
        );
        return ret;
    }

    // Configurator
    RCLCPP_DEBUG(
        get_logger(), 
        "MissionExecutorNode::on_configure(): Initializing configurator object"
    );

    configurator_ = std::make_shared<Configurator<rclcpp_lifecycle::LifecycleNode>>(
        this,
        "mission_executor"
    );
    DeclareManagedParameters(*configurator_);
    configurator_->validate();
    active_profile_ = ConfigurationProfile();

    try {
        mission_catalog_ = MissionCatalog::LoadInstalled();
        const auto & default_entry = mission_catalog_->defaultEntry(active_profile_);
        default_catalog_id_ = default_entry.id;
        active_catalog_id_ = default_entry.id;
        temporary_override_ = false;
    } catch (const std::exception & exception) {
        mission_status_degraded_reason_ = exception.what();
        RCLCPP_ERROR(get_logger(), "Mission catalog initialization failed: %s", exception.what());
        publishMissionModeStatus();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // TF Buffer
    if (tf_buffer_ == nullptr) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // Mission Executor
    auto mission_specification = std::make_shared<MissionSpecification>(
        mission_catalog_,
        mission_catalog_->entryForProfile(active_catalog_id_, active_profile_),
        this
    );
    mission_executor_ = std::make_shared<MissionExecutor>(
        this, 
        tf_buffer_,
        mission_specification,
        odometry_sub_callback_group_,
        executor_handle_
    );

    mission_executor_->Configure(
        configurator_,
        get_reference_cb_group_
    );

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_configure(): Configured");
    mission_status_degraded_reason_.clear();
    publishMissionModeStatus();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_cleanup(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_cleanup()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_cleanup(): Failed to cleanup parent class"
        );
        return ret;
    }

    cleanup();
    publishMissionModeStatus();

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_cleanup(): Cleaned up");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_activate(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_activate()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_activate(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_activate(): Failed to activate parent class"
        );
        return ret;
    }

    // Mission Executor
    RCLCPP_INFO(
        get_logger(), 
        "MissionExecutorNode::on_activate(): Starting mission executor"
    );

    if (!WaitForVehicleStatusMessage(*this, "/fmu/out/vehicle_status_v1", std::chrono::seconds(5))) {
        RCLCPP_ERROR(
            get_logger(),
            "MissionExecutorNode::on_activate(): Cannot start mission executor because "
            "/fmu/out/vehicle_status_v1 did not publish a fresh message. Start the PX4 ROS bridge "
            "and verify the FMU is publishing before activating mission execution."
        );
        mission_status_degraded_reason_ = "PX4 vehicle status topic /fmu/out/vehicle_status_v1 is stale or unavailable";
        publishMissionModeStatus();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    try {
        mission_executor_->Start(configurator_);
    } catch (const std::exception & exc) {
        RCLCPP_ERROR(
            get_logger(),
            "MissionExecutorNode::on_activate(): Failed to start mission executor: %s",
            exc.what()
        );
        mission_status_degraded_reason_ = exc.what();
        cleanup();
        publishMissionModeStatus();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    } catch (...) {
        RCLCPP_ERROR(
            get_logger(),
            "MissionExecutorNode::on_activate(): Failed to start mission executor: unknown exception"
        );
        mission_status_degraded_reason_ = "unknown mission executor activation failure";
        cleanup();
        publishMissionModeStatus();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
        get_logger(), 
        "MissionExecutorNode::on_activate(): Activated"
    );
    mission_status_degraded_reason_.clear();
    publishMissionModeStatus();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_deactivate(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_deactivate()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_deactivate(): Failed to deactivate parent class"
        );
        return ret;
    }

    // Mission Executor
    RCLCPP_DEBUG(
        get_logger(), 
        "MissionExecutorNode::on_deactivate(): Deactivating mission executor"
    );
    mission_executor_->Stop();
    publishMissionModeStatus();

    RCLCPP_INFO(
        get_logger(), 
        "MissionExecutorNode::on_deactivate(): Deactivated"
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_shutdown(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_shutdown()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_shutdown(): Failed to shutdown parent class"
        );
        return ret;
    }

    cleanup();
    publishMissionModeStatus();

    // Create and start thread detached which sleeps for 1 second, then shuts down rclcpp
    std::thread shutdown_thread([this](){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    });
    shutdown_thread.detach();

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_shutdown(): Shutdown completed");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_error(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_FATAL(get_logger(), "MissionExecutorNode::on_error(): Lifecycle transition failed.");
    cleanup();
    publishMissionModeStatus();

    return rclcpp_lifecycle::LifecycleNode::on_error(state);

}

void MissionExecutorNode::getMissionCatalogService(
    const std::shared_ptr<iii_drone_interfaces::srv::GetMissionCatalog::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::GetMissionCatalog::Response> response
)
{
    if (mission_catalog_ == nullptr) {
        response->success = false;
        response->message = "installed mission catalog is not initialized";
        return;
    }
    try {
        response->catalog_json = mission_catalog_->catalogJson(
            active_profile_,
            request->include_incompatible
        );
        response->success = true;
        response->message = "installed mission catalog returned";
    } catch (const std::exception & exception) {
        response->success = false;
        response->message = exception.what();
    }
}

void MissionExecutorNode::selectMissionCatalogEntryService(
    const std::shared_ptr<iii_drone_interfaces::srv::SelectMissionCatalogEntry::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::SelectMissionCatalogEntry::Response> response
)
{
    auto publish_active_identity = [this, &response]() {
        response->active_catalog_id = active_catalog_id_;
        response->active_catalog_hash = mission_catalog_ != nullptr ? mission_catalog_->catalogHash() : "";
        response->temporary_override = temporary_override_;
        if (mission_executor_ != nullptr && mission_executor_->mission_specification() != nullptr) {
            const auto specification = mission_executor_->mission_specification();
            response->active_entry_hash = specification->entry_hash();
            response->active_specification_asset_id = specification->specification_asset_id();
            response->active_behavior_tree_asset_ids = specification->behavior_tree_asset_ids();
        }
    };

    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        response->success = false;
        response->message = "mission catalog selection rejected because the mission executor is not active";
        publish_active_identity();
        return;
    }

    if (mission_executor_ == nullptr || configurator_ == nullptr || mission_catalog_ == nullptr) {
        response->success = false;
        response->message = "mission catalog selection rejected because the mission runtime is not initialized";
        publish_active_identity();
        return;
    }

    const std::string requested_id = request->use_default ? default_catalog_id_ : Trim(request->catalog_id);
    if (
        (!request->use_default && requested_id.empty()) ||
        (request->use_default && !Trim(request->catalog_id).empty()) ||
        requested_id.find_first_of("/\\~$") != std::string::npos
    ) {
        response->success = false;
        response->message = "mission selection accepts exactly one catalog ID or use_default; filesystem paths are forbidden";
        publish_active_identity();
        return;
    }

    MissionSpecification::SharedPtr replacement;
    const MissionCatalogEntry * selected_entry = nullptr;
    try {
        selected_entry = &mission_catalog_->entryForProfile(requested_id, active_profile_);
        replacement = std::make_shared<MissionSpecification>(
            mission_catalog_,
            *selected_entry,
            this
        );
    } catch (const std::exception & exception) {
        response->success = false;
        response->message = exception.what();
        publish_active_identity();
        return;
    }

    std::string message;
    const bool success = mission_executor_->SelectMissionSpecification(
        replacement,
        configurator_,
        get_reference_cb_group_,
        message
    );

    response->success = success;
    response->message = message;
    if (success) {
        active_catalog_id_ = requested_id;
        temporary_override_ = requested_id != default_catalog_id_;
        mission_status_degraded_reason_.clear();
        if (selected_entry->experimental()) {
            response->warning = selected_entry->experimental_warning;
            RCLCPP_WARN(
                get_logger(),
                "EXPERIMENTAL mission catalog entry selected: %s: %s",
                selected_entry->id.c_str(),
                selected_entry->experimental_warning.c_str()
            );
        }
    }
    publish_active_identity();
    publishMissionModeStatus();

    if (success) {
        RCLCPP_INFO(
            get_logger(),
            "MissionExecutorNode::selectMissionCatalogEntryService(): %s",
            response->message.c_str()
        );
    } else {
        RCLCPP_WARN(
            get_logger(),
            "MissionExecutorNode::selectMissionCatalogEntryService(): %s",
            response->message.c_str()
        );
    }

}

void MissionExecutorNode::cleanup() {

    RCLCPP_DEBUG(get_logger(), "MissionExecutorNode::cleanup()");

    // Mission Executor
    if (mission_executor_ != nullptr) {
        RCLCPP_INFO(get_logger(), "MissionExecutorNode::cleanup(): Cleaning up mission executor.");
        mission_executor_->Stop();
        mission_executor_->Cleanup();
        mission_executor_.reset();
        mission_executor_ = nullptr;
    }

    // Configurator
    if (configurator_ != nullptr) {
        RCLCPP_INFO(get_logger(), "MissionExecutorNode::cleanup(): Cleaning up configurator.");
        configurator_.reset();
        configurator_ = nullptr;
    }

    RCLCPP_DEBUG(get_logger(), "MissionExecutorNode::cleanup(): Cleaned up.");

}

std::vector<std::string> MissionExecutorNode::requiredMissionModes() const {

    if (mission_executor_ == nullptr || mission_executor_->mission_specification() == nullptr) {
        return {};
    }
    return mission_executor_->mission_specification()->mode_keys();

}

std::vector<std::string> MissionExecutorNode::registeredMissionModes() const {

    if (mission_executor_ == nullptr || mission_executor_->mode_provider() == nullptr) {
        return {};
    }
    return mission_executor_->mode_provider()->registered_mode_keys();

}

bool MissionExecutorNode::requiredMissionModesRegistered() const {

    const auto required_modes = requiredMissionModes();
    const auto registered_modes = registeredMissionModes();
    if (required_modes.empty()) {
        return false;
    }
    for (const auto & mode : required_modes) {
        if (std::find(registered_modes.begin(), registered_modes.end(), mode) == registered_modes.end()) {
            return false;
        }
    }
    return true;

}

void MissionExecutorNode::publishMissionModeStatus() {

    if (!mission_status_publisher_) {
        return;
    }

    iii_drone_interfaces::msg::MissionModeStatus msg;
    msg.stamp = get_clock()->now();
    refreshInspectionOverviewCaches();
    populateInspectionStartEligibility(msg);
    msg.active_catalog_id = active_catalog_id_;
    msg.catalog_hash = mission_catalog_ != nullptr ? mission_catalog_->catalogHash() : "";
    msg.default_catalog_id = default_catalog_id_;
    msg.configuration_profile = active_profile_.empty() ? ConfigurationProfile() : active_profile_;
    msg.temporary_override = temporary_override_;
    msg.catalog_ready = mission_catalog_ != nullptr;
    msg.catalog_error = mission_status_degraded_reason_;
    msg.required_modes = requiredMissionModes();
    msg.registered_modes = registeredMissionModes();
    msg.required_modes_registered = requiredMissionModesRegistered();

    if (mission_executor_ != nullptr && mission_executor_->mission_specification() != nullptr) {
        const auto specification = mission_executor_->mission_specification();
        msg.active_catalog_id = specification->catalog_id();
        msg.active_entry_hash = specification->entry_hash();
        msg.active_specification_asset_id = specification->specification_asset_id();
        msg.active_behavior_tree_asset_ids = specification->behavior_tree_asset_ids();
        msg.classification = specification->classification();
        msg.compatible_profiles = specification->compatible_profiles();
        msg.experimental = specification->classification() == "experimental";
        msg.experimental_warning = specification->experimental_warning();
        msg.owned_mode = specification->executor_owned_mode();
        msg.intents = mission_executor_->intentStatuses();
        const auto mode_provider = mission_executor_->mode_provider();
        if (mode_provider != nullptr) {
            for (const auto & mode : *mode_provider) {
                iii_drone_interfaces::msg::MissionModeRegistryEntry entry;
                entry.stamp = msg.stamp;
                entry.mode_key = mode->mode_key();
                entry.display_name = mode->mode_name();
                entry.mode_id = mode->mode_id();
                entry.mode_id_valid = mode->is_registered();
                entry.registered = mode->is_registered();
                entry.active = mode->active();
                entry.tree_running = mode->tree_running();
                entry.tree_finished = mode->tree_finished();
                entry.tree_success = mode->tree_success();
                entry.tree_success_valid = entry.tree_finished;
                entry.degraded_reason = mode->degraded_reason();
                entry.degraded = !entry.degraded_reason.empty();
                msg.modes.push_back(entry);
            }
        }
    }

    msg.mission_active = mission_executor_ != nullptr && mission_executor_->mission_active();
    msg.degraded_reason = mission_status_degraded_reason_;
    msg.degraded = !mission_status_degraded_reason_.empty() ||
        (msg.mission_active && !msg.required_modes_registered);
    if (!mission_status_degraded_reason_.empty()) {
        msg.degraded_reasons.push_back(mission_status_degraded_reason_);
    }
    if (msg.mission_active && !msg.required_modes_registered) {
        msg.degraded_reasons.push_back("not all mission modes are registered with PX4");
    }
    msg.ready = mission_executor_ != nullptr && !msg.degraded;

    if (msg.degraded) {
        msg.mission_state = iii_drone_interfaces::msg::MissionModeStatus::MISSION_STATE_DEGRADED;
        msg.mission_state_label = "degraded";
    } else if (msg.mission_active) {
        msg.mission_state = iii_drone_interfaces::msg::MissionModeStatus::MISSION_STATE_ACTIVE;
        msg.mission_state_label = "active";
    } else if (mission_executor_ != nullptr) {
        msg.mission_state = iii_drone_interfaces::msg::MissionModeStatus::MISSION_STATE_READY;
        msg.mission_state_label = "ready";
    } else {
        msg.mission_state = iii_drone_interfaces::msg::MissionModeStatus::MISSION_STATE_IDLE;
        msg.mission_state_label = "idle";
    }

    mission_status_publisher_->publish(msg);

}

void MissionExecutorNode::refreshInspectionOverviewCaches() {
    if (
        powerline_overview_client_ &&
        powerline_overview_client_->service_is_ready() &&
        !powerline_overview_request_pending_.exchange(true)
    ) {
        auto request = std::make_shared<iii_drone_interfaces::srv::GetPowerlineOverview::Request>();
        powerline_overview_client_->async_send_request(
            request,
            [this](rclcpp::Client<iii_drone_interfaces::srv::GetPowerlineOverview>::SharedFuture future) {
                try {
                    std::lock_guard<std::mutex> lock(inspection_overview_mutex_);
                    powerline_overview_response_ = future.get();
                } catch (const std::exception & exc) {
                    RCLCPP_WARN(get_logger(), "Failed to refresh powerline overview: %s", exc.what());
                }
                powerline_overview_request_pending_ = false;
            }
        );
    }
    if (
        pylon_overview_client_ &&
        pylon_overview_client_->service_is_ready() &&
        !pylon_overview_request_pending_.exchange(true)
    ) {
        auto request = std::make_shared<iii_drone_interfaces::srv::GetPylonOverview::Request>();
        pylon_overview_client_->async_send_request(
            request,
            [this](rclcpp::Client<iii_drone_interfaces::srv::GetPylonOverview>::SharedFuture future) {
                try {
                    std::lock_guard<std::mutex> lock(inspection_overview_mutex_);
                    pylon_overview_response_ = future.get();
                } catch (const std::exception & exc) {
                    RCLCPP_WARN(get_logger(), "Failed to refresh pylon overview: %s", exc.what());
                }
                pylon_overview_request_pending_ = false;
            }
        );
    }
}

void MissionExecutorNode::populateInspectionStartEligibility(
    iii_drone_interfaces::msg::MissionModeStatus & msg
) {
    auto & output = msg.inspection_start_eligibility;
    output.stamp = msg.stamp;
    if (mission_executor_ == nullptr) {
        output.failure_reasons.push_back("mission executor is unavailable");
        return;
    }
    const auto position = mission_executor_->currentPosition();
    if (!position) {
        output.failure_reasons.push_back("vehicle odometry has not been received");
        return;
    }

    iii_drone_interfaces::srv::GetPowerlineOverview::Response::SharedPtr powerline;
    iii_drone_interfaces::srv::GetPylonOverview::Response::SharedPtr pylons;
    {
        std::lock_guard<std::mutex> lock(inspection_overview_mutex_);
        powerline = powerline_overview_response_;
        pylons = pylon_overview_response_;
    }
    if (!powerline || !powerline->success) {
        output.failure_reasons.push_back("stored powerline overview is unavailable");
        return;
    }
    if (!pylons || !pylons->success || !pylons->valid || pylons->stored_pylon_overview.pylons.size() != 2) {
        output.failure_reasons.push_back("exactly two valid stored pylons are required");
        return;
    }
    const auto configuration = mission_executor_->phaseWaypointConfiguration();
    if (!configuration) {
        output.failure_reasons.push_back("inspection geometry configuration is unavailable");
        return;
    }

    try {
        iii_drone::adapters::PowerlineAdapter powerline_adapter(powerline->stored_powerline);
        iii_drone::types::point_t pylon_a;
        pylon_a << pylons->stored_pylon_overview.pylons.at(0).x,
            pylons->stored_pylon_overview.pylons.at(0).y, 0.0;
        iii_drone::types::point_t pylon_b;
        pylon_b << pylons->stored_pylon_overview.pylons.at(1).x,
            pylons->stored_pylon_overview.pylons.at(1).y, 0.0;
        const auto eligibility = iii_drone::behavior::EvaluateCorridorInspectionStart(
            powerline_adapter.GetPoints(),
            powerline_adapter.projection_plane().normal,
            pylon_a,
            pylon_b,
            *position,
            configuration->GetParameter("/inspection_demo/inspection_clearance_m").as_double(),
            configuration->GetParameter("/inspection_demo/pylon_end_clearance_m").as_double(),
            configuration->GetParameter("/inspection_demo/pylon_structure_extent_m").as_double(),
            configuration->GetParameter("/inspection_demo/pylon_span_margin_m").as_double(),
            configuration->GetParameter(
                "/inspection_demo/max_pylon_powerline_direction_mismatch_rad"
            ).as_double()
        );
        output.evaluable = eligibility.evaluable;
        output.eligible = eligibility.eligible;
        output.side = eligibility.side;
        output.measured_lateral_clearance_m = eligibility.measured_lateral_clearance_m;
        output.required_lateral_clearance_m = eligibility.required_lateral_clearance_m;
        output.between_pylons = eligibility.between_pylons;
        output.distance_from_start_boundary_m = eligibility.distance_from_start_boundary_m;
        output.distance_to_end_boundary_m = eligibility.distance_to_end_boundary_m;
        output.pylon_span_margin_m = eligibility.pylon_span_margin_m;
        output.ingress_point_valid = eligibility.ingress_point_valid;
        output.ingress_point.x = eligibility.ingress_point[0];
        output.ingress_point.y = eligibility.ingress_point[1];
        output.ingress_point.z = eligibility.ingress_point[2];
        output.failure_reasons = eligibility.failure_reasons;
    } catch (const std::exception & exc) {
        output.failure_reasons.push_back(
            std::string("inspection eligibility evaluation failed: ") + exc.what()
        );
    }
}

int main(int argc, char **argv) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<MissionExecutorNode>(
        executor
    );

    executor.add_node(node->get_node_base_interface());

   try {
        
        executor.spin();
        node.reset();

    } catch(const std::exception& e) {
        RCLCPP_FATAL(node->get_logger(), "MissionExecutorNode main loop failed: %s", e.what());
        node.reset();

    }
    
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

    return 0;

}
