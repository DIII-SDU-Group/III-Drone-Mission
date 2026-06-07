/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_executor_node/mission_executor_node.hpp>

#include <chrono>
#include <exception>
#include <algorithm>
#include <cctype>
#include <cstdlib>

#include <lifecycle_msgs/msg/state.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace iii_drone::configuration;
using namespace iii_drone::mission;

namespace {

using LifecycleConfigurator = Configurator<rclcpp_lifecycle::LifecycleNode>;
using ParameterType = rclcpp::ParameterType;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;

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

bool LooksLikeExplicitPath(const std::string & value)
{
    return !value.empty() &&
        (value[0] == '/' || value[0] == '~' || value[0] == '$');
}

std::string ResolveMissionSpecificationRequest(
    const std::string & requested,
    const std::string & default_mission_specification_file,
    bool use_default
) {
    if (use_default) {
        return default_mission_specification_file;
    }

    const std::string trimmed = Trim(requested);
    if (trimmed.empty()) {
        throw std::runtime_error(
            "mission_specification_file must be set unless use_default is true"
        );
    }
    if (LooksLikeExplicitPath(trimmed)) {
        return trimmed;
    }

    if (const char * mission_specification_dir = std::getenv("MISSION_SPECIFICATION_DIR");
        mission_specification_dir != nullptr && std::string(mission_specification_dir) != "") {
        std::string base_dir = mission_specification_dir;
        if (!base_dir.empty() && base_dir.back() == '/') {
            return base_dir + trimmed;
        }
        return base_dir + "/" + trimmed;
    }

    return trimmed;
}

void DeclareManagedParameters(LifecycleConfigurator & configurator)
{
    const auto bool_t = ParameterType::PARAMETER_BOOL;
    const auto int_t = ParameterType::PARAMETER_INTEGER;
    const auto double_t = ParameterType::PARAMETER_DOUBLE;
    const auto string_t = ParameterType::PARAMETER_STRING;

    configurator.DeclareParameter("/mission/mission_specification_file", string_t);
    configurator.DeclareParameter("/mission/use_nans_when_hovering", bool_t);
    configurator.DeclareParameter("/mission/max_failed_attempts_during_maneuver", int_t);
    configurator.DeclareParameter("/mission/wait_for_maneuver_start_timeout_ms", int_t);
    configurator.DeclareParameter("/control/dt", double_t);
    configurator.DeclareParameter("/mission/get_reference_timeout_ms", int_t);
    configurator.DeclareParameter("/mission/manual_stick_input_threshold", double_t);
    configurator.DeclareParameter("/mission/mission_done_select_mode", string_t);

    configurator.CreateConfiguration("maneuver_reference_client", {
        ConfigurationEntry("/mission/use_nans_when_hovering", bool_t),
        ConfigurationEntry("/mission/max_failed_attempts_during_maneuver", int_t),
        ConfigurationEntry("/mission/wait_for_maneuver_start_timeout_ms", int_t),
        ConfigurationEntry("/mission/get_reference_timeout_ms", int_t),
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

    write_behavior_tree_model_xml_service_ = create_service<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML>(
        "write_behavior_tree_model_xml",
        std::bind(&MissionExecutorNode::writeBehaviorTreeModelXmlService, this, std::placeholders::_1, std::placeholders::_2)
    );
    override_mission_specification_service_ = create_service<iii_drone_interfaces::srv::OverrideMissionSpecification>(
        "override_mission_specification",
        std::bind(&MissionExecutorNode::overrideMissionSpecificationService, this, std::placeholders::_1, std::placeholders::_2)
    );
    mission_status_publisher_ = create_publisher<iii_drone_interfaces::msg::MissionModeStatus>(
        "/mission/status",
        rclcpp::SystemDefaultsQoS()
    );
    mission_status_publisher_->on_activate();
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
    default_mission_specification_file_ = configurator_->GetParameter("/mission/mission_specification_file").as_string();
    mission_specification_file_ = default_mission_specification_file_;

    // TF Buffer
    if (tf_buffer_ == nullptr) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // Mission Executor
    mission_executor_ = std::make_shared<MissionExecutor>(
        this, 
        tf_buffer_,
        configurator_->GetParameter("/mission/mission_specification_file").as_string(),
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

void MissionExecutorNode::writeBehaviorTreeModelXmlService(
    const std::shared_ptr<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML::Response> response
) {
    (void)response;

    const BT::BehaviorTreeFactory & factory = mission_executor_->factory();

    std::string models_xml = BT::writeTreeNodesModelXML(factory);

    std::string destination = request->destination_file;

    std::ofstream file(destination);

    file << models_xml;

    file.close();

}

void MissionExecutorNode::overrideMissionSpecificationService(
    const std::shared_ptr<iii_drone_interfaces::srv::OverrideMissionSpecification::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::OverrideMissionSpecification::Response> response
) {

    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        response->success = false;
        response->message = "mission specification override rejected because mission executor lifecycle node is not active";
        response->active_mission_specification_file = mission_specification_file_;
        return;
    }

    if (mission_executor_ == nullptr || configurator_ == nullptr) {
        response->success = false;
        response->message = "mission specification override rejected because mission executor is not initialized";
        response->active_mission_specification_file = mission_specification_file_;
        return;
    }

    std::string requested_specification_file;
    try {
        requested_specification_file = ResolveMissionSpecificationRequest(
            request->mission_specification_file,
            default_mission_specification_file_,
            request->use_default
        );
    } catch (const std::exception & exception) {
        response->success = false;
        response->message = exception.what();
        response->active_mission_specification_file = mission_specification_file_;
        return;
    }

    std::string message;
    const bool success = mission_executor_->OverrideMissionSpecification(
        requested_specification_file,
        configurator_,
        get_reference_cb_group_,
        message
    );

    response->success = success;
    response->message = message;
    if (success) {
        mission_specification_file_ = mission_executor_->mission_specification()->mission_specification_file();
        mission_status_degraded_reason_.clear();
    }
    if (mission_executor_ != nullptr && mission_executor_->mission_specification() != nullptr) {
        response->active_mission_specification_file =
            mission_executor_->mission_specification()->mission_specification_file();
    } else {
        response->active_mission_specification_file = mission_specification_file_;
    }
    publishMissionModeStatus();

    if (success) {
        RCLCPP_INFO(
            get_logger(),
            "MissionExecutorNode::overrideMissionSpecificationService(): %s",
            response->message.c_str()
        );
    } else {
        RCLCPP_WARN(
            get_logger(),
            "MissionExecutorNode::overrideMissionSpecificationService(): %s",
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
    msg.active_mission_specification = mission_specification_file_;
    msg.required_modes = requiredMissionModes();
    msg.registered_modes = registeredMissionModes();
    msg.required_modes_registered = requiredMissionModesRegistered();

    if (mission_executor_ != nullptr && mission_executor_->mission_specification() != nullptr) {
        msg.owned_mode = mission_executor_->mission_specification()->executor_owned_mode();
        if (msg.active_mission_specification.empty()) {
            msg.active_mission_specification = mission_executor_->mission_specification()->mission_specification_file();
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
