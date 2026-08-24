/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/trees/tree_provider.hpp>
using namespace iii_drone::behavior;
using namespace iii_drone::mission;
using namespace iii_drone::configuration;
using namespace iii_drone::control::maneuver;

namespace {

using NodeConfigurator = Configurator<rclcpp::Node>;
using ParameterType = rclcpp::ParameterType;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;

void DeclareManagedParameters(NodeConfigurator & configurator)
{
    const auto int_t = ParameterType::PARAMETER_INTEGER;
    const auto double_t = ParameterType::PARAMETER_DOUBLE;
    const auto string_t = ParameterType::PARAMETER_STRING;
    const auto bool_t = ParameterType::PARAMETER_BOOL;

    configurator.DeclareParameter("/behavior/server_timeout_ms", int_t);
    configurator.DeclareParameter("/behavior/wait_for_server_timeout_ms", int_t);
    configurator.DeclareParameter("/behavior/tick_period_ms", int_t);
    configurator.DeclareParameter("/behavior/target_cable_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/minimum_target_altitude", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_takeoff_min_target_cable_distance", double_t);
    configurator.DeclareParameter("/control/maneuver_controller/cable_takeoff_max_target_cable_distance", double_t);
    configurator.DeclareParameter("/behavior/line_min_height_above_drone", double_t);
    configurator.DeclareParameter("/behavior/select_target_line_method", string_t);
    configurator.DeclareParameter("/behavior/hover_on_cable_target_z_velocity", double_t);
    configurator.DeclareParameter("/behavior/hover_on_cable_target_yaw_rate", double_t);
    configurator.DeclareParameter("/behavior/top_clearance_m", double_t);
    configurator.DeclareParameter("/behavior/horizontal_clearance_m", double_t);
    configurator.DeclareParameter("/behavior/under_cable_clearance_m", double_t);
    configurator.DeclareParameter("/behavior/inside_powerline_xy_distance_threshold_m", double_t);
    configurator.DeclareParameter("/tf/drone_frame_id", string_t);
    configurator.DeclareParameter("/tf/cable_gripper_frame_id", string_t);
    configurator.DeclareParameter("/tf/world_frame_id", string_t);
    configurator.DeclareParameter("/mission/bypass_battery_checks", bool_t);
    configurator.DeclareParameter("/inspection_demo/inspection_clearance_m", double_t);
    configurator.DeclareParameter("/inspection_demo/pylon_end_clearance_m", double_t);
    configurator.DeclareParameter("/inspection_demo/pylon_structure_extent_m", double_t);
    configurator.DeclareParameter("/inspection_demo/max_pylon_powerline_direction_mismatch_rad", double_t);
    configurator.DeclareParameter("/inspection_demo/pylon_span_margin_m", double_t);
    configurator.DeclareParameter("/inspection_demo/battery_topic_timeout_s", double_t);
    configurator.DeclareParameter("/inspection_demo/battery_check_retry_count", int_t);
    configurator.DeclareParameter("/inspection_demo/battery_check_retry_interval_s", double_t);
    configurator.DeclareParameter("/inspection_demo/battery_voltage_threshold_v", double_t);
    configurator.DeclareParameter("/inspection_demo/battery_voltage_debounce_s", double_t);
    configurator.DeclareParameter("/cable_charging/minimum_stay_on_cable_s", double_t);

    configurator.CreateConfiguration("target_provider", {
        ConfigurationEntry("/behavior/target_cable_distance", double_t),
        ConfigurationEntry("/tf/drone_frame_id", string_t),
        ConfigurationEntry("/tf/cable_gripper_frame_id", string_t),
    });
    configurator.CreateConfiguration("cable_takeoff_maneuver_action_node", {
        ConfigurationEntry("/control/maneuver_controller/cable_takeoff_min_target_cable_distance", double_t),
        ConfigurationEntry("/control/maneuver_controller/cable_takeoff_max_target_cable_distance", double_t),
        ConfigurationEntry("/behavior/target_cable_distance", double_t),
    });
    configurator.CreateConfiguration("select_target_line_condition_node", {
        ConfigurationEntry("/behavior/line_min_height_above_drone", double_t),
        ConfigurationEntry("/behavior/select_target_line_method", string_t),
        ConfigurationEntry("/tf/drone_frame_id", string_t),
    });
    configurator.CreateConfiguration("hover_on_cable_maneuver_action_node", {
        ConfigurationEntry("/behavior/hover_on_cable_target_z_velocity", double_t),
        ConfigurationEntry("/behavior/hover_on_cable_target_yaw_rate", double_t),
    });
    configurator.CreateConfiguration("powerline_waypoint_provider_action_node", {
        ConfigurationEntry("/behavior/line_min_height_above_drone", double_t),
        ConfigurationEntry("/behavior/top_clearance_m", double_t),
        ConfigurationEntry("/behavior/horizontal_clearance_m", double_t),
        ConfigurationEntry("/behavior/inside_powerline_xy_distance_threshold_m", double_t),
        ConfigurationEntry("/behavior/under_cable_clearance_m", double_t),
        ConfigurationEntry("/control/maneuver_controller/minimum_target_altitude", double_t),
        ConfigurationEntry("/inspection_demo/pylon_span_margin_m", double_t),
        ConfigurationEntry("/tf/world_frame_id", string_t),
        ConfigurationEntry("/tf/drone_frame_id", string_t),
    });
    configurator.CreateConfiguration("phase_waypoint_provider_action_node", {
        ConfigurationEntry("/behavior/inside_powerline_xy_distance_threshold_m", double_t),
        ConfigurationEntry("/behavior/under_cable_clearance_m", double_t),
        ConfigurationEntry("/inspection_demo/inspection_clearance_m", double_t),
        ConfigurationEntry("/inspection_demo/pylon_end_clearance_m", double_t),
        ConfigurationEntry("/inspection_demo/pylon_structure_extent_m", double_t),
        ConfigurationEntry("/inspection_demo/max_pylon_powerline_direction_mismatch_rad", double_t),
        ConfigurationEntry("/inspection_demo/pylon_span_margin_m", double_t),
    });
    configurator.CreateConfiguration("battery_recharge_condition_node", {
        ConfigurationEntry("/mission/bypass_battery_checks", bool_t),
        ConfigurationEntry("/inspection_demo/battery_topic_timeout_s", double_t),
        ConfigurationEntry("/inspection_demo/battery_check_retry_count", int_t),
        ConfigurationEntry("/inspection_demo/battery_check_retry_interval_s", double_t),
        ConfigurationEntry("/inspection_demo/battery_voltage_threshold_v", double_t),
        ConfigurationEntry("/inspection_demo/battery_voltage_debounce_s", double_t),
    });
    configurator.CreateConfiguration("cable_charging_monitor_action_node", {
        ConfigurationEntry("/mission/bypass_battery_checks", bool_t),
        ConfigurationEntry("/cable_charging/minimum_stay_on_cable_s", double_t),
    });
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TreeProvider::TreeProvider(
    tf2_ros::Buffer::SharedPtr tf_buffer,
    MissionSpecification::SharedPtr mission_specification,
    std::shared_ptr<iii_drone::mission::RuntimeIntentBuffer> runtime_intent_buffer
) : rclcpp::Node(
    "behavior_tree",
    "/mission/behavior_tree",
    rclcpp::NodeOptions().use_global_arguments(false)
),  tf_buffer_(tf_buffer),
    mission_specification_(mission_specification),
    runtime_intent_buffer_(runtime_intent_buffer)
{
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

    // RCLCPP_INFO(get_logger(), "TreeProvider::TreeProvider(): Initializing.");

	const char * log_level_env = std::getenv("BEHAVIOR_TREE_LOG_LEVEL");
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



    RCLCPP_INFO(get_logger(), "TreeProvider::TreeProvider(): Initialized.");

}

Configuration::SharedPtr TreeProvider::phaseWaypointConfiguration() const {
    if (!configurator_) {
        return nullptr;
    }
    return configurator_->GetConfiguration("phase_waypoint_provider_action_node");
}

void TreeProvider::Configure(
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) {

    if (is_configured_) {
        RCLCPP_WARN(get_logger(), "TreeProvider::Configure(): Already configured.");
        return;
    }

    RCLCPP_INFO(get_logger(), "TreeProvider::Configure(): Configuring.");

    maneuver_reference_client_ = maneuver_reference_client;

    configurator_ = std::make_shared<Configurator<rclcpp::Node>>(this, "behavior_tree");
    DeclareManagedParameters(*configurator_);
    configurator_->validate();

    global_blackboard_ = BT::Blackboard::create();

    initializeTreeExecutors();

    is_configured_ = true;

}

void TreeProvider::Cleanup() {

    if (!is_configured_) {
        RCLCPP_WARN(get_logger(), "TreeProvider::Cleanup(): Not configured.");
        return;
    }

    RCLCPP_INFO(get_logger(), "TreeProvider::Cleanup(): Cleaning up.");

    for (auto it = tree_executors_.begin(); it != tree_executors_.end(); ++it) {

        it->second->Deinitialize();

    }

    tree_executors_.clear();

    global_blackboard_.reset();

    configurator_.reset();
    configurator_ = nullptr;

    maneuver_reference_client_.reset();

    is_configured_ = false;

}

void TreeProvider::SetMissionSpecification(
    MissionSpecification::SharedPtr mission_specification
) {

    if (is_configured_) {
        throw std::runtime_error(
            "TreeProvider::SetMissionSpecification(): Cannot replace mission specification while configured."
        );
    }

    if (mission_specification == nullptr) {
        throw std::runtime_error(
            "TreeProvider::SetMissionSpecification(): mission_specification must not be null."
        );
    }

    mission_specification_ = mission_specification;

}

TreeExecutor::SharedPtr TreeProvider::GetTreeExecutor(const std::string& name) const {

    auto it = tree_executors_.find(name);

    if (it == tree_executors_.end()) {

        std::string fatal_msg = "TreeProvider::GetTreeExecutor(): Tree executor not found: " + name;

        RCLCPP_FATAL(get_logger(), fatal_msg.c_str());

        throw std::runtime_error(fatal_msg);

    }

    return it->second;

}

void TreeProvider::ClearGlobalBlackboard(const std::string & reason) {
    if (!global_blackboard_) {
        RCLCPP_WARN(
            get_logger(),
            "TreeProvider::ClearGlobalBlackboard(): Global blackboard is not configured. Reason: %s",
            reason.c_str()
        );
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "TreeProvider::ClearGlobalBlackboard(): Clearing global blackboard. Reason: %s",
        reason.c_str()
    );
    const auto keys = global_blackboard_->getKeys();
    for (const auto & key : keys) {
        global_blackboard_->unset(std::string(key));
    }

    if (runtime_intent_buffer_) {
        runtime_intent_buffer_->Clear();
    }
}

void TreeProvider::initializeTreeExecutors(
) {

    RCLCPP_INFO(get_logger(), "TreeProvider::initializeTreeExecutors(): Initializing tree executors.");

    for (mission_specification_entry_t entry : *mission_specification_) {
    
        TreeExecutor::SharedPtr tree_executor = std::make_shared<TreeExecutor>(
            entry.key,
            entry.behavior_tree_xml_file,
            maneuver_reference_client_,
            tf_buffer_,
            configurator_,
            this,
            global_blackboard_,
            runtime_intent_buffer_
        );

        tree_executor->FinalizeInitialization();

        tree_executors_[entry.key] = tree_executor;

    }

    RCLCPP_INFO(get_logger(), "TreeProvider::initializeTreeExecutors(): Initialized tree executors.");

}

TreeProviderIterator TreeProvider::begin() {

    return TreeProviderIterator(tree_executors_.begin());

}

TreeProviderIterator TreeProvider::end() {

    return TreeProviderIterator(tree_executors_.end());

}

TreeProviderIterator::TreeProviderIterator(iterator it) : it_(it) {}

TreeExecutor::SharedPtr TreeProviderIterator::operator*() const {

    return it_->second;

}

TreeProviderIterator& TreeProviderIterator::operator++() {

    ++it_;

    return *this;

}

bool TreeProviderIterator::operator!=(const TreeProviderIterator& other) const {

    return it_ != other.it_;

}
