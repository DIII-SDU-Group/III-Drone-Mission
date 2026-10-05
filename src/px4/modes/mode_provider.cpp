/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/modes/mode_provider.hpp>
#include <iii_drone_core/diagnostics/hil_trace.hpp>

#include <chrono>

using namespace iii_drone::px4;
using namespace iii_drone::behavior;
using namespace iii_drone::mission;
using namespace iii_drone::configuration;
using namespace iii_drone::control::maneuver;


/*****************************************************************************/
// Implementation
/*****************************************************************************/

ModeProvider::ModeProvider(
    iii_drone::behavior::TreeProvider::SharedPtr tree_provider,
    iii_drone::mission::MissionSpecification::SharedPtr mission_specification,
    rclcpp_lifecycle::LifecycleNode * node,
    iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    iii_drone::configuration::Configuration::SharedPtr parameters,
    uint64_t lifecycle_activation_generation
) : tree_provider_(tree_provider),
    mission_specification_(mission_specification),
    node_(node),
    lifecycle_activation_generation_(lifecycle_activation_generation)
{

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initializing.");

    mode_node_ = std::make_shared<rclcpp::Node>(
        "px4_mode",
        rclcpp::NodeOptions().use_global_arguments(false)
    );

    initializeDiagnosticProbes();
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(mode_node_->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(node_->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	const char * log_level_env = std::getenv("PX4_MODE_LOG_LEVEL");
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

    maneuver_reference_client_ = maneuver_reference_client;
    configuration_ = parameters;

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initializing modes.");

    initializeModes();

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initialized.");

}

void ModeProvider::Register() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Registering modes.");

    for (auto it = modes_.begin(); it != modes_.end(); ++it) {

        auto mode = it->second;

        mode->Register(
            tree_provider_->GetTreeExecutor(it->first),
            maneuver_reference_client_
        );

    }

}

void ModeProvider::Cleanup() {

    deinitializeDiagnosticProbes();
    deinitializeModes();
    maneuver_reference_client_.reset();
    configuration_.reset();

}

void ModeProvider::initializeDiagnosticProbes() {
    using namespace std::chrono_literals;

    // The probes only record trace events: without a trace file they would
    // wake the executor four times a second for nothing.
    if (!iii_drone::diagnostics::HilTrace::enabled()) {
        return;
    }

    diagnostic_independent_callback_group_ = mode_node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    const auto record_probe = [](const char * event_name, const char * callback_group) {
        auto event = iii_drone::diagnostics::HilTrace::event(event_name);
        event.text("callback", "mode_provider_diagnostic_probe");
        event.text("callback_group", callback_group);
        event.text("callback_group_type", "MutuallyExclusive");
        event.text("node", "/px4_mode");
        event.commit();
    };

    diagnostic_default_probe_timer_ = mode_node_->create_wall_timer(
        500ms,
        [record_probe]() {
            const auto start = std::chrono::steady_clock::now();
            record_probe(
                "callback_group_probe_default_entry",
                "px4_mode_default_mutually_exclusive"
            );
            const auto end = std::chrono::steady_clock::now();
            auto event = iii_drone::diagnostics::HilTrace::event("callback_group_probe_default_exit");
            event.text("callback", "mode_provider_diagnostic_probe");
            event.text("callback_group", "px4_mode_default_mutually_exclusive");
            event.text("callback_group_type", "MutuallyExclusive");
            event.text("node", "/px4_mode");
            event.number(
                "duration_ns",
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count())
            );
            event.commit();
        }
    );

    diagnostic_independent_probe_timer_ = mode_node_->create_wall_timer(
        500ms,
        [record_probe]() {
            const auto start = std::chrono::steady_clock::now();
            record_probe(
                "callback_group_probe_independent_entry",
                "px4_mode_diagnostic_independent_mutually_exclusive"
            );
            const auto end = std::chrono::steady_clock::now();
            auto event = iii_drone::diagnostics::HilTrace::event("callback_group_probe_independent_exit");
            event.text("callback", "mode_provider_diagnostic_probe");
            event.text("callback_group", "px4_mode_diagnostic_independent_mutually_exclusive");
            event.text("callback_group_type", "MutuallyExclusive");
            event.text("node", "/px4_mode");
            event.number(
                "duration_ns",
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count())
            );
            event.commit();
        },
        diagnostic_independent_callback_group_
    );
}

void ModeProvider::deinitializeDiagnosticProbes() {
    diagnostic_default_probe_timer_.reset();
    diagnostic_independent_probe_timer_.reset();
    diagnostic_independent_callback_group_.reset();
}

void ModeProvider::Stop() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::Stop(): Stopping.");

    for (auto it = modes_.begin(); it != modes_.end(); ++it) {

        auto mode = it->second;

        mode->Unregister(true);

    }

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::Stop(): Stopped.");

}

void ModeProvider::ClearGlobalBlackboard(const std::string & reason) {
    if (!tree_provider_) {
        RCLCPP_WARN(
            node_->get_logger(),
            "ModeProvider::ClearGlobalBlackboard(): Tree provider is not available. Reason: %s",
            reason.c_str()
        );
        return;
    }

    tree_provider_->ClearGlobalBlackboard(reason);
}

void ModeProvider::BeginModeActivation(const std::string & mode_key) {
    if (tree_provider_) {
        tree_provider_->BeginModeActivation(mode_key);
    }
}

ManeuverMode::SharedPtr ModeProvider::GetMode(const std::string& name) const {

    auto it = modes_.find(name);

    if (it == modes_.end()) {

        std::string fatal_msg = "ModeProvider::GetMode(): Mode not found: " + name;

        RCLCPP_FATAL(node_->get_logger(), fatal_msg.c_str());

        throw std::runtime_error(fatal_msg);

    }

    return it->second;

}

void ModeProvider::initializeModes() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Getting dt");

    float dt = configuration_->GetParameter("/control/dt").as_double();

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initializing modes.");

    for (mission_specification_entry_t entry : *mission_specification_) {

        bool is_owned_mode = mission_specification_->executor_owned_mode() == entry.key;

        RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initializing mode %s, owned mode: %d", entry.mode_name.c_str(), is_owned_mode);
    
        ManeuverMode::SharedPtr mode = std::make_shared<ManeuverMode>(
            *mode_node_,
            entry.key,
            entry.mode_name,
            dt,
            is_owned_mode,
            entry.allow_activate_when_disarmed,
            lifecycle_activation_generation_
        );

        modes_[entry.key] = mode;

    }

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initialized modes.");

}

void ModeProvider::deinitializeModes() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::deinitializeModes(): Deinitializing modes.");

    for (auto it = modes_.begin(); it != modes_.end(); ++it) {

        RCLCPP_DEBUG(node_->get_logger(), "ModeProvider::deinitializeModes(): Deinitializing mode %s.", it->first.c_str());

        it->second.reset();

    }

    RCLCPP_DEBUG(node_->get_logger(), "ModeProvider::deinitializeModes(): Clearing modes.");

    modes_.clear();

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::deinitializeModes(): Deinitialized modes.");

}

ModeProviderIterator ModeProvider::begin() {

    return ModeProviderIterator(modes_.begin());

}

ModeProviderIterator ModeProvider::end() {

    return ModeProviderIterator(modes_.end());

}

rclcpp::Node::SharedPtr ModeProvider::mode_node() const {

    return mode_node_;

}

ManeuverReferenceClient::SharedPtr ModeProvider::maneuver_reference_client() const {

    return maneuver_reference_client_;

}

std::vector<std::string> ModeProvider::mode_keys() const {

    std::vector<std::string> values;
    values.reserve(modes_.size());
    for (const auto & item : modes_) {
        values.push_back(item.first);
    }
    return values;

}

std::vector<std::string> ModeProvider::registered_mode_keys() const {

    std::vector<std::string> values;
    for (const auto & item : modes_) {
        if (item.second->is_registered()) {
            values.push_back(item.first);
        }
    }
    return values;

}

bool ModeProvider::all_modes_registered() const {

    if (modes_.empty()) {
        return false;
    }
    for (const auto & item : modes_) {
        if (!item.second->is_registered()) {
            return false;
        }
    }
    return true;

}

ModeProviderIterator::ModeProviderIterator(iterator it) : it_(it) {}

ManeuverMode::SharedPtr ModeProviderIterator::operator*() const {

    return it_->second;

}

ModeProviderIterator& ModeProviderIterator::operator++() {

    ++it_;

    return *this;

}

bool ModeProviderIterator::operator!=(const ModeProviderIterator& other) const {

    return it_ != other.it_;

}
