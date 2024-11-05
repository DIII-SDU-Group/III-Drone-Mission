/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/modes/mode_provider.hpp>

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
    iii_drone::configuration::ParameterBundle::SharedPtr parameters
) : tree_provider_(tree_provider),
    mission_specification_(mission_specification),
    node_(node)
{

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initializing.");

    mode_node_ = std::make_shared<rclcpp::Node>(
        "px4_mode"
    );

	std::string log_level = std::getenv("PX4_MODE_LOG_LEVEL");

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			rcutils_logging_set_logger_level(mode_node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			rcutils_logging_set_logger_level(mode_node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			rcutils_logging_set_logger_level(mode_node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			rcutils_logging_set_logger_level(mode_node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			rcutils_logging_set_logger_level(mode_node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_FATAL);
		}

	}

    maneuver_reference_client_ = maneuver_reference_client;
    parameters_ = parameters;

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

    maneuver_reference_client_.reset();
    parameters_.reset();

}

void ModeProvider::Stop() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::Stop(): Stopping.");

    for (auto it = modes_.begin(); it != modes_.end(); ++it) {

        auto mode = it->second;

        mode->Unregister(true);

    }

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::Stop(): Stopped.");

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

    float dt = parameters_->GetParameter("maneuver_setpoint_dt").as_double();

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initializing modes.");

    for (mission_specification_entry_t entry : *mission_specification_) {

        bool is_owned_mode = mission_specification_->executor_owned_mode() == entry.key;

        RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initializing mode %s, owned mode: %d", entry.mode_name.c_str(), is_owned_mode);
    
        ManeuverMode::SharedPtr mode = std::make_shared<ManeuverMode>(
            *mode_node_,
            entry.mode_name,
            dt,
            is_owned_mode,
            entry.allow_activate_when_disarmed
        );

        modes_[entry.key] = mode;

    }

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initialized modes.");

}

void ModeProvider::deinitializeModes() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::deinitializeModes(): Deinitializing modes.");

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

