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
    TreeProvider::SharedPtr tree_provider,
    MissionSpecification::SharedPtr mission_specification,
    rclcpp_lifecycle::LifecycleNode * node,
    float dt
) : tree_provider_(tree_provider),
    mission_specification_(mission_specification),
    node_(node),
    dt_(dt)
{

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initializing.");

    mode_node_ = std::make_shared<rclcpp::Node>(
        "mode",
        "/mission/px4_modes",
        rclcpp::NodeOptions()
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


    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initialized.");

}

void ModeProvider::FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor) {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::FinalizeInitialization(): Finalizing initialization.");

    initializeModes();

    executor.add_node(mode_node_);

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::FinalizeInitialization(): Finalized initialization.");

}

void ModeProvider::Configure(
    iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    iii_drone::configuration::ParameterBundle::SharedPtr parameters
) {

    maneuver_reference_client_ = maneuver_reference_client;
    parameters_ = parameters;

}

void ModeProvider::Cleanup() {

    maneuver_reference_client_.reset();
    parameters_.reset();

}

void ModeProvider::Start() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::Start(): Starting.");

    for (auto it = modes_.begin(); it != modes_.end(); ++it) {

        auto mode = it->second;

        mode->Register(
            tree_provider_->GetTreeExecutor(it->first),
            maneuver_reference_client_
        );

    }

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::Start(): Configured.");

}

void ModeProvider::Stop() {

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::Stop(): Stopping.");

    for (auto it = modes_.begin(); it != modes_.end(); ++it) {

        auto mode = it->second;

        mode->Unregister();

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

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initializing modes.");

    for (mission_specification_entry_t entry : *mission_specification_) {
    
        ManeuverMode::SharedPtr mode = std::make_shared<ManeuverMode>(
            *mode_node_,
            entry.mode_name,
            dt_,
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

