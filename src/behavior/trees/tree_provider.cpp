/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/trees/tree_provider.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::mission;
using namespace iii_drone::configuration;
using namespace iii_drone::control::maneuver;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TreeProvider::TreeProvider(
    tf2_ros::Buffer::SharedPtr tf_buffer,
    MissionSpecification::SharedPtr mission_specification
) : rclcpp::Node(
    "behavior_tree",
    "/mission/behavior_tree",
    rclcpp::NodeOptions()
),  tf_buffer_(tf_buffer),
    mission_specification_(mission_specification)
{

    // RCLCPP_INFO(get_logger(), "TreeProvider::TreeProvider(): Initializing.");

	std::string log_level = std::getenv("BEHAVIOR_TREE_LOG_LEVEL");

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_FATAL);
		}

	}



    RCLCPP_INFO(get_logger(), "TreeProvider::TreeProvider(): Initialized.");

}

void TreeProvider::FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor) {

    RCLCPP_INFO(get_logger(), "TreeProvider::FinalizeInitialization(): Finalizing initialization.");

    executor.add_node(get_node_base_interface());

}

void TreeProvider::Configure(
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) {

    RCLCPP_INFO(get_logger(), "TreeProvider::Configure(): Configuring.");

    maneuver_reference_client_ = maneuver_reference_client;

    configurator_ = std::make_shared<Configurator<rclcpp::Node>>(this, "behavior_tree");

    global_blackboard_ = BT::Blackboard::create();

    initializeTreeExecutors();

}

void TreeProvider::Cleanup() {

    RCLCPP_INFO(get_logger(), "TreeProvider::Cleanup(): Cleaning up.");

    for (auto it = tree_executors_.begin(); it != tree_executors_.end(); ++it) {

        it->second->Deinitialize();

    }

    tree_executors_.clear();

    global_blackboard_.reset();

    configurator_.reset();
    configurator_ = nullptr;

    maneuver_reference_client_.reset();

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
            global_blackboard_
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