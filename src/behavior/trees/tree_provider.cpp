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
    ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    MissionSpecification::SharedPtr mission_specification
) : rclcpp::Node(
    "behavior_tree",
    "/mission/behavior_tree"
),  tf_buffer_(tf_buffer),
    maneuver_reference_client_(maneuver_reference_client),
    mission_specification_(mission_specification)
{

    RCLCPP_INFO(get_logger(), "TreeProvider::TreeProvider(): Initializing.");

    configurator_ = std::make_shared<Configurator<rclcpp::Node>>(this);

    initializeTreeExecutors();

    RCLCPP_INFO(get_logger(), "TreeProvider::TreeProvider(): Initialized.");

}

void TreeProvider::FinalizeInitialization(rclcpp::executors::MultiThreadedExecutor & executor) {

    RCLCPP_INFO(get_logger(), "TreeProvider::FinalizeInitialization(): Finalizing initialization.");

    for (auto it = tree_executors_.begin(); it != tree_executors_.end(); ++it) {

        it->second->FinalizeInitialization();

    }

    executor.add_node(get_node_base_interface());

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

void TreeProvider::initializeTreeExecutors() {

    RCLCPP_INFO(get_logger(), "TreeProvider::initializeTreeExecutors(): Initializing tree executors.");

    for (mission_specification_entry_t entry : *mission_specification_) {
    
            TreeExecutor::SharedPtr tree_executor = std::make_shared<TreeExecutor>(
                entry.key,
                entry.behavior_tree_xml_file,
                maneuver_reference_client_,
                tf_buffer_,
                configurator_,
                this
            );
    
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

