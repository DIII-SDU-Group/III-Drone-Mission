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
    ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    MissionSpecification::SharedPtr mission_specification,
    ParameterBundle::SharedPtr parameters,
    rclcpp::Node * node
) : tree_provider_(tree_provider),
    maneuver_reference_client_(maneuver_reference_client),
    mission_specification_(mission_specification),
    parameters_(parameters),
    node_(node)
{

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initializing.");

    initializeModes();

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::ModeProvider(): Initialized.");

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
            *node_,
            entry.mode_name,
            maneuver_reference_client_,
            tree_provider_->GetTreeExecutor(entry.key),
            parameters_->GetParameter("maneuver_setpoint_dt").as_double()
        );

        if (!mode->doRegister()) {

            std::string fatal_msg = "ModeProvider::initializeModes(): Registration failed: " + entry.mode_name;

            RCLCPP_FATAL(node_->get_logger(), fatal_msg.c_str());

            throw std::runtime_error(fatal_msg);

        }

        mode->RegisterAsOffboardMode();

        modes_[entry.key] = mode;

    }

    RCLCPP_INFO(node_->get_logger(), "ModeProvider::initializeModes(): Initialized modes.");

}

ModeProviderIterator ModeProvider::begin() {

    return ModeProviderIterator(modes_.begin());

}

ModeProviderIterator ModeProvider::end() {

    return ModeProviderIterator(modes_.end());

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

