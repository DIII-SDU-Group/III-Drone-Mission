/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_specification.hpp>

using namespace iii_drone::mission;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MissionSpecification::MissionSpecification(
    const std::string& mission_specification_file,
    rclcpp_lifecycle::LifecycleNode * node
) : node_(node) {

    RCLCPP_DEBUG(
        node_->get_logger(),
        "MissionSpecification::MissionSpecification(): Initializing."
    );

    wordexp_t wordexp_result;
    wordexp(mission_specification_file.c_str(), &wordexp_result, 0);
    std::string expanded_mission_specification_file = wordexp_result.we_wordv[0];
    wordfree(&wordexp_result);

    YAML::Node mission_specification_node = YAML::LoadFile(expanded_mission_specification_file);

    executor_owned_mode_ = mission_specification_node["executor_owned_mode"].as<std::string>();

    YAML::Node entries = mission_specification_node["entries"];

    for (YAML::const_iterator it = entries.begin(); it != entries.end(); ++it) {

        mission_specification_entry_t entry;

        entry.key = (*it)["key"].as<std::string>();

        entry.mode_name = (*it)["mode_name"].as<std::string>();

        entry.behavior_tree_xml_file = (*it)["behavior_tree_xml_file"].as<std::string>();

        if (entry.behavior_tree_xml_file[0] == '~') {

            entry.behavior_tree_xml_file = std::string(getenv("HOME")) + entry.behavior_tree_xml_file.substr(1);

        }

        try {
            entry.next_mode = (*it)["next_mode"].as<std::string>();
        } catch (YAML::Exception& e){
            entry.next_mode = "";
        }

        try {
            entry.allow_activate_when_disarmed = (*it)["allow_activate_when_disarmed"].as<bool>();
        } catch (YAML::Exception& e){
            entry.allow_activate_when_disarmed = false;
        }

        mission_specification_entries_[entry.key] = entry;

        RCLCPP_DEBUG(
            node_->get_logger(),
            "MissionSpecification::MissionSpecification(): Added mission specification entry:\nkey: %s\nmode_name: %s\nbehavior_tree_xml_file: %s\nnext_mode: %s\nallow_activate_when_disarmed: %d",
            entry.key.c_str(),
            entry.mode_name.c_str(),
            entry.behavior_tree_xml_file.c_str(),
            entry.next_mode.c_str(),
            entry.allow_activate_when_disarmed
        );

    }

}

mission_specification_entry_t MissionSpecification::GetMissionSpecificationEntry(const std::string& key) const {

    auto it = mission_specification_entries_.find(key);

    if (it == mission_specification_entries_.end()) {

        std::string fatal_msg = "MissionSpecification::GetMissionSpecificationEntry(): Mission specification entry not found: " + key;

        throw std::runtime_error(fatal_msg);

    }

    return it->second;

}

MissionSpecificationIterator MissionSpecification::begin() {

    return MissionSpecificationIterator(mission_specification_entries_.begin());

}

MissionSpecificationIterator MissionSpecification::end() {

    return MissionSpecificationIterator(mission_specification_entries_.end());

}

std::string MissionSpecification::executor_owned_mode() const {

    return executor_owned_mode_;

}

MissionSpecificationIterator::MissionSpecificationIterator(base_iterator it) : it_(it) {}

mission_specification_entry_t MissionSpecificationIterator::operator*() const {

    return it_->second;

}

MissionSpecificationIterator& MissionSpecificationIterator::operator++() {

    ++it_;

    return *this;

}

bool MissionSpecificationIterator::operator!=(const MissionSpecificationIterator& other) const {

    return it_ != other.it_;

}