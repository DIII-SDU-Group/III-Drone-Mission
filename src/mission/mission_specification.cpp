/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_specification.hpp>

using namespace iii_drone::mission;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MissionSpecification::MissionSpecification(const std::string& mission_specification_file) {

    wordexp_t wordexp_result;
    wordexp(mission_specification_file.c_str(), &wordexp_result, 0);
    std::string expanded_mission_specification_file = wordexp_result.we_wordv[0];
    wordfree(&wordexp_result);

    YAML::Node mission_specification_node = YAML::LoadFile(expanded_mission_specification_file);

    YAML::Node entries = mission_specification_node["entries"];

    for (YAML::const_iterator it = entries.begin(); it != entries.end(); ++it) {

        mission_specification_entry_t entry;

        entry.key = (*it)["key"].as<std::string>();

        entry.mode_name = (*it)["mode_name"].as<std::string>();

        entry.behavior_tree_xml_file = (*it)["behavior_tree_xml_file"].as<std::string>();

        if (entry.behavior_tree_xml_file[0] == '~') {

            entry.behavior_tree_xml_file = std::string(getenv("HOME")) + entry.behavior_tree_xml_file.substr(1);

        }

        mission_specification_entries_[entry.key] = entry;

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