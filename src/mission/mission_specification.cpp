/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_specification.hpp>

#include <cstdlib>
#include <cctype>

using namespace iii_drone::mission;

namespace
{

std::string ExpandEnvironmentVariables(const std::string& input)
{
    std::string expanded;
    expanded.reserve(input.size());

    for (std::size_t i = 0; i < input.size(); ++i) {
        if (input[i] != '$') {
            expanded.push_back(input[i]);
            continue;
        }

        std::string variable_name;
        std::size_t consumed = i;
        if (i + 1 < input.size() && input[i + 1] == '{') {
            const std::size_t end = input.find('}', i + 2);
            if (end == std::string::npos) {
                expanded.push_back(input[i]);
                continue;
            }
            variable_name = input.substr(i + 2, end - (i + 2));
            consumed = end;
        } else {
            std::size_t j = i + 1;
            while (j < input.size() &&
                   (std::isalnum(static_cast<unsigned char>(input[j])) || input[j] == '_')) {
                ++j;
            }
            if (j == i + 1) {
                expanded.push_back(input[i]);
                continue;
            }
            variable_name = input.substr(i + 1, j - (i + 1));
            consumed = j - 1;
        }

        if (const char * value = std::getenv(variable_name.c_str()); value != nullptr) {
            expanded += value;
        }
        i = consumed;
    }

    return expanded;
}

std::string ExpandShellPath(const std::string& path)
{
    std::string expanded_path = path;
    if (!expanded_path.empty() && expanded_path[0] == '~') {
        if (const char * home = std::getenv("HOME"); home != nullptr) {
            expanded_path = std::string(home) + expanded_path.substr(1);
        }
    }
    return ExpandEnvironmentVariables(expanded_path);
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MissionSpecification::MissionSpecification(
    const std::string& mission_specification_file,
    rclcpp_lifecycle::LifecycleNode * node
) : node_(node) {
    std::string expanded_mission_specification_file = ExpandShellPath(mission_specification_file);
    YAML::Node mission_specification_node = YAML::LoadFile(expanded_mission_specification_file);

    executor_owned_mode_ = mission_specification_node["executor_owned_mode"].as<std::string>();

    YAML::Node entries = mission_specification_node["entries"];

    for (YAML::const_iterator it = entries.begin(); it != entries.end(); ++it) {

        mission_specification_entry_t entry;

        entry.key = (*it)["key"].as<std::string>();

        entry.mode_name = (*it)["mode_name"].as<std::string>();

        entry.behavior_tree_xml_file = (*it)["behavior_tree_xml_file"].as<std::string>();

        entry.behavior_tree_xml_file = ExpandShellPath(entry.behavior_tree_xml_file);

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
