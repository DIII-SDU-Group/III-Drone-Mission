#include <iii_drone_mission/mission/mission_specification.hpp>

#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>

using namespace iii_drone::mission;

namespace
{

std::string RequiredString(const nlohmann::json & document, const char * key)
{
    if (!document.contains(key) || !document.at(key).is_string() || document.at(key).get<std::string>().empty()) {
        throw std::runtime_error(std::string("resolved mission specification is missing ") + key);
    }
    return document.at(key).get<std::string>();
}

}  // namespace

MissionSpecification::MissionSpecification(
    MissionCatalog::SharedPtr catalog,
    const MissionCatalogEntry & catalog_entry,
    rclcpp_lifecycle::LifecycleNode * node
) : catalog_(std::move(catalog)),
    catalog_id_(catalog_entry.id),
    entry_hash_(catalog_entry.entry_hash),
    catalog_hash_(catalog_ != nullptr ? catalog_->catalogHash() : ""),
    classification_(catalog_entry.classification),
    compatible_profiles_(catalog_entry.profiles),
    experimental_warning_(catalog_entry.experimental_warning),
    node_(node)
{
    if (catalog_ == nullptr) {
        throw std::runtime_error("mission specification requires an installed mission catalog");
    }
    std::set<std::string> declared_behavior_tree_asset_ids;
    for (const auto & asset : catalog_entry.assets) {
        if (asset.kind == "mission_specification") {
            if (!specification_asset_id_.empty()) {
                throw std::runtime_error("mission catalog entry contains multiple specification assets");
            }
            specification_asset_id_ = asset.asset_id;
        } else if (asset.kind == "behavior_tree") {
            declared_behavior_tree_asset_ids.insert(asset.asset_id);
        }
    }
    if (specification_asset_id_.empty() || declared_behavior_tree_asset_ids.empty()) {
        throw std::runtime_error("mission catalog entry lacks exact specification/tree identities");
    }
    const auto & document = catalog_entry.specification;
    executor_owned_mode_ = RequiredString(document, "executor_owned_mode");
    if (!document.contains("entries") || !document.at("entries").is_array() || document.at("entries").empty()) {
        throw std::runtime_error("resolved mission specification has no entries");
    }
    for (const auto & item : document.at("entries")) {
        mission_specification_entry_t entry;
        entry.key = RequiredString(item, "key");
        entry.mode_name = RequiredString(item, "mode_name");
        const auto behavior_tree_asset_id = RequiredString(item, "behavior_tree_asset_id");
        if (declared_behavior_tree_asset_ids.count(behavior_tree_asset_id) == 0) {
            throw std::runtime_error(
                "resolved mission specification references a tree outside its entry closure"
            );
        }
        behavior_tree_asset_ids_.push_back(behavior_tree_asset_id);
        entry.behavior_tree_xml_file = catalog_->resolveAsset(behavior_tree_asset_id).string();
        entry.next_mode = item.value("next_mode", "");
        entry.allow_activate_when_disarmed = item.value("allow_activate_when_disarmed", false);
        if (!mission_specification_entries_.emplace(entry.key, entry).second) {
            throw std::runtime_error("resolved mission specification repeats mode key: " + entry.key);
        }
    }
    std::sort(behavior_tree_asset_ids_.begin(), behavior_tree_asset_ids_.end());
    behavior_tree_asset_ids_.erase(
        std::unique(behavior_tree_asset_ids_.begin(), behavior_tree_asset_ids_.end()),
        behavior_tree_asset_ids_.end()
    );
    if (
        std::set<std::string>(
            behavior_tree_asset_ids_.begin(), behavior_tree_asset_ids_.end()
        ) != declared_behavior_tree_asset_ids
    ) {
        throw std::runtime_error(
            "resolved mission specification tree identities differ from its entry closure"
        );
    }
    if (mission_specification_entries_.count(executor_owned_mode_) == 0) {
        throw std::runtime_error("resolved mission specification executor-owned mode is unavailable");
    }
    for (const auto & [key, entry] : mission_specification_entries_) {
        if (!entry.next_mode.empty() && mission_specification_entries_.count(entry.next_mode) == 0) {
            throw std::runtime_error("resolved mission mode " + key + " has an unavailable next mode");
        }
    }
    if (!document.contains("intent_services") || !document.at("intent_services").is_array()) {
        throw std::runtime_error("resolved mission specification intent service index is malformed");
    }
    for (const auto & item : document.at("intent_services")) {
        mission_intent_service_t intent_service;
        intent_service.service_name = RequiredString(item, "service_name");
        intent_service.flag_name = RequiredString(item, "flag_name");
        intent_service.type = RequiredString(item, "type");
        if (intent_service.type != "bool" || !item.contains("valid_modes") || !item.at("valid_modes").is_array()) {
            throw std::runtime_error("resolved mission intent service contract is unsupported");
        }
        for (const auto & mode : item.at("valid_modes")) {
            if (!mode.is_string() || mission_specification_entries_.count(mode.get<std::string>()) == 0) {
                throw std::runtime_error("resolved mission intent service has an unavailable valid mode");
            }
            intent_service.valid_modes.push_back(mode.get<std::string>());
        }
        intent_services_.push_back(intent_service);
    }
}

mission_specification_entry_t MissionSpecification::GetMissionSpecificationEntry(const std::string & key) const
{
    const auto it = mission_specification_entries_.find(key);
    if (it == mission_specification_entries_.end()) {
        throw std::runtime_error("mission specification entry not found: " + key);
    }
    return it->second;
}

MissionSpecificationIterator MissionSpecification::begin()
{
    return MissionSpecificationIterator(mission_specification_entries_.begin());
}

MissionSpecificationIterator MissionSpecification::end()
{
    return MissionSpecificationIterator(mission_specification_entries_.end());
}

std::string MissionSpecification::executor_owned_mode() const
{
    return executor_owned_mode_;
}

const std::string & MissionSpecification::catalog_id() const
{
    return catalog_id_;
}

const std::string & MissionSpecification::entry_hash() const
{
    return entry_hash_;
}

const std::string & MissionSpecification::catalog_hash() const
{
    return catalog_hash_;
}

const std::string & MissionSpecification::specification_asset_id() const
{
    return specification_asset_id_;
}

const std::vector<std::string> & MissionSpecification::behavior_tree_asset_ids() const
{
    return behavior_tree_asset_ids_;
}

const std::string & MissionSpecification::classification() const
{
    return classification_;
}

const std::vector<std::string> & MissionSpecification::compatible_profiles() const
{
    return compatible_profiles_;
}

const std::string & MissionSpecification::experimental_warning() const
{
    return experimental_warning_;
}

std::vector<mission_specification_entry_t> MissionSpecification::entries() const
{
    std::vector<mission_specification_entry_t> values;
    values.reserve(mission_specification_entries_.size());
    for (const auto & item : mission_specification_entries_) {
        values.push_back(item.second);
    }
    return values;
}

std::vector<std::string> MissionSpecification::mode_keys() const
{
    std::vector<std::string> values;
    values.reserve(mission_specification_entries_.size());
    for (const auto & item : mission_specification_entries_) {
        values.push_back(item.first);
    }
    return values;
}

std::vector<mission_intent_service_t> MissionSpecification::intent_services() const
{
    return intent_services_;
}

MissionSpecificationIterator::MissionSpecificationIterator(base_iterator it) : it_(it) {}

mission_specification_entry_t MissionSpecificationIterator::operator*() const
{
    return it_->second;
}

MissionSpecificationIterator & MissionSpecificationIterator::operator++()
{
    ++it_;
    return *this;
}

bool MissionSpecificationIterator::operator!=(const MissionSpecificationIterator & other) const
{
    return it_ != other.it_;
}
