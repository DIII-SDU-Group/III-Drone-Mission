#include <iii_drone_mission/mission/mission_catalog.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <openssl/sha.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace iii_drone::mission
{
namespace
{

constexpr const char * kCatalogSchema = "iii.mission-catalog/v1";
constexpr const char * kSourceStateSchema = "iii.mission-source-state/v1";
const std::regex kHashPattern("sha256:[a-f0-9]{64}");

std::string ReadFile(const std::filesystem::path & path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::runtime_error("cannot read installed mission catalog file: " + path.string());
    }
    std::ostringstream output;
    output << stream.rdbuf();
    if (!stream.good() && !stream.eof()) {
        throw std::runtime_error("failed while reading installed mission catalog file: " + path.string());
    }
    return output.str();
}

std::string Sha256(const std::string & value)
{
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(value.data()), value.size(), digest);
    std::ostringstream output;
    output << "sha256:" << std::hex << std::setfill('0');
    for (const auto byte : digest) {
        output << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return output.str();
}

std::vector<std::string> StringArray(const nlohmann::json & document, const char * key)
{
    if (!document.contains(key) || !document.at(key).is_array()) {
        throw std::runtime_error(std::string("mission catalog entry has malformed ") + key);
    }
    std::vector<std::string> values;
    for (const auto & item : document.at(key)) {
        if (!item.is_string()) {
            throw std::runtime_error(std::string("mission catalog entry has a non-string ") + key);
        }
        values.push_back(item.get<std::string>());
    }
    if (!std::is_sorted(values.begin(), values.end()) ||
        std::adjacent_find(values.begin(), values.end()) != values.end()) {
        throw std::runtime_error(std::string("mission catalog entry has unsorted or duplicate ") + key);
    }
    return values;
}

std::string RequiredString(const nlohmann::json & document, const char * key)
{
    if (!document.contains(key) || !document.at(key).is_string() || document.at(key).get<std::string>().empty()) {
        throw std::runtime_error(std::string("mission catalog is missing string field: ") + key);
    }
    return document.at(key).get<std::string>();
}

}  // namespace

bool MissionCatalogEntry::allowedForProfile(const std::string & profile) const
{
    return status == "active" && std::find(profiles.begin(), profiles.end(), profile) != profiles.end();
}

bool MissionCatalogEntry::experimental() const
{
    return classification == "experimental";
}

MissionCatalog::MissionCatalog(std::filesystem::path directory)
{
    std::error_code error;
    directory_ = std::filesystem::weakly_canonical(std::move(directory), error);
    if (error || !std::filesystem::is_directory(directory_)) {
        throw std::runtime_error("installed mission catalog directory is unavailable");
    }
    const auto catalog_path = directory_ / "catalog.json";
    const auto checksum_path = directory_ / "catalog.sha256";
    content_directory_ = std::filesystem::weakly_canonical(catalog_path, error).parent_path();
    if (error || !std::filesystem::is_directory(content_directory_)) {
        throw std::runtime_error("installed mission catalog content root is unavailable");
    }
    const std::string catalog_bytes = ReadFile(catalog_path);
    const std::string checksum = ReadFile(checksum_path);
    const std::string byte_hash = Sha256(catalog_bytes);
    const std::string expected_checksum = byte_hash.substr(std::string("sha256:").size()) + "  catalog.json\n";
    if (checksum != expected_checksum) {
        throw std::runtime_error("installed mission catalog byte checksum mismatch");
    }
    try {
        document_ = nlohmann::json::parse(catalog_bytes);
    } catch (const nlohmann::json::exception & exception) {
        throw std::runtime_error(std::string("installed mission catalog is malformed JSON: ") + exception.what());
    }
    if (!document_.is_object() || document_.value("schema", "") != kCatalogSchema) {
        throw std::runtime_error("installed mission catalog schema is unsupported");
    }
    if (document_.dump() + "\n" != catalog_bytes) {
        throw std::runtime_error("installed mission catalog is not canonical JSON");
    }
    catalog_hash_ = RequiredString(document_, "catalog_hash");
    scope_ = RequiredString(document_, "scope");
    if (!std::regex_match(catalog_hash_, kHashPattern)) {
        throw std::runtime_error("installed mission catalog identity is malformed");
    }
    auto catalog_identity_document = document_;
    catalog_identity_document.erase("catalog_hash");
    if (Sha256(catalog_identity_document.dump()) != catalog_hash_) {
        throw std::runtime_error("installed mission catalog logical identity mismatch");
    }
    const std::string source_state_bytes = ReadFile(directory_ / "source-state.json");
    nlohmann::json source_state;
    try {
        source_state = nlohmann::json::parse(source_state_bytes);
    } catch (const nlohmann::json::exception & exception) {
        throw std::runtime_error(std::string("installed mission source state is malformed JSON: ") + exception.what());
    }
    if (
        !source_state.is_object() ||
        source_state.value("schema", "") != kSourceStateSchema ||
        source_state.dump() + "\n" != source_state_bytes
    ) {
        throw std::runtime_error("installed mission source state is unsupported or non-canonical");
    }
    const auto state_hash = RequiredString(source_state, "state_hash");
    auto state_identity_document = source_state;
    state_identity_document.erase("state_hash");
    if (Sha256(state_identity_document.dump()) != state_hash) {
        throw std::runtime_error("installed mission source-state identity mismatch");
    }
    if (
        !document_.contains("compatibility") ||
        !document_["compatibility"].is_object() ||
        document_["compatibility"].value("source_state_sha256", "") != state_hash
    ) {
        throw std::runtime_error("installed mission catalog/source-state binding mismatch");
    }
    if (const char * expected = std::getenv("III_MISSION_EXPECTED_CATALOG_HASH");
        expected != nullptr && *expected != '\0' && catalog_hash_ != expected) {
        throw std::runtime_error("installed mission catalog differs from the release-bound catalog identity");
    }
    if (!document_.contains("assets") || !document_["assets"].is_array()) {
        throw std::runtime_error("installed mission catalog asset index is malformed");
    }
    for (const auto & item : document_["assets"]) {
        MissionCatalogAsset asset{
            RequiredString(item, "asset_id"),
            RequiredString(item, "content_hash"),
            RequiredString(item, "kind"),
            RequiredString(item, "logical_name"),
        };
        if (
            asset.asset_id != asset.content_hash ||
            !std::regex_match(asset.asset_id, kHashPattern) ||
            !assets_.emplace(asset.asset_id, asset).second
        ) {
            throw std::runtime_error("installed mission catalog has a duplicate or inconsistent asset");
        }
    }
    if (!document_.contains("entries") || !document_["entries"].is_array()) {
        throw std::runtime_error("installed mission catalog entry index is malformed");
    }
    std::set<std::string> referenced_asset_ids;
    for (const auto & item : document_["entries"]) {
        MissionCatalogEntry entry;
        entry.id = RequiredString(item, "id");
        entry.entry_hash = RequiredString(item, "entry_hash");
        entry.classification = RequiredString(item, "classification");
        entry.status = RequiredString(item, "status");
        entry.profiles = StringArray(item, "profiles");
        entry.default_for = StringArray(item, "default_for");
        if (item.contains("experimental_warning") && item["experimental_warning"].is_string()) {
            entry.experimental_warning = item["experimental_warning"].get<std::string>();
        }
        if (!item.contains("specification") || !item["specification"].is_object()) {
            throw std::runtime_error("installed mission catalog entry has no resolved specification");
        }
        entry.specification = item["specification"];
        entry.public_document = item;
        auto entry_identity_document = item;
        entry_identity_document.erase("entry_hash");
        if (Sha256(entry_identity_document.dump()) != entry.entry_hash) {
            throw std::runtime_error("installed mission catalog entry identity mismatch: " + entry.id);
        }
        if (
            entry.classification != "production" && entry.classification != "experimental" &&
            entry.classification != "test" && entry.classification != "legacy"
        ) {
            throw std::runtime_error("installed mission catalog entry has unknown classification: " + entry.id);
        }
        if (scope_ == "qualified" && entry.classification != "production") {
            throw std::runtime_error("qualified mission catalog contains non-production entry: " + entry.id);
        }
        if (!item.contains("assets") || !item["assets"].is_array()) {
            throw std::runtime_error("installed mission catalog entry has malformed assets");
        }
        std::set<std::string> entry_asset_ids;
        for (const auto & asset_document : item["assets"]) {
            const auto asset_id = RequiredString(asset_document, "asset_id");
            const auto found = assets_.find(asset_id);
            if (found == assets_.end() || !entry_asset_ids.insert(asset_id).second) {
                throw std::runtime_error("installed mission catalog entry has unavailable or duplicate assets");
            }
            entry.assets.push_back(found->second);
            referenced_asset_ids.insert(asset_id);
        }
        const auto dependencies = StringArray(item, "dependencies");
        if (std::set<std::string>(dependencies.begin(), dependencies.end()) != entry_asset_ids) {
            throw std::runtime_error("installed mission catalog entry dependency closure differs from its assets");
        }
        if (!entries_.emplace(entry.id, entry).second) {
            throw std::runtime_error("installed mission catalog repeats entry ID: " + entry.id);
        }
    }
    if (referenced_asset_ids.size() != assets_.size()) {
        throw std::runtime_error("installed mission catalog contains unreferenced assets");
    }
    if (!document_.contains("profiles") || !document_["profiles"].is_object()) {
        throw std::runtime_error("installed mission catalog profile index is malformed");
    }
    for (const auto & [profile, descriptor] : document_["profiles"].items()) {
        if (descriptor.contains("default_entry_id") && !descriptor["default_entry_id"].is_null()) {
            const auto default_id = RequiredString(descriptor, "default_entry_id");
            const auto found = entries_.find(default_id);
            if (found == entries_.end() || !found->second.allowedForProfile(profile)) {
                throw std::runtime_error("installed mission catalog has an invalid profile default: " + profile);
            }
            defaults_.emplace(profile, default_id);
        }
    }
    for (const auto & [asset_id, asset] : assets_) {
        (void)asset;
        resolveAsset(asset_id);
    }
}

MissionCatalog::SharedPtr MissionCatalog::LoadInstalled()
{
    const auto share = std::filesystem::path(
        ament_index_cpp::get_package_share_directory("iii_drone_mission")
    );
    return std::make_shared<MissionCatalog>(share / "mission_catalog");
}

const MissionCatalogEntry & MissionCatalog::entry(const std::string & catalog_id) const
{
    const auto found = entries_.find(catalog_id);
    if (found == entries_.end()) {
        throw std::runtime_error("unknown mission catalog ID: " + catalog_id);
    }
    return found->second;
}

const MissionCatalogEntry & MissionCatalog::entryForProfile(
    const std::string & catalog_id,
    const std::string & profile
) const
{
    const auto & result = entry(catalog_id);
    if (!result.allowedForProfile(profile)) {
        throw std::runtime_error(
            "mission catalog ID " + catalog_id + " is unavailable for active profile " + profile
        );
    }
    return result;
}

const MissionCatalogEntry & MissionCatalog::defaultEntry(const std::string & profile) const
{
    const auto found = defaults_.find(profile);
    if (found == defaults_.end()) {
        throw std::runtime_error("installed mission catalog has no commissioned default for profile: " + profile);
    }
    return entryForProfile(found->second, profile);
}

std::filesystem::path MissionCatalog::resolveAsset(const std::string & asset_id) const
{
    if (!std::regex_match(asset_id, kHashPattern) || assets_.count(asset_id) == 0) {
        throw std::runtime_error("mission catalog references an unknown asset: " + asset_id);
    }
    const auto relative_digest = asset_id.substr(std::string("sha256:").size());
    const auto candidate = directory_ / "assets" / "sha256" / relative_digest;
    std::error_code error;
    const auto resolved = std::filesystem::weakly_canonical(candidate, error);
    if (error || !std::filesystem::is_regular_file(resolved)) {
        throw std::runtime_error("installed mission asset is unavailable: " + asset_id);
    }
    const auto asset_root = std::filesystem::weakly_canonical(
        content_directory_ / "assets" / "sha256", error
    );
    if (error || resolved.parent_path() != asset_root) {
        throw std::runtime_error("installed mission asset escapes the catalog: " + asset_id);
    }
    if (Sha256(ReadFile(resolved)) != asset_id) {
        throw std::runtime_error("installed mission asset content hash mismatch: " + asset_id);
    }
    return resolved;
}

std::string MissionCatalog::catalogJson(const std::string & profile, bool include_incompatible) const
{
    auto result = document_;
    result["entries"] = nlohmann::json::array();
    std::set<std::string> visible_assets;
    for (const auto & [id, entry_value] : entries_) {
        auto item = entry_value.public_document;
        const bool available = entry_value.allowedForProfile(profile);
        item["available"] = available;
        item["unavailable_reason"] = available
            ? nullptr
            : nlohmann::json("not active or incompatible with profile " + profile);
        if (available || include_incompatible) {
            result["entries"].push_back(item);
            for (const auto & asset : entry_value.assets) {
                visible_assets.insert(asset.asset_id);
            }
        }
    }
    result["assets"] = nlohmann::json::array();
    for (const auto & item : document_["assets"]) {
        if (visible_assets.count(item.at("asset_id").get<std::string>()) != 0) {
            result["assets"].push_back(item);
        }
    }
    result["active_profile"] = profile;
    return result.dump();
}

const std::string & MissionCatalog::catalogHash() const
{
    return catalog_hash_;
}

const std::string & MissionCatalog::scope() const
{
    return scope_;
}

const std::filesystem::path & MissionCatalog::directory() const
{
    return directory_;
}

}  // namespace iii_drone::mission
