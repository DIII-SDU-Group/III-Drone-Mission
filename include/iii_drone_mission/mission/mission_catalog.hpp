#pragma once

#include <behaviortree_cpp/contrib/json.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace iii_drone::mission
{

struct MissionCatalogAsset
{
    std::string asset_id;
    std::string content_hash;
    std::string kind;
    std::string logical_name;
};

struct MissionCatalogEntry
{
    std::string id;
    std::string entry_hash;
    std::string classification;
    std::string status;
    std::vector<std::string> profiles;
    std::vector<std::string> default_for;
    std::string experimental_warning;
    std::vector<MissionCatalogAsset> assets;
    nlohmann::json specification;
    nlohmann::json public_document;

    bool allowedForProfile(const std::string & profile) const;
    bool experimental() const;
};

class MissionCatalog
{
public:
    using SharedPtr = std::shared_ptr<MissionCatalog>;

    explicit MissionCatalog(std::filesystem::path directory);

    static SharedPtr LoadInstalled();

    const MissionCatalogEntry & entry(const std::string & catalog_id) const;
    const MissionCatalogEntry & entryForProfile(
        const std::string & catalog_id,
        const std::string & profile
    ) const;
    const MissionCatalogEntry & defaultEntry(const std::string & profile) const;
    std::filesystem::path resolveAsset(const std::string & asset_id) const;
    std::string catalogJson(const std::string & profile, bool include_incompatible) const;

    const std::string & catalogHash() const;
    const std::string & scope() const;
    const std::filesystem::path & directory() const;

private:
    std::filesystem::path directory_;
    std::filesystem::path content_directory_;
    std::string catalog_hash_;
    std::string scope_;
    nlohmann::json document_;
    std::map<std::string, MissionCatalogEntry> entries_;
    std::map<std::string, std::string> defaults_;
    std::map<std::string, MissionCatalogAsset> assets_;
};

}  // namespace iii_drone::mission
