#include <gtest/gtest.h>

#include <filesystem>
#include <stdexcept>
#include <string>

#include <iii_drone_mission/mission/mission_catalog.hpp>
#include <iii_drone_mission/mission/mission_specification.hpp>

namespace mission = iii_drone::mission;

TEST(MissionCatalogTest, InstalledCatalogHasStableProfileDefaults)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    EXPECT_EQ(catalog->scope(), "local");
    EXPECT_EQ(catalog->defaultEntry("real").id, "inspection-production");
    EXPECT_EQ(catalog->defaultEntry("opti_track").id, "inspection-production");
    EXPECT_EQ(catalog->defaultEntry("sim").id, "inspection-production");
    EXPECT_EQ(catalog->defaultEntry("hil").id, "inspection-production");
}

TEST(MissionCatalogTest, CatalogJsonExplainsUnavailableEntriesWithoutPaths)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    const auto compatible = nlohmann::json::parse(catalog->catalogJson("real", false));
    const auto all = nlohmann::json::parse(catalog->catalogJson("real", true));
    ASSERT_EQ(compatible.at("entries").size(), 2U);
    ASSERT_EQ(all.at("entries").size(), 7U);
    for (const auto & entry : all.at("entries")) {
        EXPECT_TRUE(entry.contains("available"));
        EXPECT_TRUE(entry.contains("unavailable_reason"));
    }
    const std::string serialized = all.dump();
    EXPECT_EQ(serialized.find(catalog->directory().string()), std::string::npos);
    EXPECT_EQ(serialized.find("WORKSPACE_DIR"), std::string::npos);
}

TEST(MissionCatalogTest, SelectionAcceptsLogicalIdsAndRejectsPathLikeValues)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    EXPECT_EQ(catalog->entryForProfile("inspection-production", "real").classification, "production");
    EXPECT_THROW(static_cast<void>(catalog->entryForProfile("/tmp/mission.yaml", "real")), std::runtime_error);
    EXPECT_THROW(static_cast<void>(catalog->entryForProfile("ftp-legacy", "real")), std::runtime_error);
}

TEST(MissionSpecificationTest, ResolvesOnlyVerifiedContentAddressedAssets)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    mission::MissionSpecification specification(
        catalog,
        catalog->entryForProfile("inspection-production", "real"),
        nullptr
    );
    EXPECT_EQ(specification.catalog_id(), "inspection-production");
    EXPECT_EQ(specification.catalog_hash(), catalog->catalogHash());
    EXPECT_EQ(specification.classification(), "production");
    EXPECT_EQ(specification.executor_owned_mode(), "inspection_demo");
    ASSERT_EQ(specification.entries().size(), 4U);
    for (const auto & entry : specification.entries()) {
        const std::filesystem::path tree(entry.behavior_tree_xml_file);
        EXPECT_TRUE(std::filesystem::is_regular_file(tree));
        EXPECT_EQ(tree.parent_path().filename(), "sha256");
        EXPECT_EQ(tree.string().find("III-Drone-Mission/behavior_trees"), std::string::npos);
    }
    EXPECT_THROW(
        static_cast<void>(specification.GetMissionSpecificationEntry("missing")),
        std::runtime_error
    );
}

TEST(MissionSpecificationTest, ExperimentalIdentityRemainsProminent)
{
    const auto catalog = mission::MissionCatalog::LoadInstalled();
    mission::MissionSpecification specification(
        catalog,
        catalog->entryForProfile("reach-charge-leave-experimental", "real"),
        nullptr
    );
    EXPECT_EQ(specification.classification(), "experimental");
    EXPECT_NE(specification.experimental_warning().find("EXPERIMENTAL"), std::string::npos);
}
