#include <filesystem>

#include <gtest/gtest.h>

#include <iii_drone_mission/mission/overview_gnss_persistence.hpp>

namespace {
using iii_drone::mission::overview_gnss::WorldGnssReference;
using iii_drone::types::point_t;

point_t point(float x, float y, float z) {
    point_t value;
    value << x, y, z;
    return value;
}

void expectPointNear(const geometry_msgs::msg::Point & actual, const point_t & expected) {
    EXPECT_NEAR(actual.x, expected.x(), 0.03);
    EXPECT_NEAR(actual.y, expected.y(), 0.03);
    EXPECT_NEAR(actual.z, expected.z(), 0.03);
}

TEST(OverviewGnssPersistence, ReprojectsPowerlineAndPylonsIntoChangedWorldOrigin) {
    namespace persistence = iii_drone::mission::overview_gnss;
    const auto root = std::filesystem::temp_directory_path() / "iii_drone_overview_reprojection_test";
    std::filesystem::remove_all(root);

    WorldGnssReference stored_reference;
    stored_reference.drone_gnss = {55.0, 10.0, 100.0};
    stored_reference.drone_world = point(2.0F, -3.0F, 4.0F);

    const point_t new_reference_global_in_old_world = point(17.0F, 5.0F, 6.0F);
    WorldGnssReference loaded_reference;
    loaded_reference.drone_gnss = persistence::worldToGnss(
        new_reference_global_in_old_world,
        stored_reference
    );
    loaded_reference.drone_world = point(-8.0F, 11.0F, 1.0F);

    iii_drone_interfaces::msg::PylonOverview pylons;
    pylons.frame_id = "world";
    pylons.pylons.resize(2);
    pylons.pylons[0].id = 1;
    pylons.pylons[0].x = -5.0;
    pylons.pylons[0].y = 2.0;
    pylons.pylons[1].id = 2;
    pylons.pylons[1].x = 25.0;
    pylons.pylons[1].y = -7.0;

    iii_drone_interfaces::msg::Powerline powerline;
    powerline.projection_plane.point.x = 3.0;
    powerline.projection_plane.point.y = -2.0;
    powerline.projection_plane.point.z = 8.0;
    powerline.projection_plane.normal.x = 1.0;
    iii_drone_interfaces::msg::SingleLine line;
    line.id = 4;
    line.header.frame_id = "world";
    line.in_field_of_view = true;
    line.pose.position.x = 4.0;
    line.pose.position.y = -1.0;
    line.pose.position.z = 7.0;
    line.pose.orientation.w = 1.0;
    line.projected_position.x = 3.0;
    line.projected_position.y = -1.0;
    line.projected_position.z = 7.0;
    powerline.lines.push_back(line);

    const auto logger = rclcpp::get_logger("overview_gnss_persistence_test");
    const auto pylon_path = root / "pylons.yaml";
    const auto powerline_path = root / "powerline.yaml";
    ASSERT_TRUE(persistence::persistPylonOverview(pylons, stored_reference, pylon_path, logger));
    ASSERT_TRUE(persistence::persistPowerlineOverview(powerline, stored_reference, powerline_path, logger));

    const auto loaded_pylons = persistence::loadPylonOverview(pylon_path, loaded_reference, logger);
    const auto loaded_powerline = persistence::loadPowerlineOverview(powerline_path, loaded_reference, logger);
    ASSERT_TRUE(loaded_pylons.has_value());
    ASSERT_TRUE(loaded_powerline.has_value());
    ASSERT_EQ(loaded_pylons->pylons.size(), 2U);
    ASSERT_EQ(loaded_powerline->lines.size(), 1U);

    for (std::size_t index = 0; index < pylons.pylons.size(); ++index) {
        const point_t original = point(
            static_cast<float>(pylons.pylons[index].x),
            static_cast<float>(pylons.pylons[index].y),
            stored_reference.drone_world.z()
        );
        const point_t expected = persistence::gnssToWorld(
            persistence::worldToGnss(original, stored_reference),
            loaded_reference
        );
        EXPECT_EQ(loaded_pylons->pylons[index].id, pylons.pylons[index].id);
        EXPECT_NEAR(loaded_pylons->pylons[index].x, expected.x(), 0.03);
        EXPECT_NEAR(loaded_pylons->pylons[index].y, expected.y(), 0.03);
    }

    const point_t original_line = point(4.0F, -1.0F, 7.0F);
    const point_t expected_line = persistence::gnssToWorld(
        persistence::worldToGnss(original_line, stored_reference),
        loaded_reference
    );
    expectPointNear(loaded_powerline->lines[0].pose.position, expected_line);
    EXPECT_EQ(loaded_powerline->lines[0].header.frame_id, "world");

    std::filesystem::remove_all(root);
}
}

