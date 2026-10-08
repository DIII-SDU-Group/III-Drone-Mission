#include <gtest/gtest.h>

#include <iii_drone_mission/mission/rosbag_recorder_node/rosbag_qos_overrides.hpp>

using iii_drone::mission::rosbag_recorder_node::px4InputQosOverridesYaml;

TEST(RosbagQosOverrides, OverridesOnlyRequestedPx4InputTopicsAsBestEffortVolatile) {
    const auto yaml = px4InputQosOverridesYaml(
        {"/fmu/out/vehicle_status_v1", "/fmu/in/vehicle_command", "/tf", "/fmu/in/vehicle_command"},
        false);
    EXPECT_EQ(yaml,
        "/fmu/in/vehicle_command:\n"
        "  history: keep_last\n"
        "  depth: 100\n"
        "  reliability: best_effort\n"
        "  durability: volatile\n");
}

TEST(RosbagQosOverrides, AllTopicsCoversTheKnownPx4InputTopics) {
    const auto yaml = px4InputQosOverridesYaml({}, true);
    EXPECT_NE(yaml.find("/fmu/in/vehicle_command:\n"), std::string::npos);
    EXPECT_NE(yaml.find("/fmu/in/trajectory_setpoint:\n"), std::string::npos);
    EXPECT_EQ(yaml.find("/fmu/out/"), std::string::npos);
}

TEST(RosbagQosOverrides, NoPx4InputTopicsMeansNoOverrideFile) {
    EXPECT_TRUE(px4InputQosOverridesYaml({"/tf", "/fmu/out/vehicle_odometry"}, false).empty());
}
