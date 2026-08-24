#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <iii_drone_mission/mission/mission_specification.hpp>

#include <unistd.h>

namespace fs = std::filesystem;

class MissionSpecificationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  fs::path writeMissionFile(const std::string & content)
  {
    static int file_counter = 0;
    const fs::path mission_file = fs::temp_directory_path() /
      ("iii_drone_mission_specification_test_" + std::to_string(getpid()) + "_" +
      std::to_string(file_counter++) + ".yaml");
    std::ofstream out(mission_file);
    if (!out) {
      throw std::runtime_error("failed to open mission specification test file: " + mission_file.string());
    }
    out << content;
    out.close();
    if (!out) {
      throw std::runtime_error("failed to write mission specification test file: " + mission_file.string());
    }
    return mission_file;
  }
};

TEST_F(MissionSpecificationTest, LoadsEntriesAndExpandsHomePaths)
{
  const auto mission_file = writeMissionFile(R"(executor_owned_mode: executor_mode
entries:
  - key: first
    mode_name: mode_a
    behavior_tree_xml_file: ~/tree_a.xml
    next_mode: second
    allow_activate_when_disarmed: true
  - key: second
    mode_name: mode_b
    behavior_tree_xml_file: /tmp/tree_b.xml
)");

  iii_drone::mission::MissionSpecification spec(mission_file.string(), nullptr);

  const auto first = spec.GetMissionSpecificationEntry("first");
  const auto second = spec.GetMissionSpecificationEntry("second");

  EXPECT_EQ(spec.executor_owned_mode(), "executor_mode");
  EXPECT_EQ(spec.mission_specification_file(), mission_file.string());
  EXPECT_EQ(first.mode_name, "mode_a");
  EXPECT_EQ(first.next_mode, "second");
  EXPECT_TRUE(first.allow_activate_when_disarmed);
  EXPECT_EQ(first.behavior_tree_xml_file, std::string(std::getenv("HOME")) + "/tree_a.xml");
  EXPECT_EQ(second.next_mode, "");
  EXPECT_FALSE(second.allow_activate_when_disarmed);
}

TEST_F(MissionSpecificationTest, ThrowsForUnknownEntry)
{
  const auto mission_file = writeMissionFile(R"(executor_owned_mode: executor_mode
entries:
  - key: only
    mode_name: mode_a
    behavior_tree_xml_file: /tmp/tree.xml
)");

  iii_drone::mission::MissionSpecification spec(mission_file.string(), nullptr);

  EXPECT_THROW(static_cast<void>(spec.GetMissionSpecificationEntry("missing")), std::runtime_error);
}

TEST_F(MissionSpecificationTest, MissingOptionalFieldsDefaultToEmptyAndFalse)
{
  const auto mission_file = writeMissionFile(R"(executor_owned_mode: executor_mode
entries:
  - key: only
    mode_name: mode_a
    behavior_tree_xml_file: /tmp/tree.xml
)");

  iii_drone::mission::MissionSpecification spec(mission_file.string(), nullptr);

  const auto entry = spec.GetMissionSpecificationEntry("only");

  EXPECT_EQ(entry.next_mode, "");
  EXPECT_FALSE(entry.allow_activate_when_disarmed);
}

TEST_F(MissionSpecificationTest, IteratorExposesAllEntriesByKey)
{
  const auto mission_file = writeMissionFile(R"(executor_owned_mode: executor_mode
entries:
  - key: beta
    mode_name: mode_b
    behavior_tree_xml_file: /tmp/tree_b.xml
  - key: alpha
    mode_name: mode_a
    behavior_tree_xml_file: /tmp/tree_a.xml
    next_mode: beta
)");

  iii_drone::mission::MissionSpecification spec(mission_file.string(), nullptr);

  std::vector<std::string> keys;
  std::vector<std::string> modes;

  for (auto it = spec.begin(); it != spec.end(); ++it) {
    const auto entry = *it;
    keys.push_back(entry.key);
    modes.push_back(entry.mode_name);
  }

  ASSERT_EQ(keys.size(), 2U);
  EXPECT_EQ(keys[0], "alpha");
  EXPECT_EQ(keys[1], "beta");
  EXPECT_EQ(modes[0], "mode_a");
  EXPECT_EQ(modes[1], "mode_b");

  const auto entries = spec.entries();
  const auto mode_keys = spec.mode_keys();
  EXPECT_EQ(entries.size(), 2u);
  EXPECT_EQ(mode_keys.size(), 2u);
  EXPECT_EQ(mode_keys[0], "alpha");
  EXPECT_EQ(mode_keys[1], "beta");
}

TEST_F(MissionSpecificationTest, ParsesBoolIntentServices)
{
  const auto mission_file = writeMissionFile(R"(executor_owned_mode: executor_mode
entries:
  - key: only
    mode_name: mode_a
    behavior_tree_xml_file: /tmp/tree.xml
intent_services:
  - service_name: /mission/inspection_demo/trigger_recharge_now
    flag_name: inspection_demo.manual_recharge_requested
    type: bool
    valid_modes:
      - inspection_demo
  - service_name: /mission/cable_charging/stay_on_cable
    flag_name: charging.bypass_battery_full_check
    type: bool
)");

  iii_drone::mission::MissionSpecification spec(mission_file.string(), nullptr);
  const auto intent_services = spec.intent_services();

  ASSERT_EQ(intent_services.size(), 2u);
  EXPECT_EQ(intent_services[0].service_name, "/mission/inspection_demo/trigger_recharge_now");
  EXPECT_EQ(intent_services[0].flag_name, "inspection_demo.manual_recharge_requested");
  EXPECT_EQ(intent_services[0].type, "bool");
  ASSERT_EQ(intent_services[0].valid_modes.size(), 1u);
  EXPECT_EQ(intent_services[0].valid_modes[0], "inspection_demo");
  EXPECT_TRUE(intent_services[1].valid_modes.empty());
}

TEST_F(MissionSpecificationTest, RejectsUnsupportedIntentServiceType)
{
  const auto mission_file = writeMissionFile(R"(executor_owned_mode: executor_mode
entries:
  - key: only
    mode_name: mode_a
    behavior_tree_xml_file: /tmp/tree.xml
intent_services:
  - service_name: /mission/inspection_demo/trigger_recharge_now
    flag_name: inspection_demo.manual_recharge_requested
    type: string
)");

  EXPECT_THROW(
    iii_drone::mission::MissionSpecification spec(mission_file.string(), nullptr),
    std::runtime_error
  );
}
