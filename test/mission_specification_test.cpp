#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <iii_drone_mission/mission/mission_specification.hpp>

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
    const fs::path mission_file = fs::temp_directory_path() / "iii_drone_mission_specification_test.yaml";
    std::ofstream out(mission_file);
    out << content;
    out.close();
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
}
