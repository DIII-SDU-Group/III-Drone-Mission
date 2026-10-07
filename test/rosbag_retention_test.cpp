#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include <iii_drone_mission/mission/rosbag_recorder_node/rosbag_retention.hpp>

using iii_drone::mission::rosbag_recorder_node::pruneRecordings;

namespace {

std::filesystem::path makeRecording(
    const std::filesystem::path & root, const std::string & name, std::size_t bytes, int age_s
) {
    const auto directory = root / name;
    std::filesystem::create_directories(directory);
    std::ofstream(directory / (name + "_0.mcap")) << std::string(bytes, 'x');
    std::ofstream(directory / "metadata.yaml") << "m";
    std::filesystem::last_write_time(
        directory, std::filesystem::file_time_type::clock::now() - std::chrono::seconds(age_s));
    return directory;
}

}  // namespace

// User decision 2026-10-05: keep the newest 10 GB of recordings. Here the cap
// fits the two newest; the older ones go, other directories stay.
TEST(RosbagRetention, KeepsTheNewestRecordingsWithinTheCap) {
    const auto root = std::filesystem::temp_directory_path() / "iii_rosbag_retention_test";
    std::filesystem::remove_all(root);
    makeRecording(root, "newest", 1000, 10);
    makeRecording(root, "second", 1000, 20);
    makeRecording(root, "third", 1000, 30);
    makeRecording(root, "oldest", 10, 40);
    std::filesystem::create_directories(root / "not_a_recording");
    std::ofstream(root / "not_a_recording" / "notes.txt") << std::string(5000, 'n');

    const auto result = pruneRecordings(root, 2500);

    EXPECT_EQ(result.removed, 2u);
    EXPECT_TRUE(std::filesystem::exists(root / "newest"));
    EXPECT_TRUE(std::filesystem::exists(root / "second"));
    // Once the cap is reached every older recording goes, small ones too.
    EXPECT_FALSE(std::filesystem::exists(root / "third"));
    EXPECT_FALSE(std::filesystem::exists(root / "oldest"));
    EXPECT_TRUE(std::filesystem::exists(root / "not_a_recording" / "notes.txt"));
    std::filesystem::remove_all(root);
}

TEST(RosbagRetention, NeverRemovesTheActiveRecording) {
    const auto root = std::filesystem::temp_directory_path() / "iii_rosbag_retention_active_test";
    std::filesystem::remove_all(root);
    const auto active = makeRecording(root, "active", 5000, 100);
    makeRecording(root, "newer", 1000, 10);

    const auto result = pruneRecordings(root, 2000, active);

    EXPECT_TRUE(std::filesystem::exists(active));
    // The active recording already uses the cap, so the finished one goes.
    EXPECT_FALSE(std::filesystem::exists(root / "newer"));
    EXPECT_EQ(result.removed, 1u);
    std::filesystem::remove_all(root);
}
