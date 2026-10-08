#pragma once

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <vector>

namespace iii_drone::mission::rosbag_recorder_node {

struct RetentionResult {
    std::size_t removed = 0;
    std::uint64_t removed_bytes = 0;
    std::uint64_t kept_bytes = 0;
};

namespace detail {

inline bool isRecording(const std::filesystem::path & directory) {
    std::error_code error;
    if (std::filesystem::exists(directory / "metadata.yaml", error)) {
        return true;
    }
    for (const auto & entry : std::filesystem::directory_iterator(directory, error)) {
        const auto extension = entry.path().extension();
        if (extension == ".mcap" || extension == ".db3") {
            return true;
        }
    }
    return false;
}

inline std::uint64_t directoryBytes(const std::filesystem::path & directory) {
    std::uint64_t bytes = 0;
    std::error_code error;
    for (auto it = std::filesystem::recursive_directory_iterator(directory, error);
         !error && it != std::filesystem::recursive_directory_iterator();
         it.increment(error)) {
        std::error_code entry_error;
        if (it->is_regular_file(entry_error)) {
            bytes += it->file_size(entry_error);
        }
    }
    return bytes;
}

}  // namespace detail

/**
 * Keeps the newest recordings under artifact_root within max_bytes and
 * removes the older ones (user decision 2026-10-05: keep the newest 10 GB;
 * every mission adds a recording and nothing removed them). Recordings are
 * ordered by their directory's modification time; once the newest ones fill
 * max_bytes, that recording and every older one are removed. Only
 * directories that hold a metadata.yaml or bag files count as recordings;
 * other entries and `keep` (the active recording) are never touched.
 */
inline RetentionResult pruneRecordings(
    const std::filesystem::path & artifact_root,
    std::uint64_t max_bytes,
    const std::filesystem::path & keep = {}
) {
    struct Recording {
        std::filesystem::path path;
        std::filesystem::file_time_type modified;
        std::uint64_t bytes;
    };
    RetentionResult result;
    std::vector<Recording> recordings;
    std::error_code error;
    for (const auto & entry : std::filesystem::directory_iterator(artifact_root, error)) {
        std::error_code entry_error;
        if (!entry.is_directory(entry_error) || !detail::isRecording(entry.path())) {
            continue;
        }
        if (!keep.empty() && std::filesystem::equivalent(entry.path(), keep, entry_error)) {
            result.kept_bytes += detail::directoryBytes(entry.path());
            continue;
        }
        recordings.push_back({
            entry.path(),
            std::filesystem::last_write_time(entry.path(), entry_error),
            detail::directoryBytes(entry.path())});
    }
    std::sort(recordings.begin(), recordings.end(), [](const Recording & a, const Recording & b) {
        return a.modified > b.modified;
    });
    bool over = false;
    for (const auto & recording : recordings) {
        if (!over && result.kept_bytes + recording.bytes <= max_bytes) {
            result.kept_bytes += recording.bytes;
            continue;
        }
        over = true;
        std::error_code remove_error;
        std::filesystem::remove_all(recording.path, remove_error);
        if (!remove_error) {
            ++result.removed;
            result.removed_bytes += recording.bytes;
        }
    }
    return result;
}

}  // namespace iii_drone::mission::rosbag_recorder_node
