#pragma once

#include <string>
#include <vector>

namespace iii_drone::mission::rosbag_recorder_node {

// PX4 input topics carry publishers with mixed QoS (reliable/transient-local
// mode executors and tools next to best-effort/volatile operation nodes).
// ros2 bag adapts its subscription to the first publisher it discovers and
// then refuses the incompatible ones, silently dropping their commands and
// logging "incompatible QoS" warnings. A best-effort, volatile subscription
// is compatible with every publisher.
inline const std::vector<std::string> & defaultPx4InputTopics() {
    static const std::vector<std::string> topics = {
        "/fmu/in/vehicle_command",
        "/fmu/in/vehicle_command_mode_executor",
        "/fmu/in/trajectory_setpoint",
        "/fmu/in/offboard_control_mode",
        "/fmu/in/config_overrides_request",
        "/fmu/in/mode_completed",
    };
    return topics;
}

inline bool isPx4InputTopic(const std::string & topic) {
    return topic.rfind("/fmu/in/", 0) == 0;
}

// YAML for `ros2 bag record --qos-profile-overrides-path`; empty when no
// recorded topic needs an override.
inline std::string px4InputQosOverridesYaml(
    const std::vector<std::string> & requested_topics,
    bool all_topics
) {
    const auto & candidates = all_topics ? defaultPx4InputTopics() : requested_topics;
    std::string yaml;
    for (const auto & topic : candidates) {
        if (!isPx4InputTopic(topic) || yaml.find(topic + ":\n") != std::string::npos) {
            continue;
        }
        yaml += topic + ":\n"
            "  history: keep_last\n"
            "  depth: 100\n"
            "  reliability: best_effort\n"
            "  durability: volatile\n";
    }
    return yaml;
}

} // namespace iii_drone::mission::rosbag_recorder_node
