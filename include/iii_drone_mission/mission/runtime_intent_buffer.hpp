#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

namespace iii_drone {
namespace mission {

    struct RuntimeIntentUpdate {
        std::string flag_name;
        bool value = false;
        uint64_t sequence_id = 0;
        builtin_interfaces::msg::Time stamp;
    };

    class RuntimeIntentBuffer {
    public:
        uint64_t Enqueue(
            const std::string & flag_name,
            bool value,
            const builtin_interfaces::msg::Time & stamp
        ) {
            std::lock_guard<std::mutex> lock(mutex_);
            RuntimeIntentUpdate update;
            update.flag_name = flag_name;
            update.value = value;
            update.sequence_id = ++next_sequence_id_;
            update.stamp = stamp;
            pending_updates_.push_back(update);
            return update.sequence_id;
        }

        std::vector<RuntimeIntentUpdate> Drain() {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<RuntimeIntentUpdate> drained;
            drained.swap(pending_updates_);
            return drained;
        }

        void Clear() {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_updates_.clear();
        }

    private:
        std::mutex mutex_;
        uint64_t next_sequence_id_ = 0;
        std::vector<RuntimeIntentUpdate> pending_updates_;
    };

} // namespace mission
} // namespace iii_drone
