#pragma once

#include <cstdint>
#include <deque>
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
        // The mode whose activation accepted the intent; empty for an intent
        // valid in every mode.
        std::string mode_key;
    };

    /**
     * Runtime intents wait here until a behavior tree applies them. An intent
     * that is valid only in some modes belongs to the mode activation that
     * accepted it: when the next activation begins, the ones no tree applied
     * expire. Otherwise the next tree that drains the buffer applies them in
     * a mode they were not meant for (2026-10-05: a recharge request accepted
     * while Inspection Demo was already leaving for a battery-low recharge was
     * applied during that charge and started a second, stay-on-cable recharge
     * as soon as the inspection resumed).
     */
    class RuntimeIntentBuffer {
    public:
        // Returns 0 (not enqueued) when mode_key, the mode the caller
        // validated the intent against, is no longer the active activation.
        uint64_t Enqueue(
            const std::string & flag_name,
            bool value,
            const builtin_interfaces::msg::Time & stamp,
            const std::string & mode_key = ""
        ) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!mode_key.empty() && mode_key != active_mode_key_) {
                return 0;
            }
            RuntimeIntentUpdate update;
            update.flag_name = flag_name;
            update.value = value;
            update.sequence_id = ++next_sequence_id_;
            update.stamp = stamp;
            update.mode_key = mode_key;
            pending_updates_.push_back(update);
            return update.sequence_id;
        }

        std::vector<RuntimeIntentUpdate> Drain() {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<RuntimeIntentUpdate> drained;
            drained.swap(pending_updates_);
            return drained;
        }

        // A mode activation began: returns the mode-scoped intents of earlier
        // activations that no tree applied, which are dropped.
        std::vector<RuntimeIntentUpdate> BeginModeActivation(const std::string & mode_key) {
            std::lock_guard<std::mutex> lock(mutex_);
            active_mode_key_ = mode_key;
            std::vector<RuntimeIntentUpdate> kept;
            std::vector<RuntimeIntentUpdate> expired;
            for (auto & update : pending_updates_) {
                (update.mode_key.empty() ? kept : expired).push_back(std::move(update));
            }
            pending_updates_.swap(kept);
            for (const auto & update : expired) {
                expired_.emplace_back(update.sequence_id, update.mode_key);
                if (expired_.size() > kExpiredHistory) {
                    expired_.pop_front();
                }
            }
            return expired;
        }

        // Whether the intent expired unapplied; mode_key is then the mode
        // whose activation ended first.
        bool Expired(uint64_t sequence_id, std::string & mode_key) const {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const auto & entry : expired_) {
                if (entry.first == sequence_id) {
                    mode_key = entry.second;
                    return true;
                }
            }
            return false;
        }

        void Clear() {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_updates_.clear();
        }

    private:
        static constexpr std::size_t kExpiredHistory = 32;

        mutable std::mutex mutex_;
        uint64_t next_sequence_id_ = 0;
        std::string active_mode_key_;
        std::vector<RuntimeIntentUpdate> pending_updates_;
        std::deque<std::pair<uint64_t, std::string>> expired_;
    };

} // namespace mission
} // namespace iii_drone
