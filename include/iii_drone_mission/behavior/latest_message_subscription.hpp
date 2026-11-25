#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Keeps the latest message of a topic for a behavior-tree node.
     *
     * Behavior-tree nodes are destroyed on the tree worker thread while the
     * mission executor's MultiThreadedExecutor may be delivering a message to
     * them at that moment. A subscription callback that captured the node's
     * `this` then writes into freed memory and corrupts the heap. Here the
     * callback co-owns the cache, so a late delivery lands in memory that is
     * still alive, and the owning node only ever reads snapshots.
     */
    template <typename MessageT>
    class LatestMessageSubscription {
    public:
        struct Sample {
            MessageT message;
            rclcpp::Time receive_time;
            // Increments with every received message.
            uint64_t sequence;
        };

        LatestMessageSubscription(
            rclcpp::Node & node,
            const std::string & topic,
            const rclcpp::QoS & qos
        ) : state_(std::make_shared<State>()) {
            auto state = state_;
            auto clock = node.get_clock();
            subscription_ = node.create_subscription<MessageT>(
                topic,
                qos,
                [state, clock](typename MessageT::ConstSharedPtr message) {
                    const rclcpp::Time receive_time = clock->now();
                    std::lock_guard<std::mutex> lock(state->mutex);
                    state->sample = Sample{*message, receive_time, state->sample ? state->sample->sequence + 1 : 1};
                }
            );
        }

        std::optional<Sample> latest() const {
            std::lock_guard<std::mutex> lock(state_->mutex);
            return state_->sample;
        }

    private:
        struct State {
            std::mutex mutex;
            std::optional<Sample> sample;
        };

        std::shared_ptr<State> state_;
        typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

    };

} // namespace behavior
} // namespace iii_drone
