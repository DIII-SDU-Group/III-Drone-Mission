#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include <behaviortree_cpp/blackboard.h>
#include <iii_drone_interfaces/msg/charger_status.hpp>
#include <iii_drone_mission/behavior/action_nodes/cable_charging_monitor_action_node.hpp>
#include <iii_drone_mission/behavior/condition_nodes/battery_recharge_condition_node.hpp>
#include <iii_drone_mission/behavior/latest_message_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;
using iii_drone::behavior::CableChargingMonitorActionNode;
using iii_drone::behavior::LatestMessageSubscription;
using iii_drone::behavior::ShouldRechargeBatteryLowConditionNode;

namespace {

class LatestMessageSubscriptionTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("latest_message_subscription_test");
    }

    void TearDown() override {
        node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace

TEST_F(LatestMessageSubscriptionTest, KeepsLatestMessageAndCountsDeliveries) {
    LatestMessageSubscription<std_msgs::msg::Float32> latest(
        *node_, "/test/latest_voltage", rclcpp::QoS(rclcpp::KeepLast(1)));
    auto publisher = node_->create_publisher<std_msgs::msg::Float32>(
        "/test/latest_voltage", rclcpp::QoS(rclcpp::KeepLast(1)));
    EXPECT_FALSE(latest.latest().has_value());

    std_msgs::msg::Float32 message;
    uint64_t previous_sequence = 0;
    for (float value : {22.5f, 22.4f}) {
        message.data = value;
        bool received = false;
        for (int attempt = 0; attempt < 100 && !received; ++attempt) {
            publisher->publish(message);
            rclcpp::spin_some(node_);
            const auto sample = latest.latest();
            received = sample && sample->message.data == value;
            if (!received) std::this_thread::sleep_for(10ms);
        }
        ASSERT_TRUE(received);
        EXPECT_GT(latest.latest()->sequence, previous_sequence);
        previous_sequence = latest.latest()->sequence;
    }
}

// Behavior-tree nodes are destroyed on the tree worker thread while the
// mission executor's MultiThreadedExecutor keeps delivering their topics. A
// callback that wrote into the destroyed node corrupted the heap mid-flight
// (HIL: malloc() abort at the Inspection Demo -> Reach Cable handover).
// Destruction during delivery must be safe; run under AddressSanitizer to see
// any late write.
TEST_F(LatestMessageSubscriptionTest, NodesSurviveDestructionWhileMessagesAreDelivered) {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node_);
    std::thread spinner([&executor]() { executor.spin(); });

    auto publisher_node = std::make_shared<rclcpp::Node>("latest_message_subscription_publisher");
    auto voltage_publisher = publisher_node->create_publisher<std_msgs::msg::Float32>(
        "/payload/charger_gripper/battery_voltage", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    auto status_publisher = publisher_node->create_publisher<iii_drone_interfaces::msg::ChargerStatus>(
        "/payload/charger_gripper/charger_status", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    std::atomic<bool> publishing{true};
    std::thread publisher([&]() {
        std_msgs::msg::Float32 voltage;
        voltage.data = 22.5f;
        iii_drone_interfaces::msg::ChargerStatus status;
        status.charger_status = status.CHARGER_STATUS_CHARGING;
        while (publishing) {
            voltage_publisher->publish(voltage);
            status_publisher->publish(status);
        }
    });

    auto global = BT::Blackboard::create();
    BT::NodeConfig config;
    config.blackboard = BT::Blackboard::create(global);
    for (int cycle = 0; cycle < 300; ++cycle) {
        {
            ShouldRechargeBatteryLowConditionNode battery("battery_low", config, node_, nullptr);
            CableChargingMonitorActionNode charging("charge_monitor", config, node_, nullptr, global);
            std::this_thread::sleep_for(std::chrono::microseconds(500 + 50 * (cycle % 20)));
            EXPECT_EQ(battery.executeTick(), BT::NodeStatus::FAILURE);
        }
    }

    publishing = false;
    publisher.join();
    executor.cancel();
    spinner.join();
}
