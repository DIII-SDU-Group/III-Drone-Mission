#include <chrono>
#include <memory>
#include <thread>

#include <gtest/gtest.h>

#include <behaviortree_cpp/blackboard.h>
#include <iii_drone_interfaces/msg/charger_status.hpp>
#include <iii_drone_mission/behavior/action_nodes/cable_charging_monitor_action_node.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using iii_drone::behavior::CableChargingMonitorActionNode;

TEST(CableChargingBlackboard, NewManualBypassOverridesPriorCycleLocalValue) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cable_charging_blackboard_test");
    auto global = BT::Blackboard::create();
    auto local = BT::Blackboard::create(global);
    BT::NodeConfig config;
    config.blackboard = local;
    config.input_ports["minimum_stay_on_cable_s"] = "0";

    auto publisher = node->create_publisher<iii_drone_interfaces::msg::ChargerStatus>(
        "/payload/charger_gripper/charger_status", rclcpp::QoS(1).best_effort());
    CableChargingMonitorActionNode monitor("charge_monitor", config, node, nullptr, global);
    iii_drone_interfaces::msg::ChargerStatus status;
    status.charger_status = status.CHARGER_STATUS_FULLY_CHARGED;

    // Prove that the monitor received FULLY_CHARGED; missing telemetry would
    // also return RUNNING and hide this regression.
    local->set("charging.bypass_battery_full_check", false);
    global->set("charging.bypass_battery_full_check", false);
    bool observed_full = false;
    for (int attempt = 0; attempt < 50 && !observed_full; ++attempt) {
        publisher->publish(status);
        rclcpp::spin_some(node);
        observed_full = monitor.onStart() == BT::NodeStatus::SUCCESS;
        if (!observed_full) std::this_thread::sleep_for(20ms);
    }
    ASSERT_TRUE(observed_full);

    // The old Cable Charging local flag remains false while the next
    // Inspection execution publishes a new manual bypass to the shared one.
    local->set("charging.bypass_battery_full_check", false);
    global->set("charging.bypass_battery_full_check", true);
    EXPECT_EQ(monitor.onStart(), BT::NodeStatus::RUNNING);

    rclcpp::shutdown();
}
