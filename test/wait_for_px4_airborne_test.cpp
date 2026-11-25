#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <iii_drone_mission/behavior/action_nodes/wait_for_px4_airborne_action_node.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using iii_drone::behavior::WaitForPX4AirborneActionNode;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPositionSetpoint;

namespace {

VehicleLandDetected landState(bool landed, bool maybe_landed, bool ground_contact) {
    VehicleLandDetected message;
    message.landed = landed;
    message.maybe_landed = maybe_landed;
    message.ground_contact = ground_contact;
    return message;
}

VehicleLocalPositionSetpoint thrust(double upward) {
    VehicleLocalPositionSetpoint message;
    message.thrust = {0.0f, 0.0f, static_cast<float>(-upward)};
    return message;
}

class WaitForPX4AirborneTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("wait_for_px4_airborne_test");
        publisher_ = node_->create_publisher<VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local());
        setpoint_publisher_ = node_->create_publisher<VehicleLocalPositionSetpoint>(
            "/fmu/out/vehicle_local_position_setpoint",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local());
        factory_.registerNodeType<WaitForPX4AirborneActionNode>("WaitForPX4Airborne", node_);
    }

    void TearDown() override {
        tree_.reset();
        publisher_.reset();
        setpoint_publisher_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    void createTree(int hold_ms, int timeout_ms) {
        tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
            "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">"
            "<WaitForPX4Airborne hold_ms=\"" + std::to_string(hold_ms) +
            "\" timeout_ms=\"" + std::to_string(timeout_ms) + "\"/>"
            "</BehaviorTree></root>"));
    }

    // Ticks the tree while PX4 publishes `state` and `upward_thrust` every
    // 20 ms until it finishes or `duration` passes.
    BT::NodeStatus tickWhilePublishing(
        const VehicleLandDetected & state, std::chrono::milliseconds duration, double upward_thrust = 0.8) {
        const auto end = std::chrono::steady_clock::now() + duration;
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        while (std::chrono::steady_clock::now() < end) {
            publisher_->publish(state);
            setpoint_publisher_->publish(thrust(upward_thrust));
            rclcpp::spin_some(node_);
            status = tree_->tickOnce();
            if (status != BT::NodeStatus::RUNNING) break;
            std::this_thread::sleep_for(20ms);
        }
        return status;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<VehicleLandDetected>::SharedPtr publisher_;
    rclcpp::Publisher<VehicleLocalPositionSetpoint>::SharedPtr setpoint_publisher_;
    BT::BehaviorTreeFactory factory_;
    std::unique_ptr<BT::Tree> tree_;
};

}  // namespace

TEST(WaitForPX4Airborne, AirborneNeedsAFreshSampleWithoutAnyLandedStage) {
    const rclcpp::Time now(100, 0);
    EXPECT_TRUE(WaitForPX4AirborneActionNode::Airborne(landState(false, false, false), now - rclcpp::Duration(1s), now));
    EXPECT_FALSE(WaitForPX4AirborneActionNode::Airborne(landState(false, false, true), now, now));
    EXPECT_FALSE(WaitForPX4AirborneActionNode::Airborne(landState(false, true, false), now, now));
    EXPECT_FALSE(WaitForPX4AirborneActionNode::Airborne(landState(true, false, false), now, now));
    EXPECT_FALSE(WaitForPX4AirborneActionNode::Airborne(landState(false, false, false), now - rclcpp::Duration(3s), now));
}

TEST(WaitForPX4Airborne, ThrustingNeedsAFreshSampleCommandingTheMinimumThrust) {
    const rclcpp::Time now(100, 0);
    EXPECT_TRUE(WaitForPX4AirborneActionNode::Thrusting(thrust(0.7), now - rclcpp::Duration(500ms), now, 0.1));
    EXPECT_FALSE(WaitForPX4AirborneActionNode::Thrusting(thrust(0.0), now, now, 0.1));
    EXPECT_FALSE(WaitForPX4AirborneActionNode::Thrusting(thrust(0.7), now - rclcpp::Duration(1500ms), now, 0.1));
}

// Far above the ground PX4 reports airborne as soon as it arms while its
// takeoff state machine still commands zero thrust (spool-up).
TEST_F(WaitForPX4AirborneTest, AirborneWithoutThrustIsNotEnough) {
    createTree(200, 600);
    EXPECT_EQ(tickWhilePublishing(landState(false, false, false), 2s, 0.0), BT::NodeStatus::FAILURE);
}

TEST_F(WaitForPX4AirborneTest, SucceedsAfterPx4ReportsAirborneForTheHoldTime) {
    createTree(300, 3000);
    const auto started = std::chrono::steady_clock::now();
    EXPECT_EQ(tickWhilePublishing(landState(false, false, false), 2s), BT::NodeStatus::SUCCESS);
    EXPECT_GE(std::chrono::steady_clock::now() - started, 300ms);
}

TEST_F(WaitForPX4AirborneTest, AnyLandedStageRestartsTheHold) {
    createTree(400, 3000);
    EXPECT_EQ(tickWhilePublishing(landState(false, false, false), 250ms), BT::NodeStatus::RUNNING);
    EXPECT_EQ(tickWhilePublishing(landState(false, false, true), 100ms), BT::NodeStatus::RUNNING);
    // The hold restarts: 250 ms more airborne is still short of 400 ms.
    EXPECT_EQ(tickWhilePublishing(landState(false, false, false), 250ms), BT::NodeStatus::RUNNING);
    EXPECT_EQ(tickWhilePublishing(landState(false, false, false), 1s), BT::NodeStatus::SUCCESS);
}

TEST_F(WaitForPX4AirborneTest, FailsWhenPx4StaysLandedUntilTheTimeout) {
    createTree(200, 600);
    EXPECT_EQ(tickWhilePublishing(landState(true, true, true), 2s), BT::NodeStatus::FAILURE);
}
