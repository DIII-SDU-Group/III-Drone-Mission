#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <iii_drone_mission/behavior/condition_nodes/verify_gripper_closed_condition_node.hpp>
#include <rcutils/logging.h>

using iii_drone::behavior::VerifyGripperClosedConditionNode;
using iii_drone_interfaces::msg::GripperStatus;

namespace {

// Captures formatted rcutils log lines for the scope of one test.
class ScopedLogCapture {
public:
    ScopedLogCapture() : previous_(rcutils_logging_get_output_handler()) {
        std::lock_guard<std::mutex> lock(mutex());
        entries().clear();
        rcutils_logging_set_output_handler(&ScopedLogCapture::handler);
    }
    ~ScopedLogCapture() { rcutils_logging_set_output_handler(previous_); }

    size_t count(int severity, const std::string & needle) const {
        std::lock_guard<std::mutex> lock(mutex());
        return static_cast<size_t>(std::count_if(entries().begin(), entries().end(),
            [severity, &needle](const auto & entry) {
                return entry.first == severity && entry.second.find(needle) != std::string::npos;
            }));
    }

private:
    static void handler(const rcutils_log_location_t *, int severity, const char *,
                        rcutils_time_point_value_t, const char * format, va_list * args) {
        va_list copy;
        va_copy(copy, *args);
        char buffer[4096];
        std::vsnprintf(buffer, sizeof(buffer), format, copy);
        va_end(copy);
        std::lock_guard<std::mutex> lock(mutex());
        entries().emplace_back(severity, buffer);
    }
    static std::mutex & mutex() { static std::mutex value; return value; }
    static std::vector<std::pair<int, std::string>> & entries() {
        static std::vector<std::pair<int, std::string>> value;
        return value;
    }
    rcutils_logging_output_handler_t previous_;
};

class VerifyGripperClosedTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("verify_gripper_closed_test");
        BT::RosNodeParams params;
        params.nh = node_;
        params.default_port_value = "/payload/charger_gripper/gripper_status";
        BT::NodeConfig config;
        condition_ = std::make_unique<VerifyGripperClosedConditionNode>("verify_gripper_closed", config, params);
    }

    void TearDown() override {
        condition_.reset();
        node_.reset();
        rclcpp::shutdown();
    }

    static std::shared_ptr<GripperStatus> status(uint8_t value) {
        auto message = std::make_shared<GripperStatus>();
        message->gripper_status = value;
        return message;
    }

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<VerifyGripperClosedConditionNode> condition_;
};

}  // namespace

// The first tick of a fresh subscription has no sample yet and the tree
// retries the check: not a warning (2026-11-25 wind runs, one per landing retry).
TEST_F(VerifyGripperClosedTest, MissingFirstSampleFailsQuietly) {
    ScopedLogCapture logs;
    EXPECT_EQ(condition_->onTick(nullptr), BT::NodeStatus::FAILURE);
    EXPECT_EQ(condition_->onTick(status(GripperStatus::GRIPPER_STATUS_CLOSED)), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(condition_->onTick(nullptr), BT::NodeStatus::FAILURE);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "No gripper status"), 0U);
}

TEST_F(VerifyGripperClosedTest, StatusMissingForLongerIsReported) {
    ScopedLogCapture logs;
    EXPECT_EQ(condition_->onTick(nullptr), BT::NodeStatus::FAILURE);
    std::this_thread::sleep_for(VerifyGripperClosedConditionNode::kMissingStatusWarnAfter + std::chrono::milliseconds(50));
    EXPECT_EQ(condition_->onTick(nullptr), BT::NodeStatus::FAILURE);
    EXPECT_EQ(logs.count(RCUTILS_LOG_SEVERITY_WARN, "No gripper status message received for"), 1U);
}

TEST_F(VerifyGripperClosedTest, OpenGripperIsNotClosed) {
    EXPECT_EQ(condition_->onTick(status(GripperStatus::GRIPPER_STATUS_OPEN)), BT::NodeStatus::FAILURE);
}
