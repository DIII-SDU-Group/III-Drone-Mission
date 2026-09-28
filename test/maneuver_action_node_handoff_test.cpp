#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <iii_drone_configuration/configuration.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_interfaces/action/hover.hpp>
#include <iii_drone_interfaces/action/hover_by_object.hpp>

#define private public
#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>
#undef private

#include <iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp>
#include <iii_drone_mission/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp>

namespace {

using iii_drone::adapters::px4::VehicleOdometryAdapter;
using iii_drone::configuration::Configuration;
using iii_drone::configuration::configuration_entry_t;
using iii_drone::control::maneuver::ManeuverReferenceClient;
using Hover = iii_drone_interfaces::action::Hover;
using HoverByObject = iii_drone_interfaces::action::HoverByObject;

constexpr char kRequestA[] = "mri1-00000000000000010000000000000001-0000000000000001";
constexpr char kRequestB[] = "mri1-00000000000000020000000000000002-0000000000000002";
constexpr char kRequestC[] = "mri1-00000000000000030000000000000003-0000000000000003";

class RclcppContext {
public:
    RclcppContext() : initialized_here_(!rclcpp::ok()) {
        if (initialized_here_) {
            rclcpp::init(0, nullptr);
        }
    }

    ~RclcppContext() {
        if (initialized_here_) {
            rclcpp::shutdown();
        }
    }

private:
    bool initialized_here_;
};

Configuration::SharedPtr makeConfiguration() {
    const std::vector<configuration_entry_t> entries{
        {"/control/maneuver_controller/maneuver_execution_period_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/reference_stream_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/reference_loss_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_position_tolerance_m", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_velocity_tolerance_m_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_acceleration_tolerance_m_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_yaw_tolerance_rad", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_yaw_rate_tolerance_rad_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/control/maneuver_controller/controlled_cancel_settle_time_s", rclcpp::ParameterType::PARAMETER_DOUBLE},
        {"/mission/reference_rebase_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/wait_for_maneuver_start_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/max_failed_attempts_during_maneuver", rclcpp::ParameterType::PARAMETER_INTEGER},
        {"/mission/use_nans_when_hovering", rclcpp::ParameterType::PARAMETER_BOOL},
        {"/mission/get_reference_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER},
    };
    return std::make_shared<Configuration>(
        "maneuver-action-node-handoff-test",
        entries,
        [](const std::string & name) -> rclcpp::Parameter {
            if (name == "/mission/use_nans_when_hovering") {
                return rclcpp::Parameter(name, false);
            }
            if (
                name == "/control/maneuver_controller/controlled_cancel_max_jerk_m_s3" ||
                name == "/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3" ||
                name == "/mission/reference_continuity_position_tolerance_m" ||
                name == "/mission/reference_continuity_velocity_tolerance_m_s" ||
                name == "/mission/reference_continuity_acceleration_tolerance_m_s2" ||
                name == "/mission/reference_continuity_yaw_tolerance_rad" ||
                name == "/mission/reference_continuity_yaw_rate_tolerance_rad_s" ||
                name == "/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2" ||
                name == "/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2" ||
                name == "/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2" ||
                name == "/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s"
                || name == "/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s"
                || name == "/control/maneuver_controller/controlled_cancel_settle_time_s"
            ) {
                return rclcpp::Parameter(name, 1.0);
            }
            return rclcpp::Parameter(name, 10000);
        }
    );
}

class HandoffActionNode final : public iii_drone::behavior::ManeuverActionNode<Hover> {
public:
    HandoffActionNode(
        const std::string & name,
        const BT::NodeConfig & config,
        const BT::RosNodeParams & params,
        ManeuverReferenceClient::SharedPtr client,
        bool attach_to_active_stream,
        bool builder_succeeds = true
    ) : ManeuverActionNode<Hover>(name, config, params, std::move(client)),
        attach_to_active_stream_(attach_to_active_stream),
        builder_succeeds_(builder_succeeds) {}

    bool setManeuverGoal(Goal &) override {
        return builder_succeeds_;
    }

    void useFinalReference() {
        setGetFinalReferenceCallback([](const auto &) {
            return iii_drone::control::Reference();
        });
    }

    void keepStreamOnSuccess() { keep_stream_on_success_ = true; }

protected:
    bool shouldAttachToActiveManeuverStreamOnGoalAccepted() const override {
        return attach_to_active_stream_;
    }

    bool shouldStopManeuverOnSuccessfulResult(const WrappedResult &) const override {
        return !keep_stream_on_success_;
    }

private:
    bool attach_to_active_stream_;
    bool builder_succeeds_;
    bool keep_stream_on_success_ = false;
};

struct ActionServer {
    explicit ActionServer(bool accept_goal)
    : node(std::make_shared<rclcpp::Node>("maneuver_handoff_action_server")),
      accept_goal(accept_goal) {
        server = rclcpp_action::create_server<Hover>(
            node,
            "/maneuver_handoff_action",
            [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Hover::Goal> goal) {
                received_request_identities.push_back(goal->request_identity);
                ++goal_requests;
                return this->accept_goal.load()
                    ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                    : rclcpp_action::GoalResponse::REJECT;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Hover>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Hover>>) {}
        );
        executor.add_node(node);
    }

    ~ActionServer() {
        executor.remove_node(node);
    }

    void spin() {
        executor.spin_some();
    }

    rclcpp::Node::SharedPtr node;
    std::atomic<bool> accept_goal;
    std::atomic_uint goal_requests{0};
    std::vector<std::string> received_request_identities;
    rclcpp_action::Server<Hover>::SharedPtr server;
    rclcpp::executors::SingleThreadedExecutor executor;
};

struct HoverByObjectActionServer {
    HoverByObjectActionServer()
    : node(std::make_shared<rclcpp::Node>("object_handoff_action_server")) {
        server = rclcpp_action::create_server<HoverByObject>(
            node,
            "/object_handoff_action",
            [this](const rclcpp_action::GoalUUID &,
                std::shared_ptr<const HoverByObject::Goal> goal) {
                request_identity = goal->request_identity;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<HoverByObject>>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<HoverByObject>>) {});
        executor.add_node(node);
    }
    ~HoverByObjectActionServer() { executor.remove_node(node); }
    void spin() { executor.spin_some(); }

    rclcpp::Node::SharedPtr node;
    std::string request_identity;
    rclcpp_action::Server<HoverByObject>::SharedPtr server;
    rclcpp::executors::SingleThreadedExecutor executor;
};

struct Fixture {
    explicit Fixture(bool accept_goal)
    : mission_node(std::make_shared<rclcpp::Node>("maneuver_handoff_action_client")),
      core_node(std::make_shared<rclcpp_lifecycle::LifecycleNode>("maneuver_handoff_core")),
      history(std::make_shared<iii_drone::utils::History<VehicleOdometryAdapter>>(4)),
      client(std::make_shared<ManeuverReferenceClient>(
          core_node.get(),
          history,
          makeConfiguration(),
          core_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)
      )),
      server(accept_goal) {}

    void makeActivePredecessor() {
        client->reference_mode_.Store(ManeuverReferenceClient::MANEUVER);
        client->reference_stream_guard_.expectGeneration("g1");
        client->active_stream_id_ = "g1";
        client->active_request_identity_ = kRequestA;
        client->last_applied_sequence_ = 100;
    }

    void commitStream(const std::string & request_identity, const std::string & stream_id,
                      bool terminal_hold_active = false) {
        auto message = std::make_shared<iii_drone_interfaces::msg::ManeuverReferenceStream>();
        message->stream_id = stream_id;
        message->request_identity = request_identity;
        message->sequence = 1;
        message->state = iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE;
        message->is_valid = true;
        message->terminal_hold_active = terminal_hold_active;
        message->produced_at = core_node->now();
        message->valid_until = core_node->now() + rclcpp::Duration::from_seconds(30.0);
        message->reference = iii_drone::adapters::ReferenceAdapter(
            iii_drone::control::Reference()
        ).ToMsg();
        client->receiveReferenceStream(message);
        client->GetReference(0.02, [] {});
    }

    BT::NodeConfig config(int stop_delay_ms = -1) const {
        BT::NodeConfig config;
        config.blackboard = BT::Blackboard::create();
        config.output_ports.insert({"terminal_state", "{terminal_state}"});
        config.input_ports.insert({"stop_maneuver_after_timeout_ms", std::to_string(stop_delay_ms)});
        return config;
    }

    BT::RosNodeParams params() const {
        BT::RosNodeParams params(mission_node, "/maneuver_handoff_action");
        params.server_timeout = std::chrono::milliseconds(250);
        params.wait_for_server_timeout = std::chrono::milliseconds(500);
        return params;
    }

    BT::NodeConfig objectConfig(int stop_delay_ms = -1) const {
        auto result = config(stop_delay_ms);
        iii_drone_interfaces::msg::Target target;
        target.target_type = iii_drone_interfaces::msg::Target::TARGET_TYPE_CABLE;
        target.target_id = 1;
        result.blackboard->set("object_target", target);
        result.input_ports.insert({"target", "{object_target}"});
        result.input_ports.insert({"duration_s", "10"});
        result.input_ports.insert({"sustain_action", "false"});
        return result;
    }

    BT::RosNodeParams objectParams() const {
        BT::RosNodeParams result(mission_node, "/object_handoff_action");
        result.server_timeout = std::chrono::milliseconds(250);
        result.wait_for_server_timeout = std::chrono::milliseconds(500);
        return result;
    }

    bool tickObjectUntilGoalResponse(
        iii_drone::behavior::HoverByObjectManeuverActionNode & action,
        HoverByObjectActionServer & object_server) {
        for (int attempt = 0; attempt < 50; ++attempt) {
            object_server.spin();
            action.tick();
            if (client->pending_goal_handoff_ && client->pending_goal_handoff_->goal_accepted) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    bool tickUntilGoalResponse(HandoffActionNode & action) {
        for (int attempt = 0; attempt < 50; ++attempt) {
            server.spin();
            action.tick();
            if (!client->pending_goal_handoff_ || client->pending_goal_handoff_->goal_accepted) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
    }

    rclcpp::Node::SharedPtr mission_node;
    rclcpp_lifecycle::LifecycleNode::SharedPtr core_node;
    iii_drone::utils::History<VehicleOdometryAdapter>::SharedPtr history;
    ManeuverReferenceClient::SharedPtr client;
    ActionServer server;
};

TEST(ManeuverActionNodeHandoff, OrdinaryTickBeginsBeforeDispatchAndConfirmAwaitsG2) {
    RclcppContext context;
    Fixture fixture(true);
    fixture.makeActivePredecessor();
    HandoffActionNode action("ordinary", fixture.config(), fixture.params(), fixture.client, false);

    EXPECT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    EXPECT_FALSE(fixture.client->pending_goal_handoff_->preserve_active_predecessor_on_cancel);
    ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    EXPECT_TRUE(fixture.client->pending_goal_handoff_->goal_accepted);
    EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::WAIT_FOR_MANEUVER_START);
    EXPECT_EQ(fixture.server.goal_requests.load(), 1U);
    ASSERT_EQ(fixture.server.received_request_identities.size(), 1U);
    EXPECT_EQ(
        fixture.server.received_request_identities.front(),
        fixture.client->pending_goal_handoff_->request_identity
    );
}

TEST(ManeuverActionNodeHandoff, BlendedTickBeginsBeforeDispatchAndKeepsG1RunningUntilG2) {
    RclcppContext context;
    Fixture fixture(true);
    fixture.makeActivePredecessor();
    HandoffActionNode action("blended", fixture.config(), fixture.params(), fixture.client, true);

    EXPECT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    EXPECT_TRUE(fixture.client->pending_goal_handoff_->goal_accepted);
    EXPECT_TRUE(fixture.client->pending_goal_handoff_->preserve_active_predecessor_on_cancel);
    EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.server.goal_requests.load(), 1U);
}

TEST(ManeuverActionNodeHandoff, InitialGoalIsStampedAndBoundBeforeDispatch) {
    RclcppContext context;
    Fixture fixture(true);
    HandoffActionNode action("initial", fixture.config(), fixture.params(), fixture.client, false);

    EXPECT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    const std::string request_identity = fixture.client->pending_goal_handoff_->request_identity;
    EXPECT_TRUE(iii_drone::control::maneuver::isValidManeuverRequestIdentity(request_identity));
    EXPECT_TRUE(fixture.client->pending_goal_handoff_->predecessor_stream_id.empty());
    ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
    ASSERT_EQ(fixture.server.received_request_identities.size(), 1U);
    EXPECT_EQ(fixture.server.received_request_identities.front(), request_identity);
}

TEST(ManeuverActionNodeHandoff, BuilderFailureRetiresIdentityBeforeZeroDispatch) {
    RclcppContext context;
    Fixture fixture(true);
    HandoffActionNode action("builder_failure", fixture.config(), fixture.params(), fixture.client, false, false);

    EXPECT_EQ(action.tick(), BT::NodeStatus::FAILURE);
    fixture.server.spin();
    EXPECT_EQ(fixture.server.goal_requests.load(), 0U);
    EXPECT_FALSE(fixture.client->pending_goal_handoff_);
}

TEST(ManeuverActionNodeHandoff, BlendedEarlyMatchingStreamSurvivesLateAcceptance) {
    RclcppContext context;
    Fixture fixture(true);
    fixture.makeActivePredecessor();
    HandoffActionNode action("blended_early", fixture.config(), fixture.params(), fixture.client, true);

    EXPECT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    const std::string request_identity = fixture.client->pending_goal_handoff_->request_identity;
    auto message = std::make_shared<iii_drone_interfaces::msg::ManeuverReferenceStream>();
    message->stream_id = "blended:g2";
    message->request_identity = request_identity;
    message->sequence = 1;
    message->state = iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE;
    message->is_valid = true;
    message->produced_at = fixture.core_node->now();
    message->valid_until = fixture.core_node->now() + rclcpp::Duration::from_seconds(30.0);
    message->reference = iii_drone::adapters::ReferenceAdapter(iii_drone::control::Reference()).ToMsg();
    fixture.client->receiveReferenceStream(message);
    fixture.client->GetReference(0.02, [] {});
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    EXPECT_TRUE(fixture.client->pending_goal_handoff_->successor_consumed);

    ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
    EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    EXPECT_EQ(fixture.client->active_request_identity_, request_identity);
    EXPECT_EQ(fixture.client->reference_stream_guard_.streamId(), "blended:g2");
    EXPECT_FALSE(fixture.client->pending_goal_handoff_);
}

TEST(ManeuverActionNodeHandoff, FailedBeginReturnsFailureWithoutGoalDispatch) {
    RclcppContext context;
    Fixture fixture(true);
    fixture.makeActivePredecessor();
    ASSERT_TRUE(fixture.client->BeginManeuverGoalHandoff(kRequestB));
    HandoffActionNode action("failed_begin", fixture.config(), fixture.params(), fixture.client, false);

    EXPECT_EQ(action.tick(), BT::NodeStatus::FAILURE);
    fixture.server.spin();
    EXPECT_EQ(fixture.server.goal_requests.load(), 0U);
}

TEST(ManeuverActionNodeHandoff, RejectionAndHaltRoutePendingOwnershipToPhaseSpecificCleanup) {
    RclcppContext context;
    Fixture rejected(false);
    rejected.makeActivePredecessor();
    HandoffActionNode ordinary("rejected", rejected.config(), rejected.params(), rejected.client, false);

    EXPECT_EQ(ordinary.tick(), BT::NodeStatus::RUNNING);
    for (int attempt = 0; attempt < 50 && ordinary.status() == BT::NodeStatus::RUNNING; ++attempt) {
        rejected.server.spin();
        ordinary.tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(ordinary.status(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(rejected.client->reference_mode_.Load(), ManeuverReferenceClient::HOVER);

    Fixture halted(true);
    halted.makeActivePredecessor();
    HandoffActionNode blended("halted", halted.config(), halted.params(), halted.client, true);
    EXPECT_EQ(blended.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(halted.tickUntilGoalResponse(blended));
    blended.onHalt();
    EXPECT_EQ(halted.client->reference_mode_.Load(), ManeuverReferenceClient::HOVER);
}

TEST(ManeuverActionNodeHandoff, LateTerminalPathsCannotStopPendingOrCommittedSuccessor) {
    RclcppContext context;
    using WrappedResult = BT::RosActionNode<Hover>::WrappedResult;
    for (int phase = 0; phase < 2; ++phase) {
        for (int terminal_path = 0; terminal_path < 5; ++terminal_path) {
            SCOPED_TRACE("phase=" + std::to_string(phase) +
                " terminal_path=" + std::to_string(terminal_path));
            Fixture fixture(true);
            HandoffActionNode action("late_a", fixture.config(), fixture.params(), fixture.client, false);
            if (terminal_path == 3) {
                action.useFinalReference();
            }
            ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
            ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
            ASSERT_EQ(fixture.server.received_request_identities.size(), 1U);
            const std::string request_a = fixture.server.received_request_identities.front();
            fixture.commitStream(request_a, "a:g1");
            ASSERT_EQ(fixture.client->active_request_identity_, request_a);

            ASSERT_TRUE(fixture.client->BeginManeuverGoalHandoff(kRequestB));
            if (phase == 1) {
                ASSERT_TRUE(fixture.client->ConfirmManeuverGoalHandoff(kRequestB));
                fixture.commitStream(kRequestB, "b:g2");
                ASSERT_EQ(fixture.client->active_request_identity_, kRequestB);
            }

            switch (terminal_path) {
                case 0:
                    action.onHalt();
                    break;
                case 1:
                    action.onFailure(BT::ActionNodeErrorCode::ACTION_ABORTED);
                    break;
                default: {
                    WrappedResult result{};
                    result.code = terminal_path == 2
                        ? rclcpp_action::ResultCode::ABORTED
                        : rclcpp_action::ResultCode::SUCCEEDED;
                    action.onResultReceived(result);
                    break;
                }
            }
            if (phase == 0) {
                ASSERT_TRUE(fixture.client->pending_goal_handoff_);
                EXPECT_EQ(fixture.client->pending_goal_handoff_->request_identity, kRequestB);
                EXPECT_EQ(fixture.client->active_request_identity_, request_a);
            } else {
                EXPECT_FALSE(fixture.client->pending_goal_handoff_);
                EXPECT_EQ(fixture.client->active_request_identity_, kRequestB);
                EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
            }
        }
    }
}

TEST(ManeuverActionNodeHandoff, DelayedTerminalCallbackKeepsItsSchedulingOwner) {
    RclcppContext context;
    using WrappedResult = BT::RosActionNode<Hover>::WrappedResult;
    for (bool final_reference : {false, true}) {
        SCOPED_TRACE(final_reference ? "final reference" : "state hold");
        Fixture fixture(true);
        HandoffActionNode action("delayed_a", fixture.config(30000), fixture.params(), fixture.client, false);
        if (final_reference) {
            action.useFinalReference();
        }
        ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
        ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
        ASSERT_EQ(fixture.server.received_request_identities.size(), 1U);
        fixture.commitStream(fixture.server.received_request_identities.front(), "a:delayed");
        WrappedResult result{};
        result.code = rclcpp_action::ResultCode::SUCCEEDED;
        EXPECT_EQ(action.onResultReceived(result), BT::NodeStatus::SUCCESS);
        ASSERT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);
        const auto old_callback = *fixture.client->stop_maneuver_timer_callback_;
        ASSERT_TRUE(old_callback);

        ASSERT_TRUE(fixture.client->BeginManeuverGoalHandoff(kRequestB));
        old_callback();
        ASSERT_TRUE(fixture.client->pending_goal_handoff_);
        EXPECT_EQ(fixture.client->pending_goal_handoff_->request_identity, kRequestB);
        ASSERT_TRUE(fixture.client->ConfirmManeuverGoalHandoff(kRequestB));
        fixture.commitStream(kRequestB, "b:after_delayed");
        old_callback();
        EXPECT_EQ(fixture.client->active_request_identity_, kRequestB);
        EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::MANEUVER);
    }
}

TEST(ManeuverActionNodeHandoff, DelayedTerminalCallbackCompletesItsOwner) {
    RclcppContext context;
    for (bool final_reference : {false, true}) {
        SCOPED_TRACE(final_reference ? "final reference" : "state hold");
        Fixture fixture(true);
        HandoffActionNode action("owned_delay", fixture.config(30000), fixture.params(), fixture.client, false);
        if (final_reference) {
            action.useFinalReference();
        }
        ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
        ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
        ASSERT_EQ(fixture.server.received_request_identities.size(), 1U);
        fixture.commitStream(fixture.server.received_request_identities.front(), "a:owned_delay");
        BT::RosActionNode<Hover>::WrappedResult result{};
        result.code = rclcpp_action::ResultCode::SUCCEEDED;
        EXPECT_EQ(action.onResultReceived(result), BT::NodeStatus::SUCCESS);
        ASSERT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);
        const auto callback = *fixture.client->stop_maneuver_timer_callback_;
        ASSERT_TRUE(callback);
        callback();
        EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::HOVER);
        EXPECT_TRUE(fixture.client->active_request_identity_.empty());
    }
}

TEST(ManeuverActionNodeHandoff, SuccessfulGoalWithoutReferenceHonorsOwnedStopDelay) {
    RclcppContext context;
    for (const int delay_ms : {0, 30000}) {
        SCOPED_TRACE(delay_ms);
        Fixture fixture(true);
        HandoffActionNode action("no_reference", fixture.config(delay_ms), fixture.params(), fixture.client, false);
        ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
        ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
        ASSERT_TRUE(fixture.client->pending_goal_handoff_);
        const auto request_identity = fixture.client->pending_goal_handoff_->request_identity;
        BT::RosActionNode<Hover>::WrappedResult result{};
        result.code = rclcpp_action::ResultCode::SUCCEEDED;
        EXPECT_EQ(action.onResultReceived(result), BT::NodeStatus::SUCCESS);
        if (delay_ms > 0) {
            // A fast action may finish before DDS delivers its first sample.
            // Its explicit retention interval must survive that ordering.
            ASSERT_TRUE(fixture.client->pending_goal_handoff_);
            EXPECT_EQ(fixture.client->pending_goal_handoff_->request_identity, request_identity);
            EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::WAIT_FOR_MANEUVER_STOP);
            const auto callback = *fixture.client->stop_maneuver_timer_callback_;
            ASSERT_TRUE(callback);
            callback();
        }
        EXPECT_FALSE(fixture.client->pending_goal_handoff_);
        EXPECT_EQ(fixture.client->reference_mode_.Load(), ManeuverReferenceClient::HOVER);
    }
}

TEST(ManeuverActionNodeHandoff, DeferredTerminalStopCannotReportActionSuccess) {
    RclcppContext context;
    Fixture fixture(true);
    ASSERT_TRUE(fixture.client->BeginManeuverGoalHandoff(kRequestA));
    ASSERT_TRUE(fixture.client->ConfirmManeuverGoalHandoff(kRequestA));
    fixture.commitStream(kRequestA, "ftp:terminal", true);
    ASSERT_EQ(fixture.client->active_request_identity_, kRequestA);
    const auto predecessor_command = fixture.client->reference_.Load();

    HandoffActionNode action("object_result", fixture.config(), fixture.params(),
        fixture.client, false);
    action.useFinalReference();
    ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    const auto object_request = fixture.client->pending_goal_handoff_->request_identity;

    BT::RosActionNode<Hover>::WrappedResult result{};
    result.code = rclcpp_action::ResultCode::SUCCEEDED;
    EXPECT_EQ(action.onResultReceived(result), BT::NodeStatus::FAILURE);
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    EXPECT_EQ(fixture.client->pending_goal_handoff_->request_identity, object_request);
    EXPECT_EQ(fixture.client->active_request_identity_, kRequestA);
    EXPECT_LT((fixture.client->reference_.Load().position() -
        predecessor_command.position()).norm(), 1.0e-6);
    EXPECT_FALSE(fixture.client->BeginManeuverGoalHandoff(kRequestC));
}

TEST(ManeuverActionNodeHandoff, IntentionalSuccessHandoffRelinquishesLocalOwnership) {
    RclcppContext context;
    Fixture fixture(true);
    HandoffActionNode action("handoff_a", fixture.config(), fixture.params(), fixture.client, false);
    action.keepStreamOnSuccess();
    ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.tickUntilGoalResponse(action));
    ASSERT_EQ(fixture.server.received_request_identities.size(), 1U);
    fixture.commitStream(fixture.server.received_request_identities.front(), "a:handoff");
    BT::RosActionNode<Hover>::WrappedResult result{};
    result.code = rclcpp_action::ResultCode::SUCCEEDED;
    EXPECT_EQ(action.onResultReceived(result), BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(fixture.client->BeginManeuverGoalHandoff(kRequestB));
    action.onHalt();
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    EXPECT_EQ(fixture.client->pending_goal_handoff_->request_identity, kRequestB);
}

TEST(ManeuverActionNodeHandoff, HoverByObjectSuccessRetainsAppliedObjectForImmediateLanding) {
    RclcppContext context;
    Fixture fixture(true);
    HoverByObjectActionServer object_server;
    iii_drone::behavior::HoverByObjectManeuverActionNode action(
        "object_hover_success", fixture.objectConfig(), fixture.objectParams(), fixture.client);

    ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.tickObjectUntilGoalResponse(action, object_server));
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    ASSERT_TRUE(fixture.client->pending_goal_handoff_->goal_accepted);
    const auto request_identity = object_server.request_identity;
    ASSERT_TRUE(iii_drone::control::maneuver::isValidManeuverRequestIdentity(request_identity));

    auto stream = std::make_shared<iii_drone_interfaces::msg::ManeuverReferenceStream>();
    stream->stream_id = "object:hover:g1";
    stream->request_identity = request_identity;
    stream->sequence = 1;
    stream->state = iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE;
    stream->is_valid = true;
    stream->object_tracking_active = true;
    stream->produced_at = fixture.core_node->now();
    stream->valid_until = fixture.core_node->now() + rclcpp::Duration::from_seconds(30.0);
    stream->reference = iii_drone::adapters::ReferenceAdapter(
        iii_drone::control::Reference(iii_drone::types::point_t(1.2F, -0.3F, 2.0F),
            0.2, iii_drone::types::vector_t(0.08F, 0.0F, 0.02F), 0.01,
            iii_drone::types::vector_t(0.01F, 0.0F, 0.0F))).ToMsg();
    fixture.client->receiveReferenceStream(stream);
    fixture.client->GetReference(0.02, [] {});
    ASSERT_FALSE(fixture.client->pending_goal_handoff_);
    ASSERT_TRUE(fixture.client->currentAppliedObjectTrackingStream(request_identity));
    const auto applied_command = fixture.client->reference_.Load();

    BT::RosActionNode<HoverByObject>::WrappedResult result{};
    result.code = rclcpp_action::ResultCode::SUCCEEDED;
    ASSERT_EQ(action.onResultReceived(result), BT::NodeStatus::SUCCESS);
    EXPECT_FALSE(fixture.client->object_stop_);
    EXPECT_EQ(fixture.client->active_request_identity_, request_identity);
    EXPECT_TRUE(fixture.client->currentAppliedObjectTrackingStream(request_identity));
    const auto after_completion = fixture.client->reference_.Load();
    EXPECT_LT((after_completion.position() - applied_command.position()).norm(), 1.0e-6);
    EXPECT_LT((after_completion.velocity() - applied_command.velocity()).norm(), 1.0e-6);
    EXPECT_LT((after_completion.acceleration() - applied_command.acceleration()).norm(), 1.0e-6);
    EXPECT_TRUE(fixture.client->BeginManeuverGoalHandoff(kRequestB));
}

TEST(ManeuverActionNodeHandoff, HoverByObjectUnappliedSuccessFailsWithoutRetiringRequest) {
    RclcppContext context;
    Fixture fixture(true);
    HoverByObjectActionServer object_server;
    iii_drone::behavior::HoverByObjectManeuverActionNode action(
        "object_hover_unapplied", fixture.objectConfig(), fixture.objectParams(), fixture.client);
    ASSERT_EQ(action.tick(), BT::NodeStatus::RUNNING);
    ASSERT_TRUE(fixture.tickObjectUntilGoalResponse(action, object_server));
    const auto request_identity = object_server.request_identity;
    ASSERT_TRUE(iii_drone::control::maneuver::isValidManeuverRequestIdentity(request_identity));
    auto stream = std::make_shared<iii_drone_interfaces::msg::ManeuverReferenceStream>();
    stream->stream_id = "object:hover:unapplied";
    stream->request_identity = request_identity;
    stream->sequence = 1;
    stream->state = iii_drone_interfaces::msg::ManeuverReferenceStream::STATE_ACTIVE;
    stream->is_valid = true;
    stream->object_tracking_active = true;
    stream->produced_at = fixture.core_node->now();
    stream->valid_until = fixture.core_node->now() + rclcpp::Duration::from_seconds(30.0);
    stream->reference = iii_drone::adapters::ReferenceAdapter(
        iii_drone::control::Reference(iii_drone::types::point_t(1.0F, 0.0F, 2.0F), 0.0)).ToMsg();
    fixture.client->receiveReferenceStream(stream);

    BT::RosActionNode<HoverByObject>::WrappedResult result{};
    result.code = rclcpp_action::ResultCode::SUCCEEDED;
    EXPECT_EQ(action.onResultReceived(result), BT::NodeStatus::FAILURE);
    ASSERT_TRUE(fixture.client->pending_goal_handoff_);
    EXPECT_EQ(fixture.client->pending_goal_handoff_->request_identity, request_identity);
    EXPECT_FALSE(fixture.client->object_stop_);
    EXPECT_FALSE(fixture.client->BeginManeuverGoalHandoff(kRequestB));
}

}  // namespace
