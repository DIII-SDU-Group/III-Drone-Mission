#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

// Exercise the production CustomOperation server and dispatcher directly. The
// source has no public library surface because it is an executable; this guard
// omits only its executable main and the friend below is the narrow test seam.
#define III_DRONE_CUSTOM_OPERATION_TESTING
#include "../src/operations/custom_operation_node.cpp"

namespace {

class CustomOperationModeTestAccess {
public:
    static rclcpp::CallbackGroup::SharedPtr operationServerCallbackGroup(CustomOperationMode & mode) {
        return mode.operation_server_callback_group_;
    }

    static rclcpp_action::GoalResponse offerGoal(
        CustomOperationMode & mode,
        const rclcpp_action::GoalUUID & uuid,
        const std::string & operation
    ) {
        auto goal = std::make_shared<CustomOperation::Goal>();
        goal->operation = operation;
        return mode.handleOperationGoal(uuid, goal);
    }

    static void setBeforeGoalReservationHook(
        CustomOperationMode & mode,
        std::function<void()> hook
    ) {
        mode.test_before_goal_reservation_hook_ = std::move(hook);
    }

    static void setBeforeAcceptedBindHook(
        CustomOperationMode & mode,
        std::function<void(const std::shared_ptr<CustomOperationGoalHandle> &)> hook
    ) {
        mode.test_before_accepted_bind_hook_ = std::move(hook);
    }

    static auto context(CustomOperationMode & mode) {
        std::lock_guard<std::mutex> lock(mode.operation_mutex_);
        return mode.operation_context_;
    }

    static bool scopedClearPending(CustomOperationMode & mode) {
        std::lock_guard<std::mutex> lock(mode.operation_mutex_);
        return mode.scoped_clear_retry_ != nullptr;
    }

    static uint64_t scopedClearAttemptCount(CustomOperationMode & mode) {
        std::lock_guard<std::mutex> lock(mode.operation_mutex_);
        return mode.scoped_clear_retry_ ? mode.scoped_clear_retry_->attempt_id : 0;
    }

    static bool forwardedAccepted(CustomOperationMode & mode) {
        std::lock_guard<std::mutex> lock(mode.operation_mutex_);
        return mode.operation_context_ && mode.operation_context_->forwarded_accepted;
    }

    static rclcpp_action::CancelResponse cancel(
        CustomOperationMode & mode,
        const std::shared_ptr<CustomOperationGoalHandle> & goal_handle
    ) {
        return mode.handleOperationCancel(goal_handle);
    }

    template <typename ContextT>
    static void replayFailedHoverResult(CustomOperationMode & mode, const ContextT & context) {
        using Action = iii_drone_interfaces::action::Hover;
        typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult result;
        result.code = rclcpp_action::ResultCode::ABORTED;
        result.result = std::make_shared<Action::Result>();
        mode.handleForwardedResult<Action>(context, "hover", result);
    }

    template <typename ContextT>
    static std::shared_ptr<CustomOperationGoalHandle> publicGoal(const ContextT & context) {
        return context->operation_goal;
    }

    template <typename ContextT>
    static void replayHoverWatchdog(CustomOperationMode & mode, const ContextT & context) {
        mode.handleGoalResponseTimeout(context, "hover");
    }

    template <typename ContextT>
    static bool replayHoverFeedback(CustomOperationMode & mode, const ContextT & context) {
        return mode.publishOperationFeedback(context, "hover");
    }

    static void allowManeuverDispatch(CustomOperationMode & mode) {
        mode.maneuver_registered_as_offboard_ = true;
    }

    static iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr referenceClient(
        CustomOperationMode & mode
    ) {
        return mode.maneuver_reference_client_;
    }

    static bool abortCurrentOperation(CustomOperationMode & mode, const std::string & reason) {
        return mode.abortCurrentOperation(reason);
    }

    template <typename ContextT>
    static void replayReferenceFailure(CustomOperationMode & mode, const ContextT & context) {
        mode.failManeuverReference(context);
    }

    static void setAfterHandoffPrepareHook(
        CustomOperationMode & mode,
        std::function<void()> hook
    ) {
        mode.test_after_handoff_prepare_hook_ = std::move(hook);
    }

    static void replaySticks(CustomOperationMode & mode, float throttle, float roll = 0.0F) {
        auto sticks = std::make_shared<px4_msgs::msg::ManualControlSetpoint>();
        sticks->valid = true;
        sticks->throttle = throttle;
        sticks->roll = roll;
        sticks->pitch = 0.0F;
        sticks->yaw = 0.0F;
        mode.manualControlSetpointCallback(sticks);
    }

    static bool positionControlTriggered(CustomOperationMode & mode) {
        return mode.manual_position_control_triggered_.load();
    }

    static void seedStationaryOdometry(CustomOperationMode & mode) {
        px4_msgs::msg::VehicleOdometry odometry;
        odometry.pose_frame = iii_drone::adapters::px4::POSE_FRAME_LOCAL_NED;
        odometry.velocity_frame = iii_drone::adapters::px4::VELOCITY_FRAME_LOCAL_NED;
        odometry.q[0] = 1.0F;
        mode.vehicle_odometry_history_->Store(
            iii_drone::adapters::px4::VehicleOdometryAdapter(odometry)
        );
    }
};

using namespace std::chrono_literals;
using Stream = iii_drone_interfaces::msg::ManeuverReferenceStream;
using Ack = iii_drone_interfaces::msg::ManeuverReferenceAck;

constexpr char kPredecessorIdentity[] = "mri1-00000000000000010000000000000001-0000000000000001";

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

template <typename PredicateT>
bool spinUntil(
    rclcpp::executors::MultiThreadedExecutor & executor,
    PredicateT && predicate,
    std::chrono::milliseconds timeout = 1500ms
) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(2ms);
    }
    executor.spin_some();
    return predicate();
}

template <typename PredicateT>
bool waitWithoutSpinning(
    PredicateT && predicate,
    std::chrono::milliseconds timeout = 1500ms
) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(2ms);
    }
    return predicate();
}

std::vector<rclcpp::Parameter> referenceClientParameters() {
    const auto integer = [](const std::string & name, int value = 100) {
        return rclcpp::Parameter(name, value);
    };
    const auto decimal = [](const std::string & name, double value = 1.0) {
        return rclcpp::Parameter(name, value);
    };
    return {
        decimal("/control/dt", 0.02),
        integer("/mission/get_reference_timeout_ms"),
        integer("/mission/reference_loss_timeout_ms", 1000),
        integer("/mission/reference_rebase_timeout_ms", 1000),
        decimal("/control/maneuver_controller/minimum_target_altitude", 0.5),
        integer("/control/maneuver_controller/maneuver_execution_period_ms"),
        integer("/control/maneuver_controller/reference_stream_timeout_ms", 1000),
        decimal("/mission/reference_continuity_position_tolerance_m"),
        decimal("/mission/reference_continuity_velocity_tolerance_m_s"),
        decimal("/mission/reference_continuity_acceleration_tolerance_m_s2"),
        decimal("/mission/reference_continuity_yaw_tolerance_rad"),
        decimal("/mission/reference_continuity_yaw_rate_tolerance_rad_s"),
        decimal("/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2"),
        decimal("/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2"),
        decimal("/control/maneuver_controller/controlled_cancel_max_jerk_m_s3"),
        decimal("/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2"),
        decimal("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3"),
        decimal("/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", 0.1),
        decimal("/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", 0.1),
        decimal("/control/maneuver_controller/controlled_cancel_settle_time_s", 0.2),
        rclcpp::Parameter("/mission/use_nans_when_hovering", false),
        integer("/mission/max_failed_attempts_during_maneuver", 10),
        integer("/mission/wait_for_maneuver_start_timeout_ms", 1000),
        decimal("/mission/manual_stick_input_threshold", 1.0),
    };
}

// The tests do not inherit III_SYSTEM_PROFILE: every fixture names its profile.
std::vector<rclcpp::Parameter> modeParameters(const std::string & runtime_profile) {
    auto parameters = referenceClientParameters();
    parameters.emplace_back(iii_drone::mission::kRuntimeProfileParameter, runtime_profile);
    return parameters;
}

template <typename ActionT>
class ManeuverActionRecorder {
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    ManeuverActionRecorder(const rclcpp::Node::SharedPtr & node, const std::string & name)
    : node_(node), callback_group_(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {
        server_ = rclcpp_action::create_server<ActionT>(
            node_,
            std::string(kManeuverNamespace) + "/" + name,
            [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal> goal) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    request_identities_.push_back(goal->request_identity);
                }
                received_goal_.store(true);
                received_cv_.notify_all();
                if (hold_goal_response_.load()) {
                    std::unique_lock<std::mutex> lock(response_mutex_);
                    response_cv_.wait(lock, [this] { return release_goal_response_.load(); });
                }
                return accept_goal_.load()
                    ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                    : rclcpp_action::GoalResponse::REJECT;
            },
            [](const std::shared_ptr<GoalHandle>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandle> goal_handle) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    handles_.push_back(goal_handle);
                }
                accepted_cv_.notify_all();
            },
            rcl_action_server_get_default_options(),
            callback_group_
        );
    }

    ~ManeuverActionRecorder() {
        releaseGoalResponse();
    }

    void setAcceptGoal(bool value) { accept_goal_.store(value); }
    void holdGoalResponse() { hold_goal_response_.store(true); }
    void releaseGoalResponse() {
        release_goal_response_.store(true);
        response_cv_.notify_all();
    }

    bool receivedGoal() const { return received_goal_.load(); }

    bool hasAcceptedGoal() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !handles_.empty();
    }

    std::vector<std::string> requestIdentities() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return request_identities_;
    }

    void succeedLatest(bool success = true) {
        const auto handle = latestAcceptedHandle();
        ASSERT_NE(handle, nullptr);
        auto result = std::make_shared<typename ActionT::Result>();
        if constexpr (requires { result->success; }) {
            result->success = success;
        }
        handle->succeed(result);
    }

    void completeLatestAfterCancel() {
        const auto handle = latestAcceptedHandle();
        ASSERT_NE(handle, nullptr);
        auto result = std::make_shared<typename ActionT::Result>();
        if constexpr (requires { result->success; }) {
            result->success = false;
        }
        if (handle->is_canceling()) {
            handle->canceled(result);
        } else {
            handle->abort(result);
        }
    }

    void abortLatest() {
        const auto handle = latestAcceptedHandle();
        ASSERT_NE(handle, nullptr);
        auto result = std::make_shared<typename ActionT::Result>();
        if constexpr (requires { result->success; }) {
            result->success = false;
        }
        handle->abort(result);
    }

private:
    // receivedGoal() turns true in the goal callback, before the action
    // server runs the accepted callback that stores the handle; a background
    // executor can still be between the two when a test completes the goal.
    std::shared_ptr<GoalHandle> latestAcceptedHandle() {
        std::unique_lock<std::mutex> lock(mutex_);
        accepted_cv_.wait_for(lock, 2s, [this] { return !handles_.empty(); });
        return handles_.empty() ? nullptr : handles_.back();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp_action::Server<ActionT>::SharedPtr server_;
    mutable std::mutex mutex_;
    std::condition_variable accepted_cv_;
    std::vector<std::string> request_identities_;
    std::vector<std::shared_ptr<GoalHandle>> handles_;
    std::atomic_bool accept_goal_{true};
    std::atomic_bool hold_goal_response_{false};
    std::atomic_bool release_goal_response_{false};
    std::atomic_bool received_goal_{false};
    std::condition_variable received_cv_;
    std::mutex response_mutex_;
    std::condition_variable response_cv_;
};

class ScopedClearRecorder {
public:
    explicit ScopedClearRecorder(const rclcpp::Node::SharedPtr & node) : node_(node) {
        startService();
    }

    ~ScopedClearRecorder() { release(); }

    void setAvailable(bool available) {
        if (available) {
            startService();
        } else {
            service_.reset();
        }
    }
    void failNextResponse() { failures_remaining_.store(1); }
    void holdResponse() { holdResponseStartingAt(1); }
    void holdResponseStartingAt(std::size_t request_index) {
        hold_response_from_request_.store(request_index);
    }
    void release() {
        release_response_.store(true);
        response_cv_.notify_all();
    }
    bool entered() const { return entered_.load(); }
    std::vector<std::string> requestIdentities() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return request_identities_;
    }

private:
    void startService() {
        if (service_) {
            return;
        }
        service_ = node_->create_service<iii_drone_interfaces::srv::ClearManeuverQueue>(
            std::string(kManeuverNamespace) + "/clear_maneuver_queue",
            [this](
                const std::shared_ptr<iii_drone_interfaces::srv::ClearManeuverQueue::Request> request,
                std::shared_ptr<iii_drone_interfaces::srv::ClearManeuverQueue::Response> response
            ) {
                std::size_t request_index;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    request_identities_.push_back(request->request_identity);
                    request_index = request_identities_.size();
                }
                entered_.store(true);
                if (request_index >= hold_response_from_request_.load()) {
                    std::unique_lock<std::mutex> lock(response_mutex_);
                    response_cv_.wait(lock, [this] { return release_response_.load(); });
                }
                response->success = failures_remaining_.fetch_sub(1) <= 0;
                response->cleared_count = 0;
            }
        );
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Service<iii_drone_interfaces::srv::ClearManeuverQueue>::SharedPtr service_;
    mutable std::mutex mutex_;
    std::vector<std::string> request_identities_;
    std::atomic_size_t hold_response_from_request_{std::numeric_limits<std::size_t>::max()};
    std::atomic_bool release_response_{false};
    std::atomic_bool entered_{false};
    std::atomic_int failures_remaining_{0};
    std::mutex response_mutex_;
    std::condition_variable response_cv_;
};

struct Fixture {
    explicit Fixture(const std::string & runtime_profile = "sim")
    : mode_node(std::make_shared<rclcpp::Node>(
          "custom_operation_identity_mode",
          rclcpp::NodeOptions().parameter_overrides(modeParameters(runtime_profile))
      )),
      client_node(std::make_shared<rclcpp::Node>("custom_operation_identity_client")),
      maneuver_node(std::make_shared<rclcpp::Node>("custom_operation_identity_maneuvers")),
      clear_node(std::make_shared<rclcpp::Node>("custom_operation_identity_clear")),
      mode(*mode_node),
      fly_to_position(maneuver_node, "fly_to_position"),
      follow_waypoint_path(maneuver_node, "follow_waypoint_path"),
      cable_aware_fly_to_position(maneuver_node, "cable_aware_fly_to_position"),
      fly_to_object(maneuver_node, "fly_to_object"),
      hover(maneuver_node, "hover"),
      hover_by_object(maneuver_node, "hover_by_object"),
      hover_on_cable(maneuver_node, "hover_on_cable"),
      cable_landing(maneuver_node, "cable_landing"),
      cable_takeoff(maneuver_node, "cable_takeoff"),
      clear_recorder(clear_node) {
        registration_service = maneuver_node->create_service<iii_drone_interfaces::srv::RegisterOffboardMode>(
            std::string(kManeuverNamespace) + "/register_offboard_mode",
            [](
                const std::shared_ptr<iii_drone_interfaces::srv::RegisterOffboardMode::Request>,
                std::shared_ptr<iii_drone_interfaces::srv::RegisterOffboardMode::Response>
            ) {}
        );
        terminal_hold_transfer_service =
            maneuver_node->create_service<iii_drone_interfaces::srv::TerminalHoldTransfer>(
                std::string(kManeuverNamespace) + "/terminal_hold_transfer",
                [](
                    const std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Request>,
                    std::shared_ptr<iii_drone_interfaces::srv::TerminalHoldTransfer::Response> response
                ) {
                    response->accepted = false;
                    response->reason = "no retained terminal hold";
                }
            );
        custom_client = rclcpp_action::create_client<CustomOperation>(
            client_node,
            std::string(kOperationNamespace) + "/run_operation"
        );
        rclcpp::QoS stream_qos(rclcpp::KeepLast(5));
        stream_qos.reliable().durability_volatile();
        stream_qos.deadline(200ms);
        stream_qos.lifespan(1000ms);
        stream_publisher = client_node->create_publisher<Stream>(
            std::string(kManeuverNamespace) + "/reference_stream", stream_qos
        );
        ack_subscription = client_node->create_subscription<Ack>(
            std::string(kManeuverNamespace) + "/reference_ack",
            rclcpp::QoS(10).reliable(),
            [this](const Ack::SharedPtr ack) {
                std::lock_guard<std::mutex> lock(ack_mutex);
                acknowledgements.push_back(*ack);
            }
        );
        executor.add_node(mode_node);
        executor.add_callback_group(
            mode.operationCallbackGroup(), mode_node->get_node_base_interface()
        );
        executor.add_node(client_node);
        executor.add_node(maneuver_node);
        executor.add_node(clear_node);
        mode.onActivate();
        CustomOperationModeTestAccess::allowManeuverDispatch(mode);
        CustomOperationModeTestAccess::seedStationaryOdometry(mode);
    }

    ~Fixture() {
        clear_recorder.release();
        fly_to_position.releaseGoalResponse();
        follow_waypoint_path.releaseGoalResponse();
        cable_aware_fly_to_position.releaseGoalResponse();
        fly_to_object.releaseGoalResponse();
        hover.releaseGoalResponse();
        hover_by_object.releaseGoalResponse();
        hover_on_cable.releaseGoalResponse();
        cable_landing.releaseGoalResponse();
        cable_takeoff.releaseGoalResponse();
        executor.cancel();
        if (background_spinner_.joinable()) {
            background_spinner_.join();
        }
        executor.remove_node(clear_node);
        executor.remove_node(maneuver_node);
        executor.remove_node(client_node);
        executor.remove_node(mode_node);
    }

    std::shared_future<rclcpp_action::ClientGoalHandle<CustomOperation>::SharedPtr> send(
        const std::string & operation,
        const std::string & arguments = "{}"
    ) {
        CustomOperation::Goal goal;
        goal.operation = operation;
        goal.arguments_json = arguments;
        return custom_client->async_send_goal(goal);
    }

    bool waitForServer() {
        return custom_client->wait_for_action_server(1s);
    }

    void startBackgroundSpin() {
        ASSERT_FALSE(background_spinner_.joinable());
        background_spinner_ = std::thread([this] { executor.spin(); });
    }

    bool observedAck(const std::string & stream_id) const {
        std::lock_guard<std::mutex> lock(ack_mutex);
        for (const auto & ack : acknowledgements) {
            if (ack.stream_id == stream_id && ack.last_applied_sequence == 1) {
                return true;
            }
        }
        return false;
    }

    rclcpp::Node::SharedPtr mode_node;
    rclcpp::Node::SharedPtr client_node;
    rclcpp::Node::SharedPtr maneuver_node;
    rclcpp::Node::SharedPtr clear_node;
    rclcpp::Service<iii_drone_interfaces::srv::TerminalHoldTransfer>::SharedPtr
        terminal_hold_transfer_service;
    CustomOperationMode mode;
    rclcpp_action::Client<CustomOperation>::SharedPtr custom_client;
    rclcpp::Publisher<Stream>::SharedPtr stream_publisher;
    rclcpp::Subscription<Ack>::SharedPtr ack_subscription;
    mutable std::mutex ack_mutex;
    std::vector<Ack> acknowledgements;
    ManeuverActionRecorder<iii_drone_interfaces::action::FlyToPosition> fly_to_position;
    ManeuverActionRecorder<iii_drone_interfaces::action::FollowWaypointPath> follow_waypoint_path;
    ManeuverActionRecorder<iii_drone_interfaces::action::CableAwareFlyToPosition> cable_aware_fly_to_position;
    ManeuverActionRecorder<iii_drone_interfaces::action::FlyToObject> fly_to_object;
    ManeuverActionRecorder<iii_drone_interfaces::action::Hover> hover;
    ManeuverActionRecorder<iii_drone_interfaces::action::HoverByObject> hover_by_object;
    ManeuverActionRecorder<iii_drone_interfaces::action::HoverOnCable> hover_on_cable;
    ManeuverActionRecorder<iii_drone_interfaces::action::CableLanding> cable_landing;
    ManeuverActionRecorder<iii_drone_interfaces::action::CableTakeoff> cable_takeoff;
    ScopedClearRecorder clear_recorder;
    rclcpp::Service<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedPtr registration_service;
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions(), 4};
    std::thread background_spinner_;
};

template <typename RecorderT>
void assertStampedAndComplete(
    Fixture & fixture,
    const std::string & operation,
    RecorderT & recorder,
    const std::string & args = "{}"
) {
    auto outer_goal = fixture.send(operation, args);
    ASSERT_TRUE(waitWithoutSpinning([&] { return outer_goal.wait_for(0ms) == std::future_status::ready; }));
    ASSERT_NE(outer_goal.get(), nullptr);
    ASSERT_TRUE(waitWithoutSpinning([&] { return recorder.receivedGoal(); }));
    const auto identities = recorder.requestIdentities();
    ASSERT_EQ(identities.size(), 1U);
    EXPECT_TRUE(iii_drone::control::maneuver::isValidManeuverRequestIdentity(identities.front()));
    recorder.succeedLatest();
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
}

// HIL soak run 21: in a Reentrant group, a result request handled while
// rclcpp was accepting the goal (response sent before the goal is registered)
// was answered STATUS_UNKNOWN and the client saw its ingress goal finish.
TEST(CustomOperationIdentity, OperationServerHandlesItsRequestsOneAtATime) {
    RclcppContext context;
    Fixture fixture;
    const auto group = CustomOperationModeTestAccess::operationServerCallbackGroup(fixture.mode);
    ASSERT_NE(group, nullptr);
    EXPECT_EQ(group->type(), rclcpp::CallbackGroupType::MutuallyExclusive);
    EXPECT_NE(group, fixture.mode.operationCallbackGroup());
}

TEST(CustomOperationIdentity, AllProductionDispatchersStampAValidIdentity) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    // Completion now queries Core synchronously for a retained terminal
    // stream; keep the fake Core service spinning on a separate worker.
    fixture.startBackgroundSpin();

    assertStampedAndComplete(fixture, "fly_to_position", fixture.fly_to_position, "{x: 1, y: 2, z: 3}");
    assertStampedAndComplete(fixture, "follow_waypoint_path", fixture.follow_waypoint_path, "{waypoints: [{x: 1, y: 2, z: 3}]}");
    assertStampedAndComplete(fixture, "cable_aware_fly_to_position", fixture.cable_aware_fly_to_position, "{x: 1, y: 2, z: 3}");
    assertStampedAndComplete(fixture, "fly_to_object", fixture.fly_to_object);
    assertStampedAndComplete(fixture, "hover", fixture.hover);
    assertStampedAndComplete(fixture, "hover_by_object", fixture.hover_by_object);
    assertStampedAndComplete(fixture, "hover_on_cable", fixture.hover_on_cable);
    assertStampedAndComplete(fixture, "cable_landing", fixture.cable_landing);
    assertStampedAndComplete(fixture, "cable_takeoff", fixture.cable_takeoff);
}

TEST(CustomOperationIdentity, OptiTrackRejectsOperationsOutsideItsAllowlistBeforeDispatch) {
    RclcppContext context;
    Fixture fixture("opti_track");
    ASSERT_TRUE(fixture.waitForServer());
    fixture.startBackgroundSpin();

    const std::vector<std::string> rejected = {
        "cable_aware_fly_to_position",
        "fly_to_object",
        "hover_by_object",
        "hover_on_cable",
        "cable_landing",
        "cable_takeoff",
    };
    for (const auto & operation : rejected) {
        auto outer_goal = fixture.send(operation, "{x: 1, y: 2, z: 3}");
        ASSERT_TRUE(waitWithoutSpinning([&] { return outer_goal.wait_for(0ms) == std::future_status::ready; }));
        EXPECT_EQ(outer_goal.get(), nullptr) << operation;
        EXPECT_EQ(
            fixture.mode.lastRejectionReason(),
            "custom operation " + operation + " is not available in the opti_track profile"
        );
        EXPECT_FALSE(fixture.mode.operationActive());
    }
    EXPECT_FALSE(fixture.cable_aware_fly_to_position.receivedGoal());
    EXPECT_FALSE(fixture.fly_to_object.receivedGoal());
    EXPECT_FALSE(fixture.hover_by_object.receivedGoal());
    EXPECT_FALSE(fixture.hover_on_cable.receivedGoal());
    EXPECT_FALSE(fixture.cable_landing.receivedGoal());
    EXPECT_FALSE(fixture.cable_takeoff.receivedGoal());

    assertStampedAndComplete(fixture, "hover", fixture.hover);
    assertStampedAndComplete(fixture, "fly_to_position", fixture.fly_to_position, "{x: 1, y: 2, z: 3}");
    assertStampedAndComplete(fixture, "follow_waypoint_path", fixture.follow_waypoint_path, "{waypoints: [{x: 1, y: 2, z: 3}]}");
}

TEST(CustomOperationIdentity, OnlyStickMovementSinceActivationTriggersPositionControl) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.mode_node->set_parameter(
        rclcpp::Parameter("/mission/manual_stick_input_threshold", 0.35)).successful);
    // The throttle rests at the bottom (PX4 reports -1) while the mode activates.
    fixture.mode.onDeactivate();
    CustomOperationModeTestAccess::replaySticks(fixture.mode, -1.0F);
    fixture.mode.onActivate();
    for (int sample = 0; sample < 20; ++sample) {
        CustomOperationModeTestAccess::replaySticks(fixture.mode, -1.0F);
    }
    CustomOperationModeTestAccess::replaySticks(fixture.mode, -1.0F, 0.3F);
    EXPECT_FALSE(CustomOperationModeTestAccess::positionControlTriggered(fixture.mode));

    CustomOperationModeTestAccess::replaySticks(fixture.mode, -1.0F, 0.5F);
    EXPECT_TRUE(CustomOperationModeTestAccess::positionControlTriggered(fixture.mode));
}

TEST(CustomOperationIdentity, DeactivationClosesAdmissionBeforeValidatedGoalCanReserve) {
    RclcppContext context;
    Fixture fixture;
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool reached_reservation = false;
    bool release_reservation = false;
    CustomOperationModeTestAccess::setBeforeGoalReservationHook(fixture.mode, [&] {
        std::unique_lock<std::mutex> lock(gate_mutex);
        reached_reservation = true;
        gate_cv.notify_all();
        gate_cv.wait_for(lock, 2s, [&] { return release_reservation; });
    });

    rclcpp_action::GoalUUID uuid{};
    uuid[0] = 1;
    std::atomic<rclcpp_action::GoalResponse> response{rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE};
    std::thread offered([&] {
        response.store(CustomOperationModeTestAccess::offerGoal(fixture.mode, uuid, "hover"));
    });
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        EXPECT_TRUE(gate_cv.wait_for(lock, 2s, [&] { return reached_reservation; }));
    }
    fixture.mode.onDeactivate();
    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_reservation = true;
    }
    gate_cv.notify_all();
    offered.join();
    EXPECT_EQ(response.load(), rclcpp_action::GoalResponse::REJECT);
    EXPECT_FALSE(fixture.mode.operationActive());

    CustomOperationModeTestAccess::setBeforeGoalReservationHook(fixture.mode, {});
    fixture.mode.onActivate();
    uuid[0] = 2;
    EXPECT_EQ(
        CustomOperationModeTestAccess::offerGoal(fixture.mode, uuid, "hover"),
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
    );
    fixture.mode.onDeactivate();
    EXPECT_FALSE(fixture.mode.operationActive());
}

TEST(CustomOperationIdentity, ConcurrentGoalHandlersReserveAtMostOneUuid) {
    RclcppContext context;
    Fixture fixture;
    std::atomic_bool release{false};
    std::atomic_int ready{0};
    rclcpp_action::GoalUUID first_uuid{};
    rclcpp_action::GoalUUID second_uuid{};
    first_uuid[0] = 1;
    second_uuid[0] = 2;
    auto first = rclcpp_action::GoalResponse::REJECT;
    auto second = rclcpp_action::GoalResponse::REJECT;
    auto offer = [&](const rclcpp_action::GoalUUID & uuid, rclcpp_action::GoalResponse & response) {
        ready.fetch_add(1);
        while (!release.load()) {
            std::this_thread::yield();
        }
        response = CustomOperationModeTestAccess::offerGoal(fixture.mode, uuid, "hover");
    };
    std::thread first_thread(offer, std::cref(first_uuid), std::ref(first));
    std::thread second_thread(offer, std::cref(second_uuid), std::ref(second));
    EXPECT_TRUE(waitWithoutSpinning([&] { return ready.load() == 2; }));
    release.store(true);
    first_thread.join();
    second_thread.join();
    EXPECT_NE(first, second);
    EXPECT_TRUE(fixture.mode.operationActive());
    fixture.mode.onDeactivate();
    EXPECT_FALSE(fixture.mode.operationActive());
}

TEST(CustomOperationIdentity, CancellationBeforeAcceptedBindBelongsToReservedGoal) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.hover.holdGoalResponse();
    fixture.startBackgroundSpin();
    std::atomic<rclcpp_action::CancelResponse> cancel_response{rclcpp_action::CancelResponse::REJECT};
    CustomOperationModeTestAccess::setBeforeAcceptedBindHook(fixture.mode, [&](const auto & handle) {
        cancel_response.store(CustomOperationModeTestAccess::cancel(fixture.mode, handle));
    });

    auto outer_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    EXPECT_EQ(cancel_response.load(), rclcpp_action::CancelResponse::ACCEPT);
    EXPECT_TRUE(fixture.mode.operationActive());
    fixture.hover.releaseGoalResponse();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.hasAcceptedGoal(); }));
    fixture.hover.completeLatestAfterCancel();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)outer_goal;
}

TEST(CustomOperationIdentity, RetiredReservationRejectsLateAcceptedCallbackWithoutTouchingSuccessor) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.startBackgroundSpin();
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool first_entered = false;
    bool release_first = false;
    std::atomic_bool intercepted{false};
    CustomOperationModeTestAccess::setBeforeAcceptedBindHook(fixture.mode, [&](const auto &) {
        if (intercepted.exchange(true)) {
            return;
        }
        std::unique_lock<std::mutex> lock(gate_mutex);
        first_entered = true;
        gate_cv.notify_all();
        gate_cv.wait_for(lock, 2s, [&] { return release_first; });
    });

    auto first_goal = fixture.send("hover");
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        EXPECT_TRUE(gate_cv.wait_for(lock, 2s, [&] { return first_entered; }));
    }
    ASSERT_TRUE(fixture.mode.operationActive());
    fixture.mode.onDeactivate();
    EXPECT_FALSE(fixture.mode.operationActive());
    fixture.mode.onActivate();
    auto second_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    // The server handles its requests one at a time: the successor's goal
    // waits for the retired reservation's late accepted callback.
    EXPECT_FALSE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }, 300ms));
    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_first = true;
    }
    gate_cv.notify_all();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    EXPECT_EQ(fixture.mode.activeOperation(), "fly_to_position");
    EXPECT_FALSE(fixture.hover.receivedGoal());
    fixture.fly_to_position.succeedLatest();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)first_goal;
    (void)second_goal;
}

TEST(CustomOperationIdentity, RejectedInnerGoalRetiresIdentityAndAllowsSuccessor) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.hover.setAcceptGoal(false);
    auto rejected = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return fixture.hover.receivedGoal(); }));
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return !fixture.mode.operationActive(); }));
    EXPECT_EQ(fixture.hover.requestIdentities().size(), 1U);
    fixture.hover.setAcceptGoal(true);
    auto successor = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] {
        return fixture.hover.requestIdentities().size() == 2U;
    }));
    fixture.hover.succeedLatest();
    EXPECT_TRUE(spinUntil(fixture.executor, [&] { return !fixture.mode.operationActive(); }));
    (void)rejected;
    (void)successor;
}

TEST(CustomOperationIdentity, DispatchExceptionRetiresPreparedIdentityWithoutSending) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    std::atomic_bool prepared{false};
    CustomOperationModeTestAccess::setAfterHandoffPrepareHook(fixture.mode, [&] {
        prepared.store(true);
        throw std::runtime_error("injected dispatch failure");
    });
    auto failed = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return prepared.load(); }));
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return !fixture.mode.operationActive(); }));
    EXPECT_FALSE(fixture.hover.receivedGoal());
    CustomOperationModeTestAccess::setAfterHandoffPrepareHook(fixture.mode, {});
    auto successor = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return fixture.hover.receivedGoal(); }));
    fixture.hover.succeedLatest();
    EXPECT_TRUE(spinUntil(fixture.executor, [&] { return !fixture.mode.operationActive(); }));
    (void)failed;
    (void)successor;
}

TEST(CustomOperationIdentity, FailedConfirmCancelsOwnForwardedGoalAndRetiresIdentity) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.hover.holdGoalResponse();
    fixture.startBackgroundSpin();
    auto failed = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    const auto identity = fixture.hover.requestIdentities().front();
    ASSERT_TRUE(CustomOperationModeTestAccess::referenceClient(fixture.mode)->CancelManeuverGoalHandoff(identity));
    fixture.hover.releaseGoalResponse();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.hasAcceptedGoal(); }));
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    auto successor = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    fixture.fly_to_position.succeedLatest();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)failed;
    (void)successor;
}

TEST(CustomOperationIdentity, ResponseTimeoutRetiresAAndLateAcceptanceCannotStopB) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.hover.holdGoalResponse();
    fixture.startBackgroundSpin();
    auto first_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }, 4500ms));
    auto second_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    fixture.hover.releaseGoalResponse();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.hasAcceptedGoal(); }));
    EXPECT_EQ(fixture.mode.activeOperation(), "fly_to_position");
    fixture.fly_to_position.succeedLatest();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)first_goal;
    (void)second_goal;
}

TEST(CustomOperationIdentity, EarlyReferenceIsAcknowledgedBeforeLateGoalAcceptance) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.fly_to_position.holdGoalResponse();
    fixture.startBackgroundSpin();

    const auto finite_reference = iii_drone::control::Reference(
        iii_drone::types::point_t::Zero(),
        0.0,
        iii_drone::types::vector_t::Zero(),
        0.0,
        iii_drone::types::vector_t::Zero(),
        0.0
    );
    auto reference_client = CustomOperationModeTestAccess::referenceClient(fixture.mode);
    ASSERT_TRUE(reference_client->BeginManeuverGoalHandoff(kPredecessorIdentity));
    ASSERT_TRUE(reference_client->ConfirmManeuverGoalHandoff(kPredecessorIdentity));
    Stream predecessor;
    predecessor.stream_id = "custom-operation:g1";
    predecessor.request_identity = kPredecessorIdentity;
    predecessor.sequence = 1;
    predecessor.state = Stream::STATE_ACTIVE;
    predecessor.is_valid = true;
    predecessor.produced_at = fixture.client_node->now();
    predecessor.valid_until = fixture.client_node->now() + rclcpp::Duration::from_seconds(10.0);
    predecessor.reference = iii_drone::adapters::ReferenceAdapter(finite_reference).ToMsg();
    fixture.stream_publisher->publish(predecessor);
    std::this_thread::sleep_for(50ms);
    ASSERT_TRUE(waitWithoutSpinning([&] {
        reference_client->GetReference(0.02, [] {});
        return fixture.observedAck(predecessor.stream_id);
    }));

    auto outer_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    const auto identities = fixture.fly_to_position.requestIdentities();
    ASSERT_EQ(identities.size(), 1U);

    Stream stream;
    stream.stream_id = "custom-operation:g2";
    stream.request_identity = identities.front();
    stream.sequence = 1;
    stream.state = Stream::STATE_ACTIVE;
    stream.is_valid = true;
    stream.produced_at = fixture.client_node->now();
    stream.valid_until = fixture.client_node->now() + rclcpp::Duration::from_seconds(10.0);
    stream.reference = iii_drone::adapters::ReferenceAdapter(finite_reference).ToMsg();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return fixture.stream_publisher->get_subscription_count() > 0;
    }));
    fixture.stream_publisher->publish(stream);
    ASSERT_TRUE(waitWithoutSpinning([&] {
        reference_client->GetReference(0.02, [] {});
        return fixture.observedAck(stream.stream_id);
    }));

    fixture.fly_to_position.releaseGoalResponse();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.hasAcceptedGoal(); }));
    ASSERT_TRUE(waitWithoutSpinning([&] { return outer_goal.wait_for(0ms) == std::future_status::ready; }));
    ASSERT_NE(outer_goal.get(), nullptr);
    fixture.fly_to_position.succeedLatest();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
}

TEST(CustomOperationIdentity, CancelBeforeInnerAcceptanceRetainsEarlyReferenceUntilResult) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.fly_to_position.holdGoalResponse();
    fixture.startBackgroundSpin();

    const auto finite_reference = iii_drone::control::Reference(
        iii_drone::types::point_t::Zero(), 0.0,
        iii_drone::types::vector_t::Zero(), 0.0,
        iii_drone::types::vector_t::Zero(), 0.0
    );
    auto reference_client = CustomOperationModeTestAccess::referenceClient(fixture.mode);
    ASSERT_TRUE(reference_client->BeginManeuverGoalHandoff(kPredecessorIdentity));
    ASSERT_TRUE(reference_client->ConfirmManeuverGoalHandoff(kPredecessorIdentity));
    Stream predecessor;
    predecessor.stream_id = "custom-operation:cancel-predecessor";
    predecessor.request_identity = kPredecessorIdentity;
    predecessor.sequence = 1;
    predecessor.state = Stream::STATE_ACTIVE;
    predecessor.is_valid = true;
    predecessor.produced_at = fixture.client_node->now();
    predecessor.valid_until = fixture.client_node->now() + rclcpp::Duration::from_seconds(10.0);
    predecessor.reference = iii_drone::adapters::ReferenceAdapter(finite_reference).ToMsg();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return fixture.stream_publisher->get_subscription_count() > 0;
    }));
    ASSERT_TRUE(waitWithoutSpinning([&] {
        fixture.stream_publisher->publish(predecessor);
        reference_client->GetReference(0.02, [] {});
        return fixture.observedAck(predecessor.stream_id);
    }));

    auto outer_goal_future = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return outer_goal_future.wait_for(0ms) == std::future_status::ready;
    }));
    auto outer_goal = outer_goal_future.get();
    ASSERT_NE(outer_goal, nullptr);
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    const auto identity = fixture.fly_to_position.requestIdentities().front();
    auto cancel_future = fixture.custom_client->async_cancel_goal(outer_goal);
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return cancel_future.wait_for(0ms) == std::future_status::ready;
    }));
    EXPECT_TRUE(fixture.mode.operationActive());

    Stream stream;
    stream.stream_id = "custom-operation:cancel-before-acceptance";
    stream.request_identity = identity;
    stream.sequence = 1;
    stream.state = Stream::STATE_ACTIVE;
    stream.is_valid = true;
    stream.produced_at = fixture.client_node->now();
    stream.valid_until = fixture.client_node->now() + rclcpp::Duration::from_seconds(10.0);
    stream.reference = iii_drone::adapters::ReferenceAdapter(finite_reference).ToMsg();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return fixture.stream_publisher->get_subscription_count() > 0;
    }));
    fixture.stream_publisher->publish(stream);
    ASSERT_TRUE(waitWithoutSpinning([&] {
        fixture.stream_publisher->publish(stream);
        reference_client->GetReference(0.02, [] {});
        return fixture.observedAck(stream.stream_id);
    }));
    EXPECT_TRUE(fixture.mode.operationActive());

    fixture.fly_to_position.releaseGoalResponse();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.hasAcceptedGoal(); }));
    fixture.fly_to_position.completeLatestAfterCancel();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    EXPECT_FALSE(reference_client->IsManeuverActive());
}

TEST(CustomOperationIdentity, NoReferenceSuccessRetiresEarlyConsumedActiveIdentity) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.hover.holdGoalResponse();
    fixture.startBackgroundSpin();

    const auto finite_reference = iii_drone::control::Reference(
        iii_drone::types::point_t::Zero(), 0.0,
        iii_drone::types::vector_t::Zero(), 0.0,
        iii_drone::types::vector_t::Zero(), 0.0
    );
    auto reference_client = CustomOperationModeTestAccess::referenceClient(fixture.mode);
    ASSERT_TRUE(reference_client->BeginManeuverGoalHandoff(kPredecessorIdentity));
    ASSERT_TRUE(reference_client->ConfirmManeuverGoalHandoff(kPredecessorIdentity));
    Stream predecessor;
    predecessor.stream_id = "custom-operation:no-reference-predecessor";
    predecessor.request_identity = kPredecessorIdentity;
    predecessor.sequence = 1;
    predecessor.state = Stream::STATE_ACTIVE;
    predecessor.is_valid = true;
    predecessor.produced_at = fixture.client_node->now();
    predecessor.valid_until = fixture.client_node->now() + rclcpp::Duration::from_seconds(10.0);
    predecessor.reference = iii_drone::adapters::ReferenceAdapter(finite_reference).ToMsg();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return fixture.stream_publisher->get_subscription_count() > 0;
    }));
    ASSERT_TRUE(waitWithoutSpinning([&] {
        fixture.stream_publisher->publish(predecessor);
        reference_client->GetReference(0.02, [] {});
        return fixture.observedAck(predecessor.stream_id);
    }));

    auto outer_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    const auto identity = fixture.hover.requestIdentities().front();
    Stream successor = predecessor;
    successor.stream_id = "custom-operation:no-reference-successor";
    successor.request_identity = identity;
    successor.produced_at = fixture.client_node->now();
    successor.valid_until = fixture.client_node->now() + rclcpp::Duration::from_seconds(10.0);
    ASSERT_TRUE(waitWithoutSpinning([&] {
        fixture.stream_publisher->publish(successor);
        reference_client->GetReference(0.02, [] {});
        return fixture.observedAck(successor.stream_id);
    }));
    fixture.hover.releaseGoalResponse();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return CustomOperationModeTestAccess::forwardedAccepted(fixture.mode);
    }));
    fixture.hover.succeedLatest();
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    EXPECT_FALSE(reference_client->IsManeuverActive());
    (void)outer_goal;
}

TEST(CustomOperationIdentity, LateResponseCancelAndResultForRetiredALeaveBActive) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.hover.holdGoalResponse();
    fixture.startBackgroundSpin();

    auto first_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    const auto first_context = CustomOperationModeTestAccess::context(fixture.mode);
    ASSERT_NE(first_context, nullptr);
    const auto first_public_goal = CustomOperationModeTestAccess::publicGoal(first_context);
    ASSERT_NE(first_public_goal, nullptr);
    ASSERT_TRUE(CustomOperationModeTestAccess::abortCurrentOperation(fixture.mode, "retire A"));
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));

    auto second_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    EXPECT_EQ(
        CustomOperationModeTestAccess::cancel(fixture.mode, first_public_goal),
        rclcpp_action::CancelResponse::REJECT
    );
    fixture.hover.releaseGoalResponse();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.hasAcceptedGoal(); }));
    EXPECT_EQ(fixture.mode.activeOperation(), "fly_to_position");
    CustomOperationModeTestAccess::replayFailedHoverResult(fixture.mode, first_context);
    CustomOperationModeTestAccess::replayHoverWatchdog(fixture.mode, first_context);
    CustomOperationModeTestAccess::replayReferenceFailure(fixture.mode, first_context);
    EXPECT_FALSE(CustomOperationModeTestAccess::replayHoverFeedback(fixture.mode, first_context));
    EXPECT_EQ(fixture.mode.activeOperation(), "fly_to_position");

    fixture.fly_to_position.succeedLatest();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)first_goal;
    (void)second_goal;
}

TEST(CustomOperationIdentity, FailedBeginSendsNoInnerGoal) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    const std::string occupied = kPredecessorIdentity;
    ASSERT_TRUE(CustomOperationModeTestAccess::referenceClient(fixture.mode)->BeginManeuverGoalHandoff(occupied));

    auto outer_goal = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return outer_goal.wait_for(0ms) == std::future_status::ready; }));
    ASSERT_NE(outer_goal.get(), nullptr);
    EXPECT_FALSE(spinUntil(fixture.executor, [&] { return fixture.hover.receivedGoal(); }, 150ms));
    EXPECT_TRUE(spinUntil(fixture.executor, [&] { return !fixture.mode.operationActive(); }));
    EXPECT_TRUE(CustomOperationModeTestAccess::referenceClient(fixture.mode)->CancelManeuverGoalHandoff(occupied));
}

TEST(CustomOperationIdentity, MissingRegistrationServiceRejectsCachedRegistrationBeforeDispatch) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    // Fixture seeds the cached state to model a successful earlier registration;
    // remove the service to model the controller restarting after registration.
    ASSERT_TRUE(fixture.mode.registeredAsOffboardMode());
    fixture.registration_service.reset();

    auto outer_goal = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] {
        return outer_goal.wait_for(0ms) == std::future_status::ready;
    }));
    auto outer_handle = outer_goal.get();
    ASSERT_NE(outer_handle, nullptr);
    auto result_future = fixture.custom_client->async_get_result(outer_handle);

    ASSERT_TRUE(spinUntil(fixture.executor, [&] {
        return result_future.wait_for(0ms) == std::future_status::ready;
    }));
    const auto result = result_future.get();
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    ASSERT_NE(result.result, nullptr);
    EXPECT_FALSE(result.result->success);
    EXPECT_FALSE(spinUntil(fixture.executor, [&] { return fixture.hover.receivedGoal(); }, 150ms));
    EXPECT_FALSE(fixture.mode.operationActive());
    EXPECT_FALSE(fixture.mode.registeredAsOffboardMode());
}

TEST(CustomOperationIdentity, TerminalClaimCannotInterleaveHandoffPreparationAndSend) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    std::promise<void> terminal_started;
    auto terminal_started_future = terminal_started.get_future();
    std::promise<void> terminal_finished;
    auto terminal_finished_future = terminal_finished.get_future();
    std::thread terminal_thread;
    std::atomic_bool launched{false};
    CustomOperationModeTestAccess::setAfterHandoffPrepareHook(fixture.mode, [&] {
        if (launched.exchange(true)) {
            return;
        }
        terminal_thread = std::thread([&] {
            terminal_started.set_value();
            EXPECT_TRUE(CustomOperationModeTestAccess::abortCurrentOperation(
                fixture.mode, "deterministic terminal claimant after handoff prepare"
            ));
            terminal_finished.set_value();
        });
        ASSERT_EQ(terminal_started_future.wait_for(500ms), std::future_status::ready);
    });

    auto first_goal = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return fixture.hover.receivedGoal(); }));
    ASSERT_TRUE(spinUntil(fixture.executor, [&] {
        return terminal_finished_future.wait_for(0ms) == std::future_status::ready;
    }));
    if (terminal_thread.joinable()) {
        terminal_thread.join();
    }
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return !fixture.mode.operationActive(); }));

    auto successor_goal = fixture.send("hover");
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return fixture.hover.requestIdentities().size() == 2U; }));
    fixture.hover.succeedLatest();
    ASSERT_TRUE(spinUntil(fixture.executor, [&] { return !fixture.mode.operationActive(); }));
    ASSERT_TRUE(spinUntil(fixture.executor, [&] {
        return successor_goal.wait_for(0ms) == std::future_status::ready;
    }));
    (void)first_goal;
}

TEST(CustomOperationIdentity, ScopedTerminalClearKeepsAdmissionClosedThenActivationWithoutOwnerDoesNotClear) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.clear_recorder.holdResponse();
    fixture.startBackgroundSpin();

    auto first_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    const auto first_identity = fixture.fly_to_position.requestIdentities().front();
    fixture.fly_to_position.succeedLatest(false);
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.clear_recorder.entered(); }));
    ASSERT_EQ(fixture.clear_recorder.requestIdentities(), std::vector<std::string>{first_identity});

    auto blocked_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return blocked_goal.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(blocked_goal.get(), nullptr);

    fixture.clear_recorder.release();
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    ASSERT_TRUE(waitWithoutSpinning([&] { return first_goal.wait_for(0ms) == std::future_status::ready; }));

    auto second_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    fixture.hover.succeedLatest();
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    ASSERT_TRUE(waitWithoutSpinning([&] { return second_goal.wait_for(0ms) == std::future_status::ready; }));
    const auto clears_before_activation = fixture.clear_recorder.requestIdentities().size();
    fixture.mode.onDeactivate();
    fixture.mode.onActivate();
    EXPECT_EQ(fixture.clear_recorder.requestIdentities().size(), clears_before_activation);
}

TEST(CustomOperationIdentity, FailedScopedClearRetriesAndHoldsAdmissionUntilSuccess) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.clear_recorder.failNextResponse();
    fixture.clear_recorder.holdResponseStartingAt(2);
    fixture.startBackgroundSpin();

    auto first_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    const auto identity = fixture.hover.requestIdentities().front();
    fixture.hover.abortLatest();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return fixture.clear_recorder.requestIdentities().size() >= 2;
    }));
    EXPECT_TRUE(CustomOperationModeTestAccess::scopedClearPending(fixture.mode));
    EXPECT_EQ(
        fixture.clear_recorder.requestIdentities(),
        (std::vector<std::string>{identity, identity})
    );
    auto blocked_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return blocked_goal.wait_for(0ms) == std::future_status::ready;
    }));
    EXPECT_EQ(blocked_goal.get(), nullptr);

    fixture.clear_recorder.release();
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    auto successor = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    fixture.fly_to_position.succeedLatest();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)first_goal;
    (void)successor;
}

TEST(CustomOperationIdentity, UnavailableScopedClearRetriesWithoutReleasingAdmission) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.clear_recorder.setAvailable(false);
    fixture.startBackgroundSpin();

    auto first_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    fixture.hover.abortLatest();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return CustomOperationModeTestAccess::scopedClearPending(fixture.mode);
    }));
    auto blocked_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return blocked_goal.wait_for(0ms) == std::future_status::ready;
    }));
    EXPECT_EQ(blocked_goal.get(), nullptr);
    EXPECT_TRUE(fixture.clear_recorder.requestIdentities().empty());

    fixture.clear_recorder.setAvailable(true);
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }, 3000ms));
    EXPECT_EQ(fixture.clear_recorder.requestIdentities().size(), 1U);
    auto successor = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.fly_to_position.receivedGoal(); }));
    fixture.fly_to_position.succeedLatest();
    EXPECT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)first_goal;
    (void)successor;
}

TEST(CustomOperationIdentity, MissingScopedClearResponseRetriesWithoutOpeningAdmission) {
    RclcppContext context;
    Fixture fixture;
    ASSERT_TRUE(fixture.waitForServer());
    fixture.clear_recorder.holdResponse();
    fixture.startBackgroundSpin();

    auto first_goal = fixture.send("hover");
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.hover.receivedGoal(); }));
    fixture.hover.abortLatest();
    ASSERT_TRUE(waitWithoutSpinning([&] { return fixture.clear_recorder.entered(); }));
    ASSERT_TRUE(CustomOperationModeTestAccess::scopedClearPending(fixture.mode));
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return CustomOperationModeTestAccess::scopedClearAttemptCount(fixture.mode) >= 2;
    }, 2500ms));
    auto blocked_goal = fixture.send("fly_to_position", "{x: 1, y: 2, z: 3}");
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return blocked_goal.wait_for(0ms) == std::future_status::ready;
    }));
    EXPECT_EQ(blocked_goal.get(), nullptr);

    fixture.clear_recorder.release();
    ASSERT_TRUE(waitWithoutSpinning([&] {
        return fixture.clear_recorder.requestIdentities().size() >= 2;
    }));
    ASSERT_TRUE(waitWithoutSpinning([&] { return !fixture.mode.operationActive(); }));
    (void)first_goal;
}

}  // namespace
