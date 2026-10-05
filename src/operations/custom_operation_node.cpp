#include <chrono>
#include <cstdint>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <iii_drone_configuration/configurator.hpp>
#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>
#include <iii_drone_core/adapters/reference_adapter.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>
#include <iii_drone_core/control/maneuver/maneuver_request_identity.hpp>
#include <iii_drone_core/control/reference.hpp>
#include <iii_drone_core/utils/history.hpp>
#include <iii_drone_interfaces/action/cable_aware_fly_to_position.hpp>
#include <iii_drone_interfaces/action/cable_landing.hpp>
#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/action/custom_operation.hpp>
#include <iii_drone_interfaces/action/fly_to_object.hpp>
#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/action/follow_waypoint_path.hpp>
#include <iii_drone_interfaces/action/hover.hpp>
#include <iii_drone_interfaces/action/hover_by_object.hpp>
#include <iii_drone_interfaces/action/hover_on_cable.hpp>
#include <iii_drone_interfaces/msg/custom_operation_mode_status.hpp>
#include <iii_drone_interfaces/msg/string_stamped.hpp>
#include <iii_drone_interfaces/msg/target.hpp>
#include <iii_drone_interfaces/srv/clear_maneuver_queue.hpp>
#include <iii_drone_interfaces/srv/register_offboard_mode.hpp>
#include <iii_drone_mission/mission/profile_restrictions.hpp>
#include <iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_ros2/components/mode.hpp>
#include <yaml-cpp/yaml.h>
#include <iii_drone_core/utils/multi_threaded_executor.hpp>

namespace {

constexpr auto kOperationNamespace = "/mission/custom_operation";
constexpr auto kManeuverNamespace = "/control/maneuver_controller";
constexpr auto kManualControlSetpointTopic = "/fmu/out/manual_control_setpoint";
constexpr auto kVehicleCommandTopic = "/fmu/in/vehicle_command";
constexpr auto kVehicleOdometryTopic = "/fmu/out/vehicle_odometry";
constexpr auto kVehicleStatusTopic = "/fmu/out/vehicle_status_v1";
using CustomOperation = iii_drone_interfaces::action::CustomOperation;
using CustomOperationGoalHandle = rclcpp_action::ServerGoalHandle<CustomOperation>;
using VehicleOdometryHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;
using ManeuverReferenceClient = iii_drone::control::maneuver::ManeuverReferenceClient;

class OperationArgs {
public:
    explicit OperationArgs(const std::string & json) {
        try {
            root_ = json.empty() ? YAML::Node(YAML::NodeType::Map) : YAML::Load(json);
        } catch (const YAML::Exception & error) {
            throw std::runtime_error(std::string("invalid arguments_json: ") + error.what());
        }
        if (!root_ || !root_.IsMap()) {
            throw std::runtime_error("arguments_json must be an object");
        }
    }

    std::string stringValue(const std::string & key, const std::string & fallback) const {
        const auto node = root_[key];
        if (!node) {
            return fallback;
        }
        try {
            return node.as<std::string>();
        } catch (const YAML::Exception & error) {
            throw std::runtime_error("argument '" + key + "' must be a string: " + error.what());
        }
    }

    double doubleValue(const std::string & key, double fallback) const {
        const auto node = root_[key];
        if (!node) {
            return fallback;
        }
        try {
            return node.as<double>();
        } catch (const YAML::Exception & error) {
            throw std::runtime_error("argument '" + key + "' must be numeric: " + error.what());
        }
    }

    int intValue(const std::string & key, int fallback) const {
        const auto node = root_[key];
        if (!node) {
            return fallback;
        }
        try {
            return node.as<int>();
        } catch (const YAML::Exception & error) {
            throw std::runtime_error("argument '" + key + "' must be an integer: " + error.what());
        }
    }

    bool boolValue(const std::string & key, bool fallback) const {
        const auto node = root_[key];
        if (!node) {
            return fallback;
        }
        try {
            return node.as<bool>();
        } catch (const YAML::Exception & error) {
            throw std::runtime_error("argument '" + key + "' must be boolean: " + error.what());
        }
    }

    std::vector<iii_drone_interfaces::msg::Waypoint> waypoints() const {
        const auto nodes = root_["waypoints"];
        if (!nodes || !nodes.IsSequence() || nodes.size() < 1) {
            throw std::runtime_error("argument 'waypoints' must be a non-empty array");
        }

        std::vector<iii_drone_interfaces::msg::Waypoint> result;
        result.reserve(nodes.size());
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            const auto node = nodes[index];
            if (!node.IsMap()) {
                throw std::runtime_error("waypoints[" + std::to_string(index) + "] must be an object");
            }
            const auto position = node["position"] && node["position"].IsMap()
                ? node["position"]
                : node;
            try {
                iii_drone_interfaces::msg::Waypoint waypoint;
                waypoint.position.x = position["x"].as<double>();
                waypoint.position.y = position["y"].as<double>();
                waypoint.position.z = position["z"].as<double>();
                waypoint.yaw = node["yaw"] ? node["yaw"].as<float>() : 0.0F;
                waypoint.transition_mode = node["transition_mode"]
                    ? node["transition_mode"].as<uint8_t>()
                    : iii_drone_interfaces::msg::Waypoint::TRANSITION_BLEND;
                waypoint.blend_radius_m = node["blend_radius_m"]
                    ? node["blend_radius_m"].as<float>()
                    : 0.0F;
                waypoint.speed_limit_m_s = node["speed_limit_m_s"]
                    ? node["speed_limit_m_s"].as<float>()
                    : 0.0F;
                result.push_back(waypoint);
            } catch (const YAML::Exception & error) {
                throw std::runtime_error(
                    "invalid waypoints[" + std::to_string(index) + "]: " + error.what()
                );
            }
        }
        return result;
    }

    double nestedDouble(
        const std::initializer_list<std::string> & path,
        const std::string & flat_key,
        double fallback
    ) const {
        auto node = nested(path);
        if (node) {
            try {
                return node.as<double>();
            } catch (const YAML::Exception & error) {
                if (root_[flat_key]) {
                    return doubleValue(flat_key, fallback);
                }
                try {
                    return std::stod(node.Scalar());
                } catch (const std::exception &) {
                    return fallback;
                }
            }
        }
        return doubleValue(flat_key, fallback);
    }

private:
    YAML::Node root_;

    YAML::Node nested(const std::initializer_list<std::string> & path) const {
        YAML::Node node = root_;
        for (const auto & key : path) {
            if (!node || !node.IsMap()) {
                return YAML::Node();
            }
            node = node[key];
        }
        return node;
    }

    static std::string pathName(const std::initializer_list<std::string> & path) {
        std::ostringstream out;
        bool first = true;
        for (const auto & key : path) {
            if (!first) {
                out << ".";
            }
            out << key;
            first = false;
        }
        return out.str();
    }
};

geometry_msgs::msg::Point pointFromArgs(const OperationArgs & args) {
    geometry_msgs::msg::Point point;
    point.x = args.doubleValue("x", 0.0);
    point.y = args.doubleValue("y", 0.0);
    point.z = args.doubleValue("z", 0.0);
    return point;
}

template <typename ResultT>
bool resultSucceeded(const std::shared_ptr<ResultT> & result) {
    if (!result) {
        return false;
    }
    if constexpr (requires { result->success; }) {
        return result->success;
    }
    return true;
}

template <typename ResultT>
std::optional<iii_drone::control::Reference> resultReference(const std::shared_ptr<ResultT> & result) {
    if (!result) {
        return std::nullopt;
    }
    if constexpr (requires { result->target_reference; }) {
        return iii_drone::adapters::ReferenceAdapter(result->target_reference).reference().CopyWithNans();
    }
    return std::nullopt;
}

template <typename ResultT>
std::string resultJson(const std::shared_ptr<ResultT> & result) {
    std::ostringstream out;
    out << "{\"success\":" << (resultSucceeded(result) ? "true" : "false");
    if constexpr (requires { result->target_reference; }) {
        if (result) {
            out << ",\"target_reference\":{"
                << "\"x\":" << result->target_reference.position.x
                << ",\"y\":" << result->target_reference.position.y
                << ",\"z\":" << result->target_reference.position.z
                << ",\"yaw\":" << result->target_reference.yaw
                << "}";
        }
    }
    out << "}";
    return out.str();
}

class CustomOperationModeTestAccess;

class CustomOperationMode : public px4_ros2::ModeBase {
public:
    explicit CustomOperationMode(rclcpp::Node & node)
    : px4_ros2::ModeBase(
          node,
          px4_ros2::ModeBase::Settings(
              "Custom Operation",
              true
          ),
          "/"
      ),
      node_(node),
      trajectory_setpoint_(std::make_shared<iii_drone::px4::TrajectorySetpoint>(*this)),
      vehicle_odometry_history_(std::make_shared<VehicleOdometryHistory>(2)) {

        configureReferenceClient();
        configureRuntimeProfile();

        vehicle_odometry_sub_ = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
            kVehicleOdometryTopic,
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                vehicle_odometry_history_->Store(iii_drone::adapters::px4::VehicleOdometryAdapter(*msg));
            }
        );

        // A HIL SITL instance intentionally has its own PX4 system ID.  Keep
        // commands bound to the identity observed on DDS instead of assuming
        // the physical-aircraft default (1).
        vehicle_status_sub_ = node.create_subscription<px4_msgs::msg::VehicleStatus>(
            kVehicleStatusTopic,
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                if (msg->system_id != 0) {
                    vehicle_system_id_.store(msg->system_id);
                }
                if (msg->component_id != 0) {
                    vehicle_component_id_.store(msg->component_id);
                }
                if (msg->timestamp != 0) {
                    vehicle_timestamp_.store(msg->timestamp);
                }
            }
        );

        rclcpp::QoS px4_manual_qos(rclcpp::KeepLast(1));
        px4_manual_qos.transient_local();
        px4_manual_qos.best_effort();
        manual_control_setpoint_sub_ = node.create_subscription<px4_msgs::msg::ManualControlSetpoint>(
            kManualControlSetpointTopic,
            px4_manual_qos,
            [this](const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {
                manualControlSetpointCallback(msg);
            }
        );

        vehicle_command_pub_ = node.create_publisher<px4_msgs::msg::VehicleCommand>(
            kVehicleCommandTopic,
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()
        );

        clear_maneuver_queue_client_ = node.create_client<iii_drone_interfaces::srv::ClearManeuverQueue>(
            std::string(kManeuverNamespace) + "/clear_maneuver_queue",
            rclcpp::ServicesQoS()
        );

        register_offboard_mode_client_ = node.create_client<iii_drone_interfaces::srv::RegisterOffboardMode>(
            std::string(kManeuverNamespace) + "/register_offboard_mode",
            rclcpp::ServicesQoS()
        );

        operation_callback_group_ = node.create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
        // The run_operation server handles its goal, cancel and result requests
        // one at a time. rclcpp_action (Jazzy) sends the goal response before it
        // registers the goal, so a result request handled concurrently in that
        // window is answered STATUS_UNKNOWN and the client sees its accepted goal
        // finish (HIL soak run 21: the ingress "failed" 5 ms after acceptance).
        operation_server_callback_group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        fly_to_position_client_ = createManeuverClient<iii_drone_interfaces::action::FlyToPosition>("fly_to_position");
        follow_waypoint_path_client_ = createManeuverClient<iii_drone_interfaces::action::FollowWaypointPath>("follow_waypoint_path");
        cable_aware_fly_to_position_client_ = createManeuverClient<iii_drone_interfaces::action::CableAwareFlyToPosition>("cable_aware_fly_to_position");
        fly_to_object_client_ = createManeuverClient<iii_drone_interfaces::action::FlyToObject>("fly_to_object");
        hover_client_ = createManeuverClient<iii_drone_interfaces::action::Hover>("hover");
        hover_by_object_client_ = createManeuverClient<iii_drone_interfaces::action::HoverByObject>("hover_by_object");
        hover_on_cable_client_ = createManeuverClient<iii_drone_interfaces::action::HoverOnCable>("hover_on_cable");
        cable_landing_client_ = createManeuverClient<iii_drone_interfaces::action::CableLanding>("cable_landing");
        cable_takeoff_client_ = createManeuverClient<iii_drone_interfaces::action::CableTakeoff>("cable_takeoff");

        operation_server_ = rclcpp_action::create_server<CustomOperation>(
            &node_,
            std::string(kOperationNamespace) + "/run_operation",
            [this](
                const rclcpp_action::GoalUUID & goal_uuid,
                std::shared_ptr<const CustomOperation::Goal> goal
            ) {
                return handleOperationGoal(goal_uuid, goal);
            },
            [this](const std::shared_ptr<CustomOperationGoalHandle> goal_handle) {
                return handleOperationCancel(goal_handle);
            },
            [this](const std::shared_ptr<CustomOperationGoalHandle> goal_handle) {
                handleOperationAccepted(goal_handle);
            },
            rcl_action_server_get_default_options(),
            operation_server_callback_group_
        );
    }

    void onActivate() override {
        RCLCPP_INFO(node_.get_logger(), "CustomOperationMode::onActivate(): Activating.");
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            active_.store(false);
        }
        manual_position_control_triggered_.store(false);
        runShutdownStep("retire stale operation", [this]() {
            abortCurrentOperation("CustomOperation mode activated while a prior operation was still owned");
        });
        maneuver_reference_client_->SetReferenceModeHover(true);
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            active_.store(true);
        }
    }

    void onDeactivate() override {
        RCLCPP_INFO(node_.get_logger(), "CustomOperationMode::onDeactivate(): Deactivating.");
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            active_.store(false);
        }
        runShutdownStep("retire active operation", [this]() {
            abortCurrentOperation("CustomOperation mode deactivated");
        });
        maneuver_reference_client_->SetReferenceModeHover(true);
    }

    void updateSetpoint(float dt) override {
        if (!active_.load()) {
            return;
        }

        ForwardedOperationContextPtr failing_context;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            failing_context = operation_context_;
        }

        const auto reference = maneuver_reference_client_->GetReference(
            dt,
            [this, failing_context]() {
                failManeuverReference(failing_context);
            }
        );
        trajectory_setpoint_->update(reference);
    }

    bool registerAsOffboardMode() {
        maneuver_registered_as_offboard_ = setRegisteredOffboardMode(false);
        return maneuver_registered_as_offboard_;
    }

    bool ensureRegisteredAsOffboardMode() {
        if (!register_offboard_mode_client_->service_is_ready()) {
            // Registration lives in maneuver-controller memory. A cached true
            // value is stale when that service disappears (for example after
            // a controller restart), so never dispatch on it.
            maneuver_registered_as_offboard_ = false;
            return false;
        }

        // Maneuver-controller registration is held in controller memory. Refresh it
        // idempotently so CustomOperation recovers after maneuver_controller restarts.
        maneuver_registered_as_offboard_ = setRegisteredOffboardMode(false);
        return maneuver_registered_as_offboard_;
    }

    bool unregisterAsOffboardMode() {
        maneuver_registered_as_offboard_ = false;
        return setRegisteredOffboardMode(true);
    }

    bool active() const {
        return active_.load();
    }

    bool registeredAsOffboardMode() const {
        return maneuver_registered_as_offboard_;
    }

    bool operationActive() const {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return operation_context_ != nullptr;
    }

    std::string activeOperation() const {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return operation_context_ ? operation_context_->operation : "";
    }

    std::string lastRejectionReason() const {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return last_rejection_reason_;
    }

    rclcpp::CallbackGroup::SharedPtr operationCallbackGroup() const {
        return operation_callback_group_;
    }

private:
    rclcpp::Node & node_;
    std::shared_ptr<iii_drone::px4::TrajectorySetpoint> trajectory_setpoint_;
    std::shared_ptr<VehicleOdometryHistory> vehicle_odometry_history_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::CallbackGroup::SharedPtr get_reference_callback_group_;
    rclcpp::CallbackGroup::SharedPtr operation_callback_group_;
    rclcpp::CallbackGroup::SharedPtr operation_server_callback_group_;
    std::shared_ptr<iii_drone::configuration::Configurator<rclcpp::Node>> configurator_;
    iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client_;

    rclcpp::Client<iii_drone_interfaces::srv::ClearManeuverQueue>::SharedPtr clear_maneuver_queue_client_;
    rclcpp::Client<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedPtr register_offboard_mode_client_;
    rclcpp_action::Server<CustomOperation>::SharedPtr operation_server_;
    rclcpp_action::Client<iii_drone_interfaces::action::FlyToPosition>::SharedPtr fly_to_position_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::FollowWaypointPath>::SharedPtr follow_waypoint_path_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::CableAwareFlyToPosition>::SharedPtr cable_aware_fly_to_position_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::FlyToObject>::SharedPtr fly_to_object_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::Hover>::SharedPtr hover_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::HoverByObject>::SharedPtr hover_by_object_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::HoverOnCable>::SharedPtr hover_on_cable_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::CableLanding>::SharedPtr cable_landing_client_;
    rclcpp_action::Client<iii_drone_interfaces::action::CableTakeoff>::SharedPtr cable_takeoff_client_;

    std::atomic_bool active_{false};
    std::atomic_bool manual_position_control_triggered_{false};
    std::atomic_uint8_t vehicle_system_id_{1};
    std::atomic_uint8_t vehicle_component_id_{1};
    std::atomic_uint64_t vehicle_timestamp_{0};
    bool maneuver_registered_as_offboard_{false};
    std::string runtime_profile_;

    enum class OperationPhase {
        Reserved,
        Forwarding,
        CancellationRequested,
        TerminalInProgress,
        Retired,
    };

    struct ForwardedOperationContext {
        rclcpp_action::GoalUUID goal_uuid{};
        std::shared_ptr<CustomOperationGoalHandle> operation_goal;
        std::string operation;
        std::string request_identity;
        OperationPhase phase{OperationPhase::Reserved};
        bool cancel_requested{false};
        bool handoff_begun{false};
        bool forwarded_accepted{false};
        std::function<void()> cancel_forwarded_goal;
    };

    using ForwardedOperationContextPtr = std::shared_ptr<ForwardedOperationContext>;

    struct TerminalClaim {
        ForwardedOperationContextPtr context;
        std::shared_ptr<CustomOperationGoalHandle> operation_goal;
        std::string request_identity;
        bool handoff_begun = false;
        bool cancel_requested = false;
        std::function<void()> cancel_forwarded_goal;
    };

    struct ScopedClearRetry {
        ForwardedOperationContextPtr context;
        std::string reason;
        std::string request_identity;
        std::function<void()> completion;
        bool in_flight{false};
        uint64_t attempt_id{0};
        int64_t request_id{0};
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::TimerBase::SharedPtr response_timer;
    };

    friend class CustomOperationModeTestAccess;

    mutable std::mutex operation_mutex_;
    ForwardedOperationContextPtr operation_context_;
    std::shared_ptr<ScopedClearRetry> scoped_clear_retry_;
    std::string last_rejection_reason_;

#ifdef III_DRONE_CUSTOM_OPERATION_TESTING
    // The test only hook starts a competing terminal claimant while the
    // preparation/send ownership transaction holds operation_mutex_. It
    // verifies that the claimant cannot retire this context between Begin and
    // async_send_goal(). It is never compiled into the production executable.
    std::function<void()> test_after_handoff_prepare_hook_;
    std::function<void()> test_before_goal_reservation_hook_;
    std::function<void(const std::shared_ptr<CustomOperationGoalHandle> &)> test_before_accepted_bind_hook_;
#endif

    void configureReferenceClient() {
        const auto bool_t = rclcpp::ParameterType::PARAMETER_BOOL;
        const auto int_t = rclcpp::ParameterType::PARAMETER_INTEGER;
        const auto double_t = rclcpp::ParameterType::PARAMETER_DOUBLE;

        configurator_ = std::make_shared<iii_drone::configuration::Configurator<rclcpp::Node>>(
            &node_,
            "custom_operation"
        );
        configurator_->DeclareParameter("/control/dt", double_t);
        configurator_->DeclareParameter("/mission/get_reference_timeout_ms", int_t);
        configurator_->DeclareParameter("/mission/reference_loss_timeout_ms", int_t);
        configurator_->DeclareParameter("/mission/reference_rebase_timeout_ms", int_t);
        configurator_->DeclareParameter("/control/maneuver_controller/minimum_target_altitude", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/maneuver_execution_period_ms", int_t);
        configurator_->DeclareParameter("/control/maneuver_controller/reference_stream_timeout_ms", int_t);
        configurator_->DeclareParameter("/mission/reference_continuity_position_tolerance_m", double_t);
        configurator_->DeclareParameter("/mission/reference_continuity_velocity_tolerance_m_s", double_t);
        configurator_->DeclareParameter("/mission/reference_continuity_acceleration_tolerance_m_s2", double_t);
        configurator_->DeclareParameter("/mission/reference_continuity_yaw_tolerance_rad", double_t);
        configurator_->DeclareParameter("/mission/reference_continuity_yaw_rate_tolerance_rad_s", double_t);
        configurator_->DeclareParameter("/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", double_t);
        configurator_->DeclareParameter("/control/maneuver_controller/controlled_cancel_settle_time_s", double_t);
        configurator_->DeclareParameter("/mission/use_nans_when_hovering", bool_t);
        configurator_->DeclareParameter("/mission/max_failed_attempts_during_maneuver", int_t);
        configurator_->DeclareParameter("/mission/wait_for_maneuver_start_timeout_ms", int_t);
        configurator_->DeclareParameter("/mission/manual_stick_input_threshold", double_t);
        configurator_->CreateConfiguration("maneuver_reference_client", {
            ConfigurationEntry("/mission/use_nans_when_hovering", bool_t),
            ConfigurationEntry("/mission/max_failed_attempts_during_maneuver", int_t),
            ConfigurationEntry("/mission/wait_for_maneuver_start_timeout_ms", int_t),
            ConfigurationEntry("/mission/get_reference_timeout_ms", int_t),
            ConfigurationEntry("/mission/reference_loss_timeout_ms", int_t),
            ConfigurationEntry("/mission/reference_rebase_timeout_ms", int_t),
            ConfigurationEntry("/control/maneuver_controller/minimum_target_altitude", double_t),
            ConfigurationEntry("/control/maneuver_controller/maneuver_execution_period_ms", int_t),
            ConfigurationEntry("/control/maneuver_controller/reference_stream_timeout_ms", int_t),
            ConfigurationEntry("/mission/reference_continuity_position_tolerance_m", double_t),
            ConfigurationEntry("/mission/reference_continuity_velocity_tolerance_m_s", double_t),
            ConfigurationEntry("/mission/reference_continuity_acceleration_tolerance_m_s2", double_t),
            ConfigurationEntry("/mission/reference_continuity_yaw_tolerance_rad", double_t),
            ConfigurationEntry("/mission/reference_continuity_yaw_rate_tolerance_rad_s", double_t),
            ConfigurationEntry("/mission/reference_continuity_yaw_acceleration_tolerance_rad_s2", double_t),
            ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_deceleration_m_s2", double_t),
            ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_jerk_m_s3", double_t),
            ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_yaw_deceleration_rad_s2", double_t),
            ConfigurationEntry("/control/maneuver_controller/controlled_cancel_max_yaw_jerk_rad_s3", double_t),
            ConfigurationEntry("/control/maneuver_controller/controlled_cancel_velocity_threshold_m_s", double_t),
            ConfigurationEntry("/control/maneuver_controller/controlled_cancel_yaw_rate_threshold_rad_s", double_t),
            ConfigurationEntry("/control/maneuver_controller/controlled_cancel_settle_time_s", double_t),
        });

        const double control_dt_s = configurator_->GetParameter("/control/dt").as_double();
        if (control_dt_s <= 0.0) {
            throw std::runtime_error("CustomOperationMode::configureReferenceClient(): /control/dt must be positive");
        }
        setSetpointUpdateRate(static_cast<float>(1.0 / control_dt_s));

        get_reference_callback_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        maneuver_reference_client_ = std::make_shared<iii_drone::control::maneuver::ManeuverReferenceClient>(
            &node_,
            vehicle_odometry_history_,
            configurator_->GetConfiguration("maneuver_reference_client"),
            get_reference_callback_group_
        );
    }

    void configureRuntimeProfile() {
        using iii_drone::mission::kRuntimeProfileParameter;
        if (!node_.has_parameter(kRuntimeProfileParameter)) {
            node_.declare_parameter<std::string>(kRuntimeProfileParameter, "");
        }
        runtime_profile_ = iii_drone::mission::ResolveRuntimeProfile(
            node_.get_parameter(kRuntimeProfileParameter).as_string()
        );
        if (iii_drone::mission::CustomOperationAllowlists().count(runtime_profile_) != 0) {
            RCLCPP_INFO(
                node_.get_logger(),
                "CustomOperationMode::configureRuntimeProfile(): The %s profile restricts custom operations.",
                runtime_profile_.c_str()
            );
        }
    }

    template <typename ActionT>
    typename rclcpp_action::Client<ActionT>::SharedPtr createManeuverClient(const std::string & action_name) {
        return rclcpp_action::create_client<ActionT>(
            &node_,
            std::string(kManeuverNamespace) + "/" + action_name,
            operation_callback_group_
        );
    }

    void manualControlSetpointCallback(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg) {
        if (!active_.load() || manual_position_control_triggered_.load()) {
            return;
        }

        const double threshold = configurator_->GetParameter("/mission/manual_stick_input_threshold").as_double();
        const bool switch_to_position_control =
            std::abs(msg->throttle) > threshold ||
            std::abs(msg->yaw) > threshold ||
            std::abs(msg->roll) > threshold ||
            std::abs(msg->pitch) > threshold;

        if (!switch_to_position_control) {
            return;
        }

        manual_position_control_triggered_.store(true);
        RCLCPP_WARN(
            node_.get_logger(),
            "CustomOperationMode::manualControlSetpointCallback(): Position control triggered by manual input; requesting POSCTL and cancelling active custom operation."
        );

        const bool retired = abortCurrentOperation("manual position control triggered");
        if (!retired) {
            maneuver_reference_client_->SetReferenceModeHover(true);
        }

        publishSetNavStateCommand(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL);
    }

    void publishSetNavStateCommand(uint8_t nav_state) {
        px4_msgs::msg::VehicleCommand command;
        // PX4 validates VehicleCommand timestamps in its boot-time domain.
        // On split-host HIL the Pi's wall clock is deliberately independent of
        // SITL, so use the latest DDS timestamp observed from PX4 itself.
        const auto vehicle_timestamp = vehicle_timestamp_.load();
        command.timestamp = vehicle_timestamp != 0
            ? vehicle_timestamp
            : static_cast<uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);
        command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
        command.param1 = static_cast<float>(nav_state);
        command.target_system = vehicle_system_id_.load();
        command.target_component = vehicle_component_id_.load();
        command.source_system = 1;
        command.source_component = 1;
        command.from_external = true;
        vehicle_command_pub_->publish(command);
    }

    bool isSupportedOperation(const std::string & operation) const {
        return operation == "fly_to_position" ||
            operation == "follow_waypoint_path" ||
            operation == "cable_aware_fly_to_position" ||
            operation == "fly_to_object" ||
            operation == "hover" ||
            operation == "hover_by_object" ||
            operation == "hover_on_cable" ||
            operation == "cable_landing" ||
            operation == "cable_takeoff";
    }

    rclcpp_action::GoalResponse handleOperationGoal(
        const rclcpp_action::GoalUUID & goal_uuid,
        std::shared_ptr<const CustomOperation::Goal> goal
    ) {
        if (!isSupportedOperation(goal->operation)) {
            const std::string reason = "unsupported operation: " + goal->operation;
            {
                std::lock_guard<std::mutex> lock(operation_mutex_);
                last_rejection_reason_ = reason;
            }
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::handleOperationGoal(): Rejecting %s because it is unsupported.", goal->operation.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!iii_drone::mission::CustomOperationAllowedInProfile(goal->operation, runtime_profile_)) {
            const std::string reason = iii_drone::mission::NotAvailableInProfileMessage(
                "custom operation " + goal->operation,
                runtime_profile_
            );
            {
                std::lock_guard<std::mutex> lock(operation_mutex_);
                last_rejection_reason_ = reason;
            }
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::handleOperationGoal(): Rejecting: %s.", reason.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

#ifdef III_DRONE_CUSTOM_OPERATION_TESTING
        if (test_before_goal_reservation_hook_) {
            test_before_goal_reservation_hook_();
        }
#endif
        std::lock_guard<std::mutex> lock(operation_mutex_);
        if (!active_.load()) {
            last_rejection_reason_ = "CustomOperation mode is not active";
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (manual_position_control_triggered_.load()) {
            last_rejection_reason_ = "manual position control handoff is in progress";
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (operation_context_) {
            last_rejection_reason_ = "another custom operation is active or completing cleanup";
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::handleOperationGoal(): Rejecting %s because an operation owns admission.", goal->operation.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        auto context = std::make_shared<ForwardedOperationContext>();
        context->goal_uuid = goal_uuid;
        context->operation = goal->operation;
        context->phase = OperationPhase::Reserved;
        operation_context_ = std::move(context);
        last_rejection_reason_.clear();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleOperationCancel(
        const std::shared_ptr<CustomOperationGoalHandle> goal_handle
    ) {
        std::function<void()> cancel_forwarded_goal;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            if (
                !operation_context_ ||
                operation_context_->goal_uuid != goal_handle->get_goal_id() ||
                operation_context_->phase == OperationPhase::TerminalInProgress ||
                operation_context_->phase == OperationPhase::Retired
            ) {
                return rclcpp_action::CancelResponse::REJECT;
            }
            operation_context_->cancel_requested = true;
            operation_context_->phase = OperationPhase::CancellationRequested;
            cancel_forwarded_goal = operation_context_->cancel_forwarded_goal;
        }
        RCLCPP_INFO(
            node_.get_logger(),
            "CustomOperationMode::handleOperationCancel(): Forwarding cancellation and retaining maneuver reference control until the maneuver stops."
        );
        if (cancel_forwarded_goal) {
            cancel_forwarded_goal();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleOperationAccepted(const std::shared_ptr<CustomOperationGoalHandle> goal_handle) {
#ifdef III_DRONE_CUSTOM_OPERATION_TESTING
        if (test_before_accepted_bind_hook_) {
            test_before_accepted_bind_hook_(goal_handle);
        }
#endif
        ForwardedOperationContextPtr context;
        bool cancel_late_acceptance = false;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            if (
                !operation_context_ ||
                operation_context_->goal_uuid != goal_handle->get_goal_id() ||
                operation_context_->phase == OperationPhase::TerminalInProgress ||
                operation_context_->phase == OperationPhase::Retired
            ) {
                cancel_late_acceptance = true;
            } else {
                context = operation_context_;
                context->operation_goal = goal_handle;
                if (context->phase == OperationPhase::Reserved) {
                    context->phase = OperationPhase::Forwarding;
                }
            }
        }
        if (cancel_late_acceptance) {
            finishOperationGoalAsCancelledOrAborted(goal_handle, "operation retired before acceptance");
            return;
        }

        const auto goal = goal_handle->get_goal();
        const std::string & operation = context->operation;
        RCLCPP_INFO(
            node_.get_logger(),
            "CustomOperationMode::handleOperationAccepted(): Dispatching operation '%s'.",
            operation.c_str()
        );
        try {
            if (operation == "fly_to_position") {
                dispatchTyped<iii_drone_interfaces::action::FlyToPosition>(
                    context,
                    fly_to_position_client_,
                    makeFlyToPositionGoal(goal->arguments_json)
                );
            } else if (operation == "follow_waypoint_path") {
                dispatchTyped<iii_drone_interfaces::action::FollowWaypointPath>(
                    context,
                    follow_waypoint_path_client_,
                    makeFollowWaypointPathGoal(goal->arguments_json)
                );
            } else if (operation == "cable_aware_fly_to_position") {
                dispatchTyped<iii_drone_interfaces::action::CableAwareFlyToPosition>(
                    context,
                    cable_aware_fly_to_position_client_,
                    makeCableAwareFlyToPositionGoal(goal->arguments_json)
                );
            } else if (operation == "fly_to_object") {
                dispatchTyped<iii_drone_interfaces::action::FlyToObject>(
                    context,
                    fly_to_object_client_,
                    makeFlyToObjectGoal(goal->arguments_json)
                );
            } else if (operation == "hover") {
                dispatchTyped<iii_drone_interfaces::action::Hover>(
                    context,
                    hover_client_,
                    makeHoverGoal(goal->arguments_json)
                );
            } else if (operation == "hover_by_object") {
                dispatchTyped<iii_drone_interfaces::action::HoverByObject>(
                    context,
                    hover_by_object_client_,
                    makeHoverByObjectGoal(goal->arguments_json)
                );
            } else if (operation == "hover_on_cable") {
                dispatchTyped<iii_drone_interfaces::action::HoverOnCable>(
                    context,
                    hover_on_cable_client_,
                    makeHoverOnCableGoal(goal->arguments_json)
                );
            } else if (operation == "cable_landing") {
                dispatchTyped<iii_drone_interfaces::action::CableLanding>(
                    context,
                    cable_landing_client_,
                    makeCableLandingGoal(goal->arguments_json)
                );
            } else if (operation == "cable_takeoff") {
                dispatchTyped<iii_drone_interfaces::action::CableTakeoff>(
                    context,
                    cable_takeoff_client_,
                    makeCableTakeoffGoal(goal->arguments_json)
                );
            }
        } catch (const std::exception & error) {
            abortOperation(context, error.what());
        }
    }

    bool contextOwnsOperation(const ForwardedOperationContextPtr & context) const {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return operation_context_ == context &&
            context->phase != OperationPhase::TerminalInProgress &&
            context->phase != OperationPhase::Retired;
    }

    template <typename ActionT>
    void dispatchTyped(
        const ForwardedOperationContextPtr & context,
        typename rclcpp_action::Client<ActionT>::SharedPtr client,
        const typename ActionT::Goal & forwarded_goal
    ) {
        const std::string operation_name = context->operation;
        if (!contextOwnsOperation(context)) {
            return;
        }
        if (!ensureRegisteredAsOffboardMode()) {
            abortOperation(context, "failed to register CustomOperation as maneuver-controller offboard mode before dispatching: " + operation_name);
            return;
        }
        if (!client->wait_for_action_server(std::chrono::seconds(2))) {
            abortOperation(context, "underlying maneuver action unavailable: " + operation_name);
            return;
        }
        auto stamped_goal = forwarded_goal;
        auto goal_response_received = std::make_shared<std::atomic_bool>(false);
        auto goal_response_watchdog = std::make_shared<rclcpp::TimerBase::SharedPtr>();
        *goal_response_watchdog = node_.create_wall_timer(
            std::chrono::seconds(3),
            [this, context, operation_name, goal_response_received, goal_response_watchdog]() {
                if (goal_response_received->exchange(true)) {
                    return;
                }
                if (*goal_response_watchdog) {
                    (*goal_response_watchdog)->cancel();
                }
                handleGoalResponseTimeout(context, operation_name);
            },
            operation_callback_group_
        );

        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        options.goal_response_callback =
            [this, context, client, operation_name, goal_response_received, goal_response_watchdog](
                typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr forwarded_goal_handle
            ) {
                goal_response_received->store(true);
                if (*goal_response_watchdog) {
                    (*goal_response_watchdog)->cancel();
                }
                std::function<void()> cancel_now;
                bool owns_operation = false;
                bool confirmed = false;
                {
                    std::lock_guard<std::mutex> lock(operation_mutex_);
                    owns_operation = operation_context_ == context &&
                        context->phase != OperationPhase::TerminalInProgress &&
                        context->phase != OperationPhase::Retired && active_.load();
                    if (owns_operation && forwarded_goal_handle) {
                        // Confirm is a local reference-client transition. Fence it
                        // with terminal claim so retired A cannot confirm after B
                        // has gained admission.
                        confirmed = maneuver_reference_client_->ConfirmManeuverGoalHandoff(
                            context->request_identity
                        );
                        if (confirmed) {
                            context->forwarded_accepted = true;
                            context->cancel_forwarded_goal = [client, forwarded_goal_handle, operation_name]() {
                                cancelForwardedHandle<ActionT>(client, forwarded_goal_handle, operation_name);
                            };
                            if (context->cancel_requested) {
                                cancel_now = context->cancel_forwarded_goal;
                            }
                        }
                    }
                }
                if (!owns_operation) {
                    cancelForwardedHandle<ActionT>(client, forwarded_goal_handle, operation_name);
                    return;
                }
                if (!forwarded_goal_handle) {
                    abortOperation(context, "underlying maneuver goal rejected: " + operation_name);
                    return;
                }
                if (!confirmed) {
                    cancelForwardedHandle<ActionT>(client, forwarded_goal_handle, operation_name);
                    abortOperation(context, "failed to confirm ManeuverReferenceClient handoff for: " + operation_name);
                    return;
                }
                if (cancel_now) {
                    cancel_now();
                }
            };
        options.feedback_callback =
            [this, context, operation_name](
                typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
                const std::shared_ptr<const typename ActionT::Feedback>
            ) {
                publishOperationFeedback(context, operation_name);
            };
        options.result_callback =
            [this, context, operation_name](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wrapped_result) {
                handleForwardedResult<ActionT>(context, operation_name, wrapped_result);
            };

        bool sent = false;
        std::string dispatch_error;
        try {
            // Begin and async_send_goal form one short ownership transaction.
            // No wait occurs while operation_mutex_ is held. A terminal
            // claimant therefore cannot retire this context after Begin and
            // before the action request has been handed to ROS.
            std::lock_guard<std::mutex> lock(operation_mutex_);
            if (
                operation_context_ != context || !active_.load() ||
                context->phase == OperationPhase::TerminalInProgress ||
                context->phase == OperationPhase::Retired
            ) {
                return;
            }
            context->request_identity = iii_drone::control::maneuver::nextProcessManeuverRequestIdentity();
            stamped_goal.request_identity = context->request_identity;
            if (!maneuver_reference_client_->BeginManeuverGoalHandoff(context->request_identity)) {
                dispatch_error = "failed to authorize maneuver reference handoff before dispatching: " + operation_name;
            } else {
                context->handoff_begun = true;
#ifdef III_DRONE_CUSTOM_OPERATION_TESTING
                if (test_after_handoff_prepare_hook_) {
                    test_after_handoff_prepare_hook_();
                }
#endif
                client->async_send_goal(stamped_goal, options);
                sent = true;
            }
        } catch (const std::exception & error) {
            dispatch_error = std::string("exception while dispatching maneuver ") + operation_name + ": " + error.what();
        }
        if (!sent) {
            abortOperation(
                context,
                dispatch_error.empty()
                    ? "operation retired before underlying maneuver dispatch: " + operation_name
                    : dispatch_error
            );
        }
    }

    template <typename ActionT>
    static void cancelForwardedHandle(
        const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
        const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & goal_handle,
        const std::string & operation_name
    ) {
        if (!goal_handle) {
            return;
        }
        try {
            client->async_cancel_goal(goal_handle);
        } catch (const std::exception &) {
            (void)operation_name;
        }
    }

    void handleGoalResponseTimeout(
        const ForwardedOperationContextPtr & context,
        const std::string & operation_name
    ) {
        abortOperation(context, "timed out waiting for underlying maneuver goal response: " + operation_name);
    }

    bool publishOperationFeedback(
        const ForwardedOperationContextPtr & context,
        const std::string & operation_name
    ) {
        auto feedback = std::make_shared<CustomOperation::Feedback>();
        feedback->operation = operation_name;
        feedback->state = "running";
        feedback->feedback_json = "{}";
        // publish_feedback cannot call back into this server synchronously;
        // the ownership fence excludes terminal claim and successor admission.
        std::lock_guard<std::mutex> lock(operation_mutex_);
        if (operation_context_ != context ||
            context->phase == OperationPhase::TerminalInProgress ||
            context->phase == OperationPhase::Retired || !context->operation_goal) {
            return false;
        }
        context->operation_goal->publish_feedback(feedback);
        return true;
    }

    template <typename ActionT>
    void handleForwardedResult(
        const ForwardedOperationContextPtr & context,
        const std::string & operation_name,
        const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wrapped_result
    ) {
        const auto claim = tryClaimTerminal(context);
        if (!claim) {
            RCLCPP_WARN(
                node_.get_logger(),
                "CustomOperationMode::handleForwardedResult(): Ignoring late result for retired operation '%s'.",
                operation_name.c_str()
            );
            return;
        }

        const bool cancelled = wrapped_result.code == rclcpp_action::ResultCode::CANCELED || claim->cancel_requested;
        const bool succeeded = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && resultSucceeded(wrapped_result.result);
        const auto final_reference = resultReference(wrapped_result.result);
        bool reference_retired = true;
        if (succeeded && !cancelled && final_reference && claim->handoff_begun) {
            if (operation_name == "cable_aware_fly_to_position" ||
                operation_name == "fly_to_position") {
                // The result target is nominal mission metadata. Core owns
                // the live corrected command beyond action completion.
                const auto retention = maneuver_reference_client_->RetainCompletedTerminalHold(
                    claim->request_identity, 500);
                if (retention == ManeuverReferenceClient::TerminalHoldRetention::Retained) {
                    reference_retired = true;
                } else if (retention == ManeuverReferenceClient::TerminalHoldRetention::NoOffer) {
                    reference_retired = maneuver_reference_client_->CompleteManeuverGoalHandoff(
                        claim->request_identity, *final_reference);
                } else {
                    reference_retired = false;
                }
            } else {
                reference_retired = maneuver_reference_client_->CompleteManeuverGoalHandoff(
                    claim->request_identity, *final_reference);
            }
        } else {
            reference_retired = retireHandoffAndStop(*claim);
        }

        auto result = std::make_shared<CustomOperation::Result>();
        result->operation = operation_name;
        result->success = succeeded && !cancelled && reference_retired;
        result->result_json = resultJson(wrapped_result.result);
        result->error = result->success ? "" :
            (cancelled ? "operation cancelled" :
            (succeeded && !reference_retired ? "maneuver reference ownership was lost" : "underlying maneuver action failed"));
        auto complete = [this, claim = *claim, result, cancelled]() {
            if (claim.operation_goal) {
                if (cancelled) {
                    finishOperationGoalAsCancelledOrAborted(claim.operation_goal, result);
                } else if (result->success) {
                    claim.operation_goal->succeed(result);
                } else {
                    claim.operation_goal->abort(result);
                }
            }
            completeTerminal(claim.context);
        };
        if (result->success) {
            complete();
        } else {
            clearManeuverQueueThen(
                "custom operation terminal failure/cancel",
                claim->request_identity,
                claim->context,
                std::move(complete)
            );
        }
    }

    iii_drone_interfaces::action::FlyToPosition::Goal makeFlyToPositionGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::FlyToPosition::Goal goal;
        goal.frame_id = parsed.stringValue("frame_id", "world");
        goal.target_position = pointFromArgs(parsed);
        goal.target_yaw = static_cast<float>(parsed.doubleValue("yaw", 0.0));
        goal.blend_to_next = parsed.boolValue("blend_to_next", false);
        goal.ignore_altitude = parsed.boolValue("ignore_altitude", false);
        return goal;
    }

    iii_drone_interfaces::action::CableAwareFlyToPosition::Goal makeCableAwareFlyToPositionGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::CableAwareFlyToPosition::Goal goal;
        goal.frame_id = parsed.stringValue("frame_id", "world");
        goal.target_position = pointFromArgs(parsed);
        goal.target_yaw = static_cast<float>(parsed.doubleValue("yaw", 0.0));
        goal.ignore_altitude = parsed.boolValue("ignore_altitude", false);
        return goal;
    }

    iii_drone_interfaces::action::FollowWaypointPath::Goal makeFollowWaypointPathGoal(
        const std::string & args
    ) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::FollowWaypointPath::Goal goal;
        goal.frame_id = parsed.stringValue("frame_id", "world");
        goal.waypoints = parsed.waypoints();
        goal.repeat = parsed.boolValue("repeat", false);
        const int repeat_from_index = parsed.intValue("repeat_from_index", 0);
        if (repeat_from_index < 0) {
            throw std::runtime_error("argument 'repeat_from_index' must be non-negative");
        }
        goal.repeat_from_index = static_cast<uint32_t>(repeat_from_index);
        goal.nominal_speed_m_s = static_cast<float>(parsed.doubleValue("nominal_speed_m_s", 0.0));
        goal.max_acceleration_m_s2 = static_cast<float>(
            parsed.doubleValue("max_acceleration_m_s2", 0.0)
        );
        goal.max_jerk_m_s3 = static_cast<float>(parsed.doubleValue("max_jerk_m_s3", 0.0));
        return goal;
    }

    iii_drone_interfaces::msg::Target makeTarget(const OperationArgs & args) {
        iii_drone_interfaces::msg::Target target;
        target.target_type = static_cast<uint8_t>(args.intValue("target_type", iii_drone_interfaces::msg::Target::TARGET_TYPE_CABLE));
        target.target_id = args.intValue("target_id", 0);
        target.reference_frame_id = args.stringValue("reference_frame_id", "world");
        target.target_transform.translation.x = args.nestedDouble({"target_transform", "translation", "x"}, "target_transform_translation_x", 0.0);
        target.target_transform.translation.y = args.nestedDouble({"target_transform", "translation", "y"}, "target_transform_translation_y", 0.0);
        target.target_transform.translation.z = args.nestedDouble({"target_transform", "translation", "z"}, "target_transform_translation_z", 0.0);
        target.target_transform.rotation.x = args.nestedDouble({"target_transform", "rotation", "x"}, "target_transform_rotation_x", 0.0);
        target.target_transform.rotation.y = args.nestedDouble({"target_transform", "rotation", "y"}, "target_transform_rotation_y", 0.0);
        target.target_transform.rotation.z = args.nestedDouble({"target_transform", "rotation", "z"}, "target_transform_rotation_z", 0.0);
        target.target_transform.rotation.w = args.nestedDouble({"target_transform", "rotation", "w"}, "target_transform_rotation_w", 1.0);
        return target;
    }

    iii_drone_interfaces::action::FlyToObject::Goal makeFlyToObjectGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::FlyToObject::Goal goal;
        goal.target = makeTarget(parsed);
        return goal;
    }

    iii_drone_interfaces::action::Hover::Goal makeHoverGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::Hover::Goal goal;
        goal.duration_s = static_cast<float>(parsed.doubleValue("duration_s", 1.0));
        goal.sustain_duration_s = static_cast<float>(parsed.doubleValue("sustain_duration_s", 0.0));
        goal.sustain_action = parsed.boolValue("sustain_action", false);
        return goal;
    }

    iii_drone_interfaces::action::HoverByObject::Goal makeHoverByObjectGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::HoverByObject::Goal goal;
        goal.target = makeTarget(parsed);
        goal.duration_s = static_cast<float>(parsed.doubleValue("duration_s", 1.0));
        goal.sustain_action = parsed.boolValue("sustain_action", false);
        return goal;
    }

    iii_drone_interfaces::action::HoverOnCable::Goal makeHoverOnCableGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::HoverOnCable::Goal goal;
        goal.target_cable_id = parsed.intValue("target_cable_id", 0);
        goal.target_z_velocity = static_cast<float>(parsed.doubleValue("target_z_velocity", 0.0));
        goal.target_yaw_rate = static_cast<float>(parsed.doubleValue("target_yaw_rate", 0.0));
        goal.duration_s = static_cast<float>(parsed.doubleValue("duration_s", 1.0));
        goal.sustain_action = parsed.boolValue("sustain_action", false);
        return goal;
    }

    iii_drone_interfaces::action::CableLanding::Goal makeCableLandingGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::CableLanding::Goal goal;
        goal.target_cable_id = parsed.intValue("target_cable_id", 0);
        return goal;
    }

    iii_drone_interfaces::action::CableTakeoff::Goal makeCableTakeoffGoal(const std::string & args) {
        const OperationArgs parsed(args);
        iii_drone_interfaces::action::CableTakeoff::Goal goal;
        goal.target_cable_id = parsed.intValue("target_cable_id", 0);
        goal.target_cable_distance = static_cast<float>(parsed.doubleValue("target_cable_distance", 0.0));
        return goal;
    }

    std::optional<TerminalClaim> tryClaimTerminal(const ForwardedOperationContextPtr & context) {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        if (
            operation_context_ != context ||
            context->phase == OperationPhase::TerminalInProgress ||
            context->phase == OperationPhase::Retired
        ) {
            return std::nullopt;
        }
        context->phase = OperationPhase::TerminalInProgress;
        return TerminalClaim{
            context,
            context->operation_goal,
            context->request_identity,
            context->handoff_begun,
            context->cancel_requested,
            context->cancel_forwarded_goal,
        };
    }

    void completeTerminal(const ForwardedOperationContextPtr & context) {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        if (operation_context_ == context && context->phase == OperationPhase::TerminalInProgress) {
            context->phase = OperationPhase::Retired;
            operation_context_.reset();
        }
    }

    bool retireHandoffAndStop(const TerminalClaim & claim) {
        // Only the request that completed Begin owns a reference transition.
        // In particular, a failed Begin means another identity already owns
        // the client; forcing hover here would let this failed operation stop
        // that owner. A late terminal after its handoff was already retired
        // must likewise not affect a successor.
        if (!claim.handoff_begun) {
            return true;
        }
        return maneuver_reference_client_->CancelManeuverGoalHandoff(claim.request_identity);
    }

    bool abortOperation(const ForwardedOperationContextPtr & context, const std::string & error) {
        const auto claim = tryClaimTerminal(context);
        if (!claim) {
            return false;
        }
        RCLCPP_ERROR(node_.get_logger(), "CustomOperationMode::abortOperation(): %s", error.c_str());
        if (claim->cancel_forwarded_goal) {
            claim->cancel_forwarded_goal();
        }
        retireHandoffAndStop(*claim);
        auto result = std::make_shared<CustomOperation::Result>();
        result->success = false;
        result->operation = claim->context->operation;
        result->error = error;
        result->result_json = "{}";
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            last_rejection_reason_ = error;
        }
        clearManeuverQueueThen("custom operation aborted", claim->request_identity, claim->context, [this, claim = *claim, result]() {
            if (claim.operation_goal) {
                finishOperationGoalAsCancelledOrAborted(claim.operation_goal, result);
            }
            completeTerminal(claim.context);
        });
        return true;
    }

    bool abortCurrentOperation(const std::string & error) {
        ForwardedOperationContextPtr context;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            context = operation_context_;
        }
        return context && abortOperation(context, error);
    }

    void failManeuverReference(const ForwardedOperationContextPtr & context) {
        RCLCPP_ERROR(
            node_.get_logger(),
            "CustomOperationMode::updateSetpoint(): Maneuver reference unavailable; cancelling the operation that owned the failing read."
        );
        if (context) {
            abortOperation(context, "maneuver reference unavailable");
        }
        // GetReference owns the fallback hover and checks its captured Core
        // ownership epoch after this callback returns.
    }

    void finishOperationGoalAsCancelledOrAborted(
        const std::shared_ptr<CustomOperationGoalHandle> goal_handle,
        const std::string & error
    ) {
        auto result = std::make_shared<CustomOperation::Result>();
        result->success = false;
        result->operation = goal_handle->get_goal()->operation;
        result->error = error;
        result->result_json = "{}";
        finishOperationGoalAsCancelledOrAborted(goal_handle, result);
    }

    void finishOperationGoalAsCancelledOrAborted(
        const std::shared_ptr<CustomOperationGoalHandle> goal_handle,
        const std::shared_ptr<CustomOperation::Result> result
    ) {
        try {
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
            } else {
                goal_handle->abort(result);
            }
        } catch (const std::exception & error) {
            RCLCPP_WARN(
                node_.get_logger(),
                "CustomOperationMode::finishOperationGoalAsCancelledOrAborted(): Ignoring terminal transition error: %s",
                error.what()
            );
        }
    }

    void clearManeuverQueueThen(
        const std::string & reason,
        const std::string & request_identity,
        const ForwardedOperationContextPtr & context,
        std::function<void()> completion
    ) {
        if (!iii_drone::control::maneuver::isValidManeuverRequestIdentity(request_identity)) {
            RCLCPP_ERROR(node_.get_logger(), "CustomOperationMode::clearManeuverQueueThen(): Refusing an automatic clear without an owned request identity.");
            completion();
            return;
        }
        auto retry = std::make_shared<ScopedClearRetry>();
        retry->context = context;
        retry->reason = reason;
        retry->request_identity = request_identity;
        retry->completion = std::move(completion);
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            if (operation_context_ != context || context->phase != OperationPhase::TerminalInProgress) {
                return;
            }
            scoped_clear_retry_ = retry;
        }
        attemptScopedClear(retry);
    }

    void scheduleScopedClearRetry(const std::shared_ptr<ScopedClearRetry> & retry) {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        if (scoped_clear_retry_ != retry || retry->in_flight || retry->timer) {
            return;
        }
        std::weak_ptr<ScopedClearRetry> weak_retry = retry;
        retry->timer = node_.create_wall_timer(
            std::chrono::milliseconds(100),
            [this, weak_retry]() {
                const auto retry = weak_retry.lock();
                if (!retry) {
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(operation_mutex_);
                    if (scoped_clear_retry_ != retry) {
                        return;
                    }
                    retry->timer->cancel();
                    retry->timer.reset();
                }
                attemptScopedClear(retry);
            },
            operation_callback_group_
        );
    }

    void finishScopedClearAttempt(
        const std::shared_ptr<ScopedClearRetry> & retry,
        uint64_t attempt_id,
        bool success
    ) {
        std::function<void()> completion;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            if (scoped_clear_retry_ != retry || retry->attempt_id != attempt_id ||
                operation_context_ != retry->context ||
                retry->context->phase != OperationPhase::TerminalInProgress) {
                return;
            }
            retry->in_flight = false;
            if (retry->response_timer) {
                retry->response_timer->cancel();
                retry->response_timer.reset();
            }
            retry->request_id = 0;
            if (success) {
                if (retry->timer) {
                    retry->timer->cancel();
                    retry->timer.reset();
                }
                completion = std::move(retry->completion);
                scoped_clear_retry_.reset();
            }
        }
        if (completion) {
            completion();
        } else if (!success) {
            scheduleScopedClearRetry(retry);
        }
    }

    void attemptScopedClear(const std::shared_ptr<ScopedClearRetry> & retry) {
        uint64_t attempt_id;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            if (scoped_clear_retry_ != retry || retry->in_flight ||
                operation_context_ != retry->context ||
                retry->context->phase != OperationPhase::TerminalInProgress) {
                return;
            }
            retry->in_flight = true;
            attempt_id = ++retry->attempt_id;
        }
        if (!clear_maneuver_queue_client_->service_is_ready()) {
            finishScopedClearAttempt(retry, attempt_id, false);
            return;
        }
        auto request = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Request>();
        request->reason = retry->reason;
        request->request_identity = retry->request_identity;
        std::optional<int64_t> sent_request_id;
        try {
            const auto sent = clear_maneuver_queue_client_->async_send_request(
                request,
                [this, retry, attempt_id](
                    rclcpp::Client<iii_drone_interfaces::srv::ClearManeuverQueue>::SharedFuture future
                ) {
                    bool success = false;
                    try {
                        success = future.get()->success;
                    } catch (const std::exception & error) {
                        RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::clearManeuverQueueThen(): Clear response failed: %s", error.what());
                    }
                    finishScopedClearAttempt(retry, attempt_id, success);
                }
            );
            sent_request_id = sent.request_id;
            {
                std::lock_guard<std::mutex> lock(operation_mutex_);
                if (scoped_clear_retry_ == retry && retry->in_flight && retry->attempt_id == attempt_id) {
                    retry->request_id = sent.request_id;
                    std::weak_ptr<ScopedClearRetry> weak_retry = retry;
                    retry->response_timer = node_.create_wall_timer(
                        std::chrono::seconds(1),
                        [this, weak_retry, attempt_id]() {
                            const auto retry = weak_retry.lock();
                            if (!retry) {
                                return;
                            }
                            int64_t request_id = 0;
                            {
                                std::lock_guard<std::mutex> lock(operation_mutex_);
                                if (scoped_clear_retry_ != retry || !retry->in_flight ||
                                    retry->attempt_id != attempt_id) {
                                    return;
                                }
                                request_id = retry->request_id;
                                retry->in_flight = false;
                                ++retry->attempt_id;
                                retry->response_timer->cancel();
                                retry->response_timer.reset();
                                retry->request_id = 0;
                            }
                            clear_maneuver_queue_client_->remove_pending_request(request_id);
                            scheduleScopedClearRetry(retry);
                        },
                        operation_callback_group_
                    );
                }
            }
        } catch (const std::exception & error) {
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::clearManeuverQueueThen(): Unable to send scoped clear request: %s", error.what());
            if (sent_request_id) {
                clear_maneuver_queue_client_->remove_pending_request(*sent_request_id);
            }
            finishScopedClearAttempt(retry, attempt_id, false);
        }
    }

    bool setRegisteredOffboardMode(bool deregister) {
        if (!register_offboard_mode_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::setRegisteredOffboardMode(): register_offboard_mode service unavailable.");
            return false;
        }
        auto request = std::make_shared<iii_drone_interfaces::srv::RegisterOffboardMode::Request>();
        request->mode_id = id();
        request->deregister = deregister;
        register_offboard_mode_client_->async_send_request(request);
        return true;
    }

    template <typename CallbackT>
    void runShutdownStep(const char * description, CallbackT && callback) {
        try {
            callback();
        } catch (const std::exception & error) {
            RCLCPP_WARN(
                node_.get_logger(),
                "CustomOperationMode::runShutdownStep(): Ignoring error while trying to %s: %s",
                description,
                error.what()
            );
        }
    }
};

}  // namespace

#ifndef III_DRONE_CUSTOM_OPERATION_TESTING
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "custom_operation",
        "/mission/custom_operation"
    );

    RCLCPP_INFO(
        node->get_logger(),
        "Registering CustomOperation mode; PX4/micro-ROS readiness is enforced by system supervision."
    );
    auto mode = std::make_shared<CustomOperationMode>(*node);

    // ModeBase constructs the PX4 registration endpoints immediately, but DDS
    // matching is asynchronous. If the request is sent before the volatile
    // reply reader has matched PX4's reply writer, PX4 accepts the mode while
    // this process waits forever for the already-sent reply. Allow discovery
    // to settle before the first request; subsequent retry handling remains
    // unchanged for genuine registration failures.
    rclcpp::sleep_for(std::chrono::seconds(10));

    bool registered = false;
    for (int attempt = 1; attempt <= 12; ++attempt) {
        if (mode->doRegister()) {
            registered = true;
            break;
        }
        RCLCPP_WARN(
            node->get_logger(),
            "Failed to register CustomOperation mode with PX4; retrying (%d/12).",
            attempt
        );
        rclcpp::sleep_for(std::chrono::seconds(5));
    }
    if (!registered) {
        RCLCPP_FATAL(node->get_logger(), "Failed to register CustomOperation mode with PX4 after retries.");
        rclcpp::shutdown();
        return 1;
    }

    iii_drone::utils::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_callback_group(
        mode->operationCallbackGroup(),
        node->get_node_base_interface()
    );
    std::thread executor_thread([&executor]() {
        executor.spin();
    });

    auto status_pub = node->create_publisher<iii_drone_interfaces::msg::StringStamped>(
        std::string(kOperationNamespace) + "/status",
        rclcpp::QoS(1).best_effort().transient_local()
    );
    auto typed_status_pub = node->create_publisher<iii_drone_interfaces::msg::CustomOperationModeStatus>(
        std::string(kOperationNamespace) + "/mode_status",
        rclcpp::QoS(1).best_effort().transient_local()
    );
    std::atomic_bool publish_status{true};
    std::thread status_thread([node, mode, status_pub, typed_status_pub, &publish_status]() {
        rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
        while (publish_status.load()) {
            iii_drone_interfaces::msg::StringStamped msg;
            msg.stamp = system_clock.now();
            msg.data = std::string("{\"mode_id\":") + std::to_string(mode->id()) +
                ",\"active\":" + (mode->active() ? "true" : "false") + "}";
            status_pub->publish(msg);

            iii_drone_interfaces::msg::CustomOperationModeStatus typed_msg;
            typed_msg.stamp = system_clock.now();
            typed_msg.operation_active = mode->operationActive();
            typed_msg.active_operation = mode->activeOperation();
            typed_msg.custom_operation_modes_registered = mode->registeredAsOffboardMode();
            typed_msg.required_modes = {"custom_operation"};
            if (typed_msg.custom_operation_modes_registered) {
                typed_msg.registered_modes = {"custom_operation"};
            }
            typed_msg.owned_mode = "CustomOperation";
            typed_msg.control_owner = mode->active() ? "custom_operation" : "unknown";
            typed_msg.cancel_available = typed_msg.operation_active;
            typed_msg.ready = mode->active() && typed_msg.custom_operation_modes_registered && !typed_msg.operation_active;
            typed_msg.degraded = !typed_msg.custom_operation_modes_registered;
            if (typed_msg.degraded) {
                typed_msg.degraded_reason = "CustomOperation mode is not registered as maneuver-controller offboard mode";
                typed_msg.degraded_reasons.push_back(typed_msg.degraded_reason);
            }
            const auto rejection_reason = mode->lastRejectionReason();
            if (!rejection_reason.empty()) {
                typed_msg.degraded_reasons.push_back(rejection_reason);
            }
            if (typed_msg.operation_active) {
                typed_msg.operation_state = iii_drone_interfaces::msg::CustomOperationModeStatus::OPERATION_STATE_ACTIVE;
                typed_msg.operation_state_label = "active";
            } else if (mode->active()) {
                typed_msg.operation_state = iii_drone_interfaces::msg::CustomOperationModeStatus::OPERATION_STATE_READY;
                typed_msg.operation_state_label = "ready";
            } else {
                typed_msg.operation_state = iii_drone_interfaces::msg::CustomOperationModeStatus::OPERATION_STATE_IDLE;
                typed_msg.operation_state_label = "idle";
            }
            typed_status_pub->publish(typed_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });

    if (!mode->registerAsOffboardMode()) {
        RCLCPP_WARN(node->get_logger(), "CustomOperation mode was not registered as maneuver-controller offboard mode.");
    }

    auto maneuver_registration_timer = node->create_wall_timer(std::chrono::seconds(2), [mode]() {
        mode->ensureRegisteredAsOffboardMode();
    });

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    publish_status.store(false);
    if (status_thread.joinable()) {
        status_thread.join();
    }
    mode->unregisterAsOffboardMode();
    mode->doUnregister();
    executor.cancel();
    if (executor_thread.joinable()) {
        executor_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
#endif
