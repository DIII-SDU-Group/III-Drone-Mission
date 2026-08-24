#include <chrono>
#include <atomic>
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
#include <iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <px4_ros2/components/mode.hpp>
#include <yaml-cpp/yaml.h>

namespace {

constexpr auto kOperationNamespace = "/mission/custom_operation";
constexpr auto kManeuverNamespace = "/control/maneuver_controller";
constexpr auto kManualControlSetpointTopic = "/fmu/out/manual_control_setpoint";
constexpr auto kVehicleCommandTopic = "/fmu/in/vehicle_command";
constexpr auto kVehicleOdometryTopic = "/fmu/out/vehicle_odometry";
using CustomOperation = iii_drone_interfaces::action::CustomOperation;
using CustomOperationGoalHandle = rclcpp_action::ServerGoalHandle<CustomOperation>;
using VehicleOdometryHistory = iii_drone::utils::History<iii_drone::adapters::px4::VehicleOdometryAdapter>;
using ConfigurationEntry = iii_drone::configuration::configuration_entry_t;

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

        vehicle_odometry_sub_ = node.create_subscription<px4_msgs::msg::VehicleOdometry>(
            kVehicleOdometryTopic,
            rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                vehicle_odometry_history_->Store(iii_drone::adapters::px4::VehicleOdometryAdapter(*msg));
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
                const rclcpp_action::GoalUUID &,
                std::shared_ptr<const CustomOperation::Goal> goal
            ) {
                return handleOperationGoal(goal);
            },
            [this](const std::shared_ptr<CustomOperationGoalHandle> goal_handle) {
                return handleOperationCancel(goal_handle);
            },
            [this](const std::shared_ptr<CustomOperationGoalHandle> goal_handle) {
                handleOperationAccepted(goal_handle);
            },
            rcl_action_server_get_default_options(),
            operation_callback_group_
        );
    }

    void onActivate() override {
        RCLCPP_INFO(node_.get_logger(), "CustomOperationMode::onActivate(): Activating.");
        active_.store(false);
        manual_position_control_triggered_.store(false);
        runShutdownStep("cancel stale forwarded goal", [this]() { cancelForwardedGoal(); });
        runShutdownStep("finish stale operation as canceled", [this]() { finishOperationAsCanceled(); });
        runShutdownStep("clear maneuver queue", [this]() { clearManeuverQueue("custom operation activated"); });
        runShutdownStep("set hover reference", [this]() { maneuver_reference_client_->SetReferenceModeHover(true); });
        active_.store(true);
    }

    void onDeactivate() override {
        RCLCPP_INFO(node_.get_logger(), "CustomOperationMode::onDeactivate(): Deactivating.");
        active_.store(false);
        runShutdownStep("cancel forwarded goal", [this]() { cancelForwardedGoal(); });
        runShutdownStep("clear maneuver queue", [this]() { clearManeuverQueue("custom operation deactivated"); });
        runShutdownStep("finish operation as canceled", [this]() { finishOperationAsCanceled(); });
        runShutdownStep("stop maneuver reference client", [this]() { stopManeuverReferenceClient(); });
        runShutdownStep("reset maneuver reference client", [this]() {
            maneuver_reference_client_->SetReferenceModeHover(true);
        });
    }

    void updateSetpoint(float dt) override {
        if (!active_.load()) {
            return;
        }

        const auto reference = maneuver_reference_client_->GetReference(
            dt,
            [this]() {
                RCLCPP_ERROR(
                    node_.get_logger(),
                    "CustomOperationMode::updateSetpoint(): Maneuver reference unavailable; cancelling active operation and hovering."
                );
                cancelForwardedGoal();
                finishOperationAsCanceled();
                maneuver_reference_client_->SetReferenceModeHover(true);
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
            return maneuver_registered_as_offboard_;
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
        return operation_active_;
    }

    std::string activeOperation() const {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return active_operation_;
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
    rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::CallbackGroup::SharedPtr get_reference_callback_group_;
    rclcpp::CallbackGroup::SharedPtr operation_callback_group_;
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
    std::atomic_bool maneuver_reference_active_{false};
    std::atomic_bool manual_position_control_triggered_{false};
    bool maneuver_registered_as_offboard_{false};

    mutable std::mutex operation_mutex_;
    bool operation_active_{false};
    bool cancel_requested_{false};
    std::string active_operation_;
    std::string last_rejection_reason_;
    std::shared_ptr<CustomOperationGoalHandle> active_operation_goal_handle_;
    std::function<void()> cancel_forwarded_goal_;
    std::function<void()> cancel_operation_goal_;

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

        runShutdownStep("cancel forwarded goal after manual input", [this]() { cancelForwardedGoal(); });
        runShutdownStep("clear maneuver queue after manual input", [this]() { clearManeuverQueue("custom operation manual position control triggered"); });
        runShutdownStep("finish operation as canceled after manual input", [this]() { finishOperationAsCanceled(); });
        runShutdownStep("set hover reference while switching to position", [this]() { maneuver_reference_client_->SetReferenceModeHover(true); });

        publishSetNavStateCommand(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL);
    }

    void publishSetNavStateCommand(uint8_t nav_state) {
        px4_msgs::msg::VehicleCommand command;
        command.timestamp = static_cast<uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);
        command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
        command.param1 = static_cast<float>(nav_state);
        command.target_system = 1;
        command.target_component = 1;
        command.source_system = 1;
        command.source_component = 1;
        command.from_external = true;
        vehicle_command_pub_->publish(command);
    }

    rclcpp_action::GoalResponse handleOperationGoal(std::shared_ptr<const CustomOperation::Goal> goal) {
        if (!active_.load()) {
            const std::string reason = "CustomOperation mode is not active";
            {
                std::lock_guard<std::mutex> lock(operation_mutex_);
                last_rejection_reason_ = reason;
            }
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::handleOperationGoal(): Rejecting %s because %s.", goal->operation.c_str(), reason.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (manual_position_control_triggered_.load()) {
            const std::string reason = "manual position control handoff is in progress";
            {
                std::lock_guard<std::mutex> lock(operation_mutex_);
                last_rejection_reason_ = reason;
            }
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::handleOperationGoal(): Rejecting %s because %s.", goal->operation.c_str(), reason.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        std::lock_guard<std::mutex> lock(operation_mutex_);
        if (operation_active_) {
            last_rejection_reason_ = "another custom operation is active";
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::handleOperationGoal(): Rejecting %s because an operation is active.", goal->operation.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleOperationCancel(const std::shared_ptr<CustomOperationGoalHandle>) {
        RCLCPP_INFO(
            node_.get_logger(),
            "CustomOperationMode::handleOperationCancel(): Forwarding cancellation and retaining maneuver reference control until the maneuver stops."
        );
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            cancel_requested_ = true;
        }
        cancelForwardedGoal();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleOperationAccepted(const std::shared_ptr<CustomOperationGoalHandle> goal_handle) {
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            operation_active_ = true;
            cancel_requested_ = false;
            active_operation_ = goal_handle->get_goal()->operation;
            active_operation_goal_handle_ = goal_handle;
            last_rejection_reason_.clear();
            cancel_operation_goal_ = [this, goal_handle]() {
                finishOperationGoalAsCancelledOrAborted(goal_handle, "operation cancelled");
            };
        }

        const auto goal = goal_handle->get_goal();
        const std::string & operation = goal->operation;
        RCLCPP_INFO(
            node_.get_logger(),
            "CustomOperationMode::handleOperationAccepted(): Dispatching operation '%s'.",
            operation.c_str()
        );
        try {
            if (operation == "fly_to_position") {
                dispatchTyped<iii_drone_interfaces::action::FlyToPosition>(
                    goal_handle,
                    fly_to_position_client_,
                    makeFlyToPositionGoal(goal->arguments_json)
                );
            } else if (operation == "follow_waypoint_path") {
                dispatchTyped<iii_drone_interfaces::action::FollowWaypointPath>(
                    goal_handle,
                    follow_waypoint_path_client_,
                    makeFollowWaypointPathGoal(goal->arguments_json)
                );
            } else if (operation == "cable_aware_fly_to_position") {
                dispatchTyped<iii_drone_interfaces::action::CableAwareFlyToPosition>(
                    goal_handle,
                    cable_aware_fly_to_position_client_,
                    makeCableAwareFlyToPositionGoal(goal->arguments_json)
                );
            } else if (operation == "fly_to_object") {
                dispatchTyped<iii_drone_interfaces::action::FlyToObject>(
                    goal_handle,
                    fly_to_object_client_,
                    makeFlyToObjectGoal(goal->arguments_json)
                );
            } else if (operation == "hover") {
                dispatchTyped<iii_drone_interfaces::action::Hover>(
                    goal_handle,
                    hover_client_,
                    makeHoverGoal(goal->arguments_json)
                );
            } else if (operation == "hover_by_object") {
                dispatchTyped<iii_drone_interfaces::action::HoverByObject>(
                    goal_handle,
                    hover_by_object_client_,
                    makeHoverByObjectGoal(goal->arguments_json)
                );
            } else if (operation == "hover_on_cable") {
                dispatchTyped<iii_drone_interfaces::action::HoverOnCable>(
                    goal_handle,
                    hover_on_cable_client_,
                    makeHoverOnCableGoal(goal->arguments_json)
                );
            } else if (operation == "cable_landing") {
                dispatchTyped<iii_drone_interfaces::action::CableLanding>(
                    goal_handle,
                    cable_landing_client_,
                    makeCableLandingGoal(goal->arguments_json)
                );
            } else if (operation == "cable_takeoff") {
                dispatchTyped<iii_drone_interfaces::action::CableTakeoff>(
                    goal_handle,
                    cable_takeoff_client_,
                    makeCableTakeoffGoal(goal->arguments_json)
                );
            } else {
                abortOperation(goal_handle, "unsupported operation: " + operation);
            }
        } catch (const std::exception & error) {
            abortOperation(goal_handle, error.what());
        }
    }

    template <typename ActionT>
    void dispatchTyped(
        const std::shared_ptr<CustomOperationGoalHandle> operation_goal,
        typename rclcpp_action::Client<ActionT>::SharedPtr client,
        const typename ActionT::Goal & forwarded_goal
    ) {
        const std::string operation_name = operation_goal->get_goal()->operation;
        if (!ensureRegisteredAsOffboardMode()) {
            abortOperation(operation_goal, "failed to register CustomOperation as maneuver-controller offboard mode before dispatching: " + operation_name);
            return;
        }

        if (!client->wait_for_action_server(std::chrono::seconds(2))) {
            abortOperation(operation_goal, "underlying maneuver action unavailable: " + operation_name);
            return;
        }

        auto goal_response_received = std::make_shared<std::atomic_bool>(false);
        auto goal_response_watchdog = std::make_shared<rclcpp::TimerBase::SharedPtr>();
        *goal_response_watchdog = node_.create_wall_timer(
            std::chrono::seconds(3),
            [this, operation_goal, operation_name, goal_response_received, goal_response_watchdog]() {
                if (goal_response_received->exchange(true)) {
                    return;
                }
                if (*goal_response_watchdog) {
                    (*goal_response_watchdog)->cancel();
                }
                if (!operationGoalStillActive(operation_goal)) {
                    return;
                }
                abortOperation(
                    operation_goal,
                    "timed out waiting for underlying maneuver goal response: " + operation_name
                );
            },
            operation_callback_group_
        );

        typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
        options.goal_response_callback =
            [this, operation_goal, client, operation_name, goal_response_received, goal_response_watchdog](
                typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr forwarded_goal_handle
            ) {
                try {
                    goal_response_received->store(true);
                    if (*goal_response_watchdog) {
                        (*goal_response_watchdog)->cancel();
                    }
                    if (!operationGoalStillActive(operation_goal)) {
                        RCLCPP_WARN(
                            node_.get_logger(),
                            "CustomOperationMode::dispatchTyped(): Ignoring late maneuver goal response for inactive operation '%s'.",
                            operation_name.c_str()
                        );
                        if (forwarded_goal_handle) {
                            try {
                                client->async_cancel_goal(forwarded_goal_handle);
                            } catch (const std::exception & error) {
                                RCLCPP_WARN(
                                    node_.get_logger(),
                                    "CustomOperationMode::dispatchTyped(): Ignoring late cancel error for '%s': %s",
                                    operation_name.c_str(),
                                    error.what()
                                );
                            }
                        }
                        return;
                    }

                    if (!forwarded_goal_handle) {
                        abortOperation(operation_goal, "underlying maneuver goal rejected: " + operation_name);
                        return;
                    }

                    RCLCPP_INFO(
                        node_.get_logger(),
                        "CustomOperationMode::dispatchTyped(): Underlying maneuver goal accepted for '%s'; starting ManeuverReferenceClient.",
                        operation_name.c_str()
                    );

                    if (!maneuver_reference_client_->StartManeuver()) {
                        client->async_cancel_goal(forwarded_goal_handle);
                        abortOperation(operation_goal, "failed to start ManeuverReferenceClient for: " + operation_name);
                        return;
                    }
                    maneuver_reference_active_.store(true);

                    bool cancel_pending = false;
                    {
                        std::lock_guard<std::mutex> lock(operation_mutex_);
                        cancel_forwarded_goal_ = [client, forwarded_goal_handle]() {
                            try {
                                client->async_cancel_goal(forwarded_goal_handle);
                            } catch (const std::exception &) {
                                // Goal cancellation is best-effort; terminal callbacks handle cleanup.
                            }
                        };
                        cancel_pending = cancel_requested_;
                    }
                    if (cancel_pending) {
                        try {
                            client->async_cancel_goal(forwarded_goal_handle);
                        } catch (const std::exception &) {
                            // Goal cancellation is best-effort; terminal callbacks handle cleanup.
                        }
                    }
                } catch (const std::exception & error) {
                    if (forwarded_goal_handle) {
                        try {
                            client->async_cancel_goal(forwarded_goal_handle);
                        } catch (const std::exception &) {
                            // Best-effort cleanup only.
                        }
                    }
                    abortOperation(operation_goal, std::string("exception while starting ManeuverReferenceClient for ") + operation_name + ": " + error.what());
                    return;
                }
            };
        options.feedback_callback =
            [operation_goal, operation_name](
                typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
                const std::shared_ptr<const typename ActionT::Feedback>
            ) {
                auto feedback = std::make_shared<CustomOperation::Feedback>();
                feedback->operation = operation_name;
                feedback->state = "running";
                feedback->feedback_json = "{}";
                operation_goal->publish_feedback(feedback);
            };
        options.result_callback =
            [this, operation_goal, operation_name](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wrapped_result) {
                handleForwardedResult<ActionT>(operation_goal, operation_name, wrapped_result);
            };

        client->async_send_goal(forwarded_goal, options);
    }

    template <typename ActionT>
    void handleForwardedResult(
        const std::shared_ptr<CustomOperationGoalHandle> operation_goal,
        const std::string & operation_name,
        const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wrapped_result
    ) {
        if (!operationGoalStillActive(operation_goal)) {
            RCLCPP_WARN(
                node_.get_logger(),
                "CustomOperationMode::handleForwardedResult(): Ignoring late result for inactive operation '%s'.",
                operation_name.c_str()
            );
            return;
        }
        const bool cancelled = wrapped_result.code == rclcpp_action::ResultCode::CANCELED || cancelRequested();
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && resultReference(wrapped_result.result)) {
            maneuver_reference_client_->StopManeuver(*resultReference(wrapped_result.result));
        } else {
            stopManeuverReferenceClient();
        }
        if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !resultSucceeded(wrapped_result.result)) {
            clearManeuverQueue("custom operation terminal failure/cancel");
        }

        auto result = std::make_shared<CustomOperation::Result>();
        result->operation = operation_name;
        result->success = wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && resultSucceeded(wrapped_result.result);
        result->result_json = resultJson(wrapped_result.result);
        result->error = result->success ? "" : "underlying maneuver action failed";

        if (cancelled) {
            result->success = false;
            result->error = "operation cancelled";
            finishOperationGoalAsCancelledOrAborted(operation_goal, result);
        } else if (result->success) {
            operation_goal->succeed(result);
        } else {
            operation_goal->abort(result);
        }
        clearOperation();
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

    void abortOperation(const std::shared_ptr<CustomOperationGoalHandle> goal_handle, const std::string & error) {
        RCLCPP_ERROR(node_.get_logger(), "CustomOperationMode::abortOperation(): %s", error.c_str());
        stopManeuverReferenceClient();
        clearManeuverQueue("custom operation aborted");
        auto result = std::make_shared<CustomOperation::Result>();
        result->success = false;
        result->operation = goal_handle->get_goal()->operation;
        result->error = error;
        result->result_json = "{}";
        finishOperationGoalAsCancelledOrAborted(goal_handle, result);
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            last_rejection_reason_ = error;
        }
        clearOperation();
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

    bool cancelRequested() {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return cancel_requested_;
    }

    bool operationGoalStillActive(const std::shared_ptr<CustomOperationGoalHandle> & goal_handle) {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        return operation_active_ && active_operation_goal_handle_ == goal_handle;
    }

    void clearOperation() {
        std::lock_guard<std::mutex> lock(operation_mutex_);
        operation_active_ = false;
        cancel_requested_ = false;
        active_operation_.clear();
        active_operation_goal_handle_.reset();
        cancel_forwarded_goal_ = nullptr;
        cancel_operation_goal_ = nullptr;
    }

    void finishOperationAsCanceled() {
        std::function<void()> cancel_operation;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            cancel_operation = cancel_operation_goal_;
        }
        if (cancel_operation) {
            cancel_operation();
        }
        clearOperation();
    }

    void cancelForwardedGoal() {
        std::function<void()> cancel_goal;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            cancel_goal = cancel_forwarded_goal_;
        }
        if (cancel_goal) {
            cancel_goal();
        }
    }

    void stopManeuverReferenceClient() {
        if (maneuver_reference_active_.exchange(false)) {
            maneuver_reference_client_->StopManeuver();
        } else {
            maneuver_reference_client_->SetReferenceModeHover(true);
        }
    }

    void clearManeuverQueue(const std::string & reason) {
        if (!clear_maneuver_queue_client_->wait_for_service(std::chrono::milliseconds(250))) {
            RCLCPP_WARN(node_.get_logger(), "CustomOperationMode::clearManeuverQueue(): Service unavailable.");
            return;
        }
        auto request = std::make_shared<iii_drone_interfaces::srv::ClearManeuverQueue::Request>();
        request->reason = reason;
        clear_maneuver_queue_client_->async_send_request(request);
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

    rclcpp::executors::MultiThreadedExecutor executor;
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
