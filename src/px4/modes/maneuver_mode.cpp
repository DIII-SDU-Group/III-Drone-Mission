/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>
#include <iii_drone_core/diagnostics/hil_trace.hpp>

#include <chrono>
#include <exception>
#include <future>
#include <sstream>
#include <utility>

using namespace iii_drone::px4;
using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::utils;
using namespace iii_drone::behavior;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverMode::ManeuverMode(
    rclcpp::Node & node,
    std::string mode_key,
    std::string mode_name,
    float dt,
    bool is_owned_mode,
    bool allow_activate_when_disarmed,
    uint64_t lifecycle_activation_generation
) : px4_ros2::ModeBase(
        node, 
        Settings(
            mode_name,
            allow_activate_when_disarmed
        ),
        "/"
),  mode_name_(mode_name),
    mode_key_(mode_key),
    is_owned_mode_(is_owned_mode),
    lifecycle_activation_generation_(lifecycle_activation_generation) {

    traj_setpoint_ = std::make_shared<iii_drone::px4::TrajectorySetpoint>(*this);

    dt_ = dt;

    register_offboard_mode_client_ = node.create_client<iii_drone_interfaces::srv::RegisterOffboardMode>(
        "/control/maneuver_controller/register_offboard_mode",
        rclcpp::ServicesQoS()
    );

    vehicle_command_publisher_ = node.create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command",
        rclcpp::SystemDefaultsQoS()
    );

    vehicle_status_subscription_ = node.create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status_v1",
        rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
            const auto callback_start = std::chrono::steady_clock::now();
            auto callback_entry = iii_drone::diagnostics::HilTrace::event("callback_group_callback_entry");
            callback_entry.text("callback", "maneuver_vehicle_status_v1");
            callback_entry.text("callback_group", "px4_mode_default_mutually_exclusive");
            callback_entry.text("callback_group_type", "MutuallyExclusive");
            callback_entry.text("node", "/px4_mode");
            callback_entry.commit();
            auto event = iii_drone::diagnostics::HilTrace::event("maneuver_vehicle_status_callback");
            event.text("mode", mode_key_);
            event.number("timestamp", message->timestamp);
            event.number("system_id", message->system_id);
            event.number("component_id", message->component_id);
            event.boolean("failsafe", message->failsafe);
            event.text("callback_group", "px4_mode_default_mutually_exclusive");
            event.text("callback_group_type", "MutuallyExclusive");
            event.commit();
            if (message->system_id != 0) {
                vehicle_system_id_ = message->system_id;
            }
            if (message->component_id != 0) {
                vehicle_component_id_ = message->component_id;
            }
            if (message->timestamp != 0) {
                vehicle_timestamp_ = message->timestamp;
            }
            const auto callback_end = std::chrono::steady_clock::now();
            auto callback_exit = iii_drone::diagnostics::HilTrace::event("callback_group_callback_exit");
            callback_exit.text("callback", "maneuver_vehicle_status_v1");
            callback_exit.text("callback_group", "px4_mode_default_mutually_exclusive");
            callback_exit.text("callback_group_type", "MutuallyExclusive");
            callback_exit.text("node", "/px4_mode");
            callback_exit.number(
                "duration_ns",
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    callback_end - callback_start).count()));
            callback_exit.commit();
        }
    );

    status_publisher_ = node.create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "/mission/modes/" + mode_key_ + "/status",
        rclcpp::SystemDefaultsQoS()
    );
    status_timer_ = node.create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            const auto callback_start = std::chrono::steady_clock::now();
            auto timer_entry = iii_drone::diagnostics::HilTrace::event("mode_status_timer_entry");
            timer_entry.text("mode_key", mode_key_);
            timer_entry.text("mode_name", mode_name_);
            timer_entry.number("mode_id", static_cast<uint64_t>(id()));
            timer_entry.number("object_address", reinterpret_cast<uintptr_t>(this));
            timer_entry.number("status_publisher_address", reinterpret_cast<uintptr_t>(status_publisher_.get()));
            timer_entry.number("status_timer_address", reinterpret_cast<uintptr_t>(status_timer_.get()));
            timer_entry.number("lifecycle_activation_generation", lifecycle_activation_generation_);
            timer_entry.text("callback_group", "px4_mode_default_mutually_exclusive");
            timer_entry.text("callback_group_type", "MutuallyExclusive");
            timer_entry.commit();
            publishStatus();
            const auto callback_end = std::chrono::steady_clock::now();
            auto timer_exit = iii_drone::diagnostics::HilTrace::event("mode_status_timer_exit");
            timer_exit.text("mode_key", mode_key_);
            timer_exit.text("callback_group", "px4_mode_default_mutually_exclusive");
            timer_exit.text("callback_group_type", "MutuallyExclusive");
            timer_exit.number(
                "duration_ns",
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    callback_end - callback_start).count()));
            timer_exit.commit();
        }
    );

    auto constructed = iii_drone::diagnostics::HilTrace::event("mode_status_object_constructed");
    constructed.text("mode_key", mode_key_);
    constructed.text("mode_name", mode_name_);
    constructed.number("mode_id", static_cast<uint64_t>(id()));
    constructed.number("object_address", reinterpret_cast<uintptr_t>(this));
    constructed.number("status_publisher_address", reinterpret_cast<uintptr_t>(status_publisher_.get()));
    constructed.number("status_timer_address", reinterpret_cast<uintptr_t>(status_timer_.get()));
    constructed.number("lifecycle_activation_generation", lifecycle_activation_generation_);
    constructed.commit();

}

ManeuverMode::~ManeuverMode() {
    auto destroyed = iii_drone::diagnostics::HilTrace::event("mode_status_object_destroyed");
    destroyed.text("mode_key", mode_key_);
    destroyed.text("mode_name", mode_name_);
    destroyed.number("mode_id", static_cast<uint64_t>(id()));
    destroyed.number("object_address", reinterpret_cast<uintptr_t>(this));
    destroyed.number("status_publisher_address", reinterpret_cast<uintptr_t>(status_publisher_.get()));
    destroyed.number("status_timer_address", reinterpret_cast<uintptr_t>(status_timer_.get()));
    destroyed.number("lifecycle_activation_generation", lifecycle_activation_generation_);
    destroyed.commit();
}

void ManeuverMode::Register(
    TreeExecutor::SharedPtr tree_executor,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) {

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Register(): Registering mode %s", mode_name_.c_str());

    if (is_registered_) {
        RCLCPP_WARN(node().get_logger(), "ManeuverMode::Register(): Mode %s already registered", mode_name_.c_str());
        return;
    }

    maneuver_reference_client_ = maneuver_reference_client;

    tree_executor_ = tree_executor;

    if (!is_owned_mode_) {
        if (!doRegister()) {
            RCLCPP_FATAL(node().get_logger(), "ManeuverMode::Register(): Failed to register mode %s with PX4", mode_name_.c_str());
            throw std::runtime_error("ManeuverMode::Register(): Failed to register mode with PX4");
        }
    }

    sendRegisterOffboardModeRequest(false);

    is_registered_ = true;
    publishStatus();
    startExecutionIfReady();

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Register(): Mode %s registered", mode_name_.c_str());

}

void ManeuverMode::Unregister(bool force) {

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Unregister(): Unregistering mode %s", mode_name_.c_str());

    if (!is_registered_) {
        RCLCPP_WARN(node().get_logger(), "ManeuverMode::Unregister(): Mode %s not registered", mode_name_.c_str());
        return;
    }

    maneuver_reference_client_.reset();

    tree_executor_.reset();

    RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::Unregister(): Sending deregister request for mode %s", mode_name_.c_str());

    sendRegisterOffboardModeRequest(
        true,
        force
    );

    RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::Unregister(): Calling doUnregister() for mode %s", mode_name_.c_str());

    if (!doUnregister()) {
        if (!force) {
            RCLCPP_FATAL(node().get_logger(), "ManeuverMode::Unregister(): Failed to unregister mode %s with PX4", mode_name_.c_str());
            throw std::runtime_error("ManeuverMode::Unregister(): Failed to unregister mode with PX4");
        } else {
            RCLCPP_WARN(node().get_logger(), "ManeuverMode::Unregister(): Failed to unregister mode %s with PX4", mode_name_.c_str());
        }
    }

    is_registered_ = false;
    active_ = false;
    publishStatus();

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Unregister(): Mode %s deregistered", mode_name_.c_str());
    
}

bool ManeuverMode::sendRegisterOffboardModeRequest(
    bool deregister,
    bool force
) {

    auto request = std::make_shared<iii_drone_interfaces::srv::RegisterOffboardMode::Request>();
    request->mode_id = id();
    request->deregister = deregister;

    int cnt = 0;
    const int max_attempts = 5;

    while (!register_offboard_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Service not available, waiting again...");

        if (++cnt >= max_attempts) {
            if (force) {
                RCLCPP_WARN(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Service not available after %d attempts, continuing", max_attempts);
                return false;
            } else {
                RCLCPP_FATAL(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Service not available after %d attempts, exiting", max_attempts);
                throw std::runtime_error("ManeuverMode::sendRegisterOffboardModeRequest(): Service not available after max attempts");
            }
        }
    }

    auto future = register_offboard_mode_client_->async_send_request(
        request
    );

    rclcpp::FutureReturnCode result = rclcpp::FutureReturnCode::TIMEOUT;
    auto node_base = node().get_node_base_interface();

    try {
        if (node_base->get_associated_with_executor_atomic().load()) {
            result = future.wait_for(std::chrono::seconds(5)) == std::future_status::ready
                ? rclcpp::FutureReturnCode::SUCCESS
                : rclcpp::FutureReturnCode::TIMEOUT;
        } else {
            result = rclcpp::spin_until_future_complete(
                node_base,
                future,
                std::chrono::seconds(5)
            );
        }
    } catch (const std::exception & exc) {
        RCLCPP_WARN(
            node().get_logger(),
            "ManeuverMode::sendRegisterOffboardModeRequest(): Failed while waiting for %s request for mode %s: %s",
            deregister ? "deregister" : "register",
            mode_name_.c_str(),
            exc.what()
        );
        result = future.wait_for(std::chrono::seconds(5)) == std::future_status::ready
            ? rclcpp::FutureReturnCode::SUCCESS
            : rclcpp::FutureReturnCode::TIMEOUT;
    }

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
        register_offboard_mode_client_->remove_pending_request(future);

        if (force) {
            RCLCPP_WARN(
                node().get_logger(),
                "ManeuverMode::sendRegisterOffboardModeRequest(): Failed to %s mode %s as offboard mode, continuing",
                deregister ? "deregister" : "register",
                mode_name_.c_str()
            );
            return false;
        }

        RCLCPP_FATAL(
            node().get_logger(),
            "ManeuverMode::sendRegisterOffboardModeRequest(): Failed to %s mode %s as offboard mode",
            deregister ? "deregister" : "register",
            mode_name_.c_str()
        );
        throw std::runtime_error("ManeuverMode::sendRegisterOffboardModeRequest(): Failed to register mode as offboard mode");
    }

    offboard_mode_registered_ = !deregister;
    return true;

}

void ManeuverMode::publishHoldCommand() {

    if (!vehicle_command_publisher_) {
        return;
    }

    px4_msgs::msg::VehicleCommand command;
    const auto vehicle_timestamp = vehicle_timestamp_.Load();
    command.timestamp = vehicle_timestamp != 0
        ? vehicle_timestamp
        : static_cast<uint64_t>(node().get_clock()->now().nanoseconds() / 1000);
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
    command.param1 = static_cast<float>(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
    command.target_system = vehicle_system_id_.Load();
    command.target_component = vehicle_component_id_.Load();
    command.source_system = 1;
    command.source_component = 1;
    command.from_external = true;
    vehicle_command_publisher_->publish(command);

}

void ManeuverMode::onActivate() {

    RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::onActivate(): Activating mode %s", mode_name_.c_str());

    active_ = true;
    publishStatus();
    startExecutionIfReady();

    on_next_activate_callback_.Invoke();

}

void ManeuverMode::startExecutionIfReady() {

    if (!active_) {
        return;
    }

    if (!tree_executor_ || !maneuver_reference_client_) {
        RCLCPP_WARN(
            node().get_logger(),
            "ManeuverMode::startExecutionIfReady(): Mode %s activated before execution dependencies were registered; deferring tree start.",
            mode_name_.c_str()
        );
        return;

    }

    if (!tree_executor_->running()) {

        RCLCPP_INFO(node().get_logger(), "ManeuverMode::startExecutionIfReady(): Starting mode %s", mode_name_.c_str());

        try {
            if (!offboard_mode_registered_) {
                throw std::runtime_error(
                    "ManeuverMode::startExecutionIfReady(): Mode has no confirmed maneuver-controller offboard registration"
                );
            }
        } catch (const std::exception & exception) {
            RCLCPP_ERROR(
                node().get_logger(),
                "ManeuverMode::startExecutionIfReady(): Cannot start mode %s: %s. "
                "Holding current reference and reporting mode failure.",
                mode_name_.c_str(),
                exception.what()
            );
            emergency_reference_hold_active_ = true;
            tree_completion_reported_ = true;
            stop_controls_ = false;
            maneuver_reference_client_->SetReferenceModeHover(true);
            publishHoldCommand();
            publishStatus();
            completed(px4_ros2::Result::ModeFailureOther);
            return;
        }

        reference_control_owner_ = maneuver_reference_client_->AcquireReferenceControl();

        // A predecessor mode may have completed while Core still owns a live
        // terminal correction. Claim that exact acknowledged generation before
        // this mode can publish its first setpoint or start a BT goal.
        const auto terminal_adoption =
            maneuver_reference_client_->TryAdoptTerminalHold(250);
        if (terminal_adoption == ManeuverReferenceClient::TerminalHoldAdoption::Failed) {
            RCLCPP_ERROR(node().get_logger(),
                "ManeuverMode::startExecutionIfReady(): terminal hold ownership transfer failed for %s",
                mode_name_.c_str());
            emergency_reference_hold_active_ = true;
            tree_completion_reported_ = true;
            stop_controls_ = false;
            publishHoldCommand();
            publishStatus();
            completed(px4_ros2::Result::ModeFailureOther);
            return;
        }

        setSetpointUpdateRate(1./dt_);

        tree_completion_reported_ = false;
        emergency_reference_hold_active_ = false;

        try {
            tree_executor_->StartExecution();
            publishStatus();

            stop_controls_ = false;
        } catch (const std::exception & exception) {
            RCLCPP_ERROR(
                node().get_logger(),
                "ManeuverMode::startExecutionIfReady(): Failed to start behavior tree for mode %s: %s. "
                "Holding current reference and reporting mode failure without terminating mission_executor.",
                mode_name_.c_str(),
                exception.what()
            );
            emergency_reference_hold_active_ = true;
            tree_completion_reported_ = true;
            stop_controls_ = false;
            maneuver_reference_client_->SetReferenceModeHover(true);
            publishHoldCommand();
            publishStatus();
            completed(px4_ros2::Result::ModeFailureOther);
        } catch (...) {
            RCLCPP_ERROR(
                node().get_logger(),
                "ManeuverMode::startExecutionIfReady(): Failed to start behavior tree for mode %s with unknown exception. "
                "Holding current reference and reporting mode failure without terminating mission_executor.",
                mode_name_.c_str()
            );
            emergency_reference_hold_active_ = true;
            tree_completion_reported_ = true;
            stop_controls_ = false;
            maneuver_reference_client_->SetReferenceModeHover(true);
            publishHoldCommand();
            publishStatus();
            completed(px4_ros2::Result::ModeFailureOther);
        }
    
    } else {

        RCLCPP_WARN(node().get_logger(), "ManeuverMode::startExecutionIfReady(): Resuming mode %s", mode_name_.c_str());

    }

}

void ManeuverMode::onDeactivate() { 

    active_ = false;
    emergency_reference_hold_active_ = false;
    publishStatus();

    if (!stay_alive_on_next_deactivate_) {

        RCLCPP_INFO(node().get_logger(), "ManeuverMode::onDeactivate(): Full deactivation of mode %s", mode_name_.c_str());

        // PX4 expects external-mode arming-check replies every 300 ms.  A
        // still-running tree must not be joined from this callback because a
        // kinematically safe action cancellation can take seconds.  A tree
        // that already reported completion is different: its worker is in the
        // final destruction window, and allowing the next mode to start while
        // that worker is still joinable races BT node teardown.  Join only the
        // completed case; keep the bounded non-blocking stop for interruption.
        const bool tree_finished = tree_executor_ != nullptr && tree_executor_->finished();
        if (tree_executor_ != nullptr) {
            tree_executor_->StopExecution(tree_finished, "MODE_DEACTIVATE");
        }
        // A successor may already own the shared client. A delayed predecessor
        // deactivation must not clear that successor's stream or pending goal.
        maneuver_reference_client_->ReleaseReferenceControl(reference_control_owner_.Load());
        reference_control_owner_ = 0;
        stop_controls_ = true;

    } else {

        RCLCPP_WARN(node().get_logger(), "ManeuverMode::onDeactivate(): Partial deactivation of mode %s", mode_name_.c_str());

    }

    on_next_activate_callback_.OnDeactivate(stay_alive_on_next_deactivate_);

    stay_alive_on_next_deactivate_ = false;

}

void ManeuverMode::StayAliveOnNextDeactivate() {

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::StayAliveOnNextDeactivate(): Will keep running on next deactivate for mode %s", mode_name_.c_str());
 
    stay_alive_on_next_deactivate_ = true;

}

void ManeuverMode::ClearStayAliveOnNextDeactivate() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::ClearStayAliveOnNextDeactivate(): Will not keep running on next deactivate for mode %s", mode_name_.c_str());

    stay_alive_on_next_deactivate_ = false;

}

void ManeuverMode::RegisterOnNextActivateCallback(std::function<void()> callback) { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::RegisterOnNextActivateCallback(): Registering callback for next activate for mode %s", mode_name_.c_str());

    on_next_activate_callback_.Set(std::move(callback));

}

void ManeuverMode::StopControls() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::StopControls(): Stopping controls for mode %s", mode_name_.c_str());

    stop_controls_ = true;

}

void ManeuverMode::StartControls() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::StartControls(): Starting controls for mode %s", mode_name_.c_str());

    stop_controls_ = false;

}

void ManeuverMode::StopExecution(const char * diagnostic_reason) { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::StopExecution(): Stopping execution for mode %s", mode_name_.c_str());

    // Signal cancellation here and let the tree worker complete its safe stop
    // without starving PX4 external-mode registration callbacks.  If the
    // worker already finished, join it before a subsequent activation can
    // destroy/recreate the behavior-tree object.
    if (tree_executor_ != nullptr) {
        tree_executor_->StopExecution(
            tree_executor_->finished(),
            diagnostic_reason == nullptr ? "MODE_STOP_EXECUTION" : diagnostic_reason
        );
    }

    on_next_activate_callback_.Cancel();

    active_ = false;
    emergency_reference_hold_active_ = false;
    publishStatus();

}

void ManeuverMode::updateSetpoint(float dt) { 

    const auto callback_start = std::chrono::steady_clock::now();
    bool tree_completion_detected = false;
    bool stop_execution_wait_entered = false;
    auto entry = iii_drone::diagnostics::HilTrace::event("maneuver_update_setpoint_entry");
    entry.text("mode", mode_key_);
    entry.text("callback_group", "px4_mode_default_mutually_exclusive");
    entry.text("callback_group_type", "MutuallyExclusive");
    entry.decimal("dt_s", dt);
    entry.commit();

    if (!stop_controls_) {

        Reference reference = maneuver_reference_client_->GetReference(
            dt,
            [this]() {
                RCLCPP_ERROR(
                    node().get_logger(), 
                    "ManeuverMode::updateSetpoint(): Reference not available for mode %s, entering emergency hover hold and halting behavior tree execution", 
                    mode_name_.c_str()
                );
                emergency_reference_hold_active_ = true;
                tree_executor_->StopExecution(false, "REFERENCE_FAILURE");
                maneuver_reference_client_->SetReferenceModeHover(true);
                publishHoldCommand();
            }
        );

        traj_setpoint_->update(reference);

    }

    if (tree_executor_->finished() && !tree_completion_reported_) {

        if (emergency_reference_hold_active_) {
            RCLCPP_ERROR(
                node().get_logger(),
                "ManeuverMode::updateSetpoint(): Tree finished after reference outage in mode %s; keeping PX4 mode alive with hover setpoints until explicit deactivation",
                mode_name_.c_str()
            );
            tree_completion_reported_ = true;
            const auto callback_end = std::chrono::steady_clock::now();
            auto exit = iii_drone::diagnostics::HilTrace::event("maneuver_update_setpoint_exit");
            exit.text("mode", mode_key_);
            exit.text("callback_group", "px4_mode_default_mutually_exclusive");
            exit.text("callback_group_type", "MutuallyExclusive");
            exit.boolean("tree_completion_detected", false);
            exit.boolean("stop_execution_wait_entered", false);
            exit.number(
                "duration_ns",
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                    callback_end - callback_start).count()));
            exit.commit();
            return;
        }

        tree_completion_reported_ = true;
        tree_completion_detected = true;
        const bool tree_success = tree_executor_->success();

        // GenericModeExecutor may activate the successor before PX4 invokes
        // this mode's onDeactivate().  Join the completed worker here so the
        // successor cannot construct its behavior tree while this mode still
        // owns a joinable execution thread or any final BT teardown work.
        stop_execution_wait_entered = true;
        tree_executor_->StopExecution(true, "MODE_COMPLETION_REAP");

        // Retire this mode's stream before completed() can activate a successor.
        // Do not publish another setpoint from this completed mode meanwhile.
        stop_controls_ = true;
        maneuver_reference_client_->ReleaseReferenceControl(reference_control_owner_.Load());
        reference_control_owner_ = 0;

        RCLCPP_INFO(
            node().get_logger(), 
            "ManeuverMode::updateSetpoint(): Tree execution finished %s",
            tree_success ? "successfully" : "unsuccessfully"
        );
        
        completed(
            tree_success ? px4_ros2::Result::Success : px4_ros2::Result::ModeFailureOther
        );

    }

    const auto callback_end = std::chrono::steady_clock::now();
    auto exit = iii_drone::diagnostics::HilTrace::event("maneuver_update_setpoint_exit");
    exit.text("mode", mode_key_);
    exit.text("callback_group", "px4_mode_default_mutually_exclusive");
    exit.text("callback_group_type", "MutuallyExclusive");
    exit.boolean("tree_completion_detected", tree_completion_detected);
    exit.boolean("stop_execution_wait_entered", stop_execution_wait_entered);
    exit.number(
        "duration_ns",
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            callback_end - callback_start).count()));
    exit.commit();
}

std::string ManeuverMode::mode_name() const { return mode_name_; }

std::string ManeuverMode::mode_key() const { return mode_key_; }

uint8_t ManeuverMode::mode_id() const { return static_cast<uint8_t>(id()); }

bool ManeuverMode::is_registered() const { return is_registered_; }

bool ManeuverMode::active() const { return active_; }

bool ManeuverMode::tree_running() const {
    return tree_executor_ != nullptr && tree_executor_->running();
}

bool ManeuverMode::tree_finished() const {
    return tree_executor_ != nullptr && tree_executor_->finished();
}

bool ManeuverMode::tree_success() const {
    return tree_executor_ != nullptr && tree_executor_->finished() && tree_executor_->success();
}

bool ManeuverMode::emergency_reference_hold_active() const {
    return emergency_reference_hold_active_;
}

std::string ManeuverMode::degraded_reason() const {
    if (emergency_reference_hold_active_) {
        return "emergency reference hold is active";
    }
    return "";
}

void ManeuverMode::publishStatus(const char * trigger) {

    auto publish_start = iii_drone::diagnostics::HilTrace::event("mode_status_publish_start");
    publish_start.text("mode_key", mode_key_);
    publish_start.text("mode_name", mode_name_);
    publish_start.text("trigger", trigger == nullptr ? "unknown" : trigger);
    publish_start.number("mode_id", static_cast<uint64_t>(id()));
    publish_start.number("object_address", reinterpret_cast<uintptr_t>(this));
    publish_start.number("status_publisher_address", reinterpret_cast<uintptr_t>(status_publisher_.get()));
    publish_start.number("status_timer_address", reinterpret_cast<uintptr_t>(status_timer_.get()));
    publish_start.number("lifecycle_activation_generation", lifecycle_activation_generation_);
    publish_start.text("callback_group", "px4_mode_default_mutually_exclusive");
    publish_start.text("callback_group_type", "MutuallyExclusive");
    publish_start.boolean("publisher_available", status_publisher_ != nullptr);
    publish_start.commit();

    if (!status_publisher_) {
        auto publish_return = iii_drone::diagnostics::HilTrace::event("mode_status_publish_return");
        publish_return.text("mode_key", mode_key_);
        publish_return.text("mode_name", mode_name_);
        publish_return.text("trigger", trigger == nullptr ? "unknown" : trigger);
        publish_return.number("mode_id", static_cast<uint64_t>(id()));
        publish_return.number("object_address", reinterpret_cast<uintptr_t>(this));
        publish_return.number("lifecycle_activation_generation", lifecycle_activation_generation_);
        publish_return.text("callback_group", "px4_mode_default_mutually_exclusive");
        publish_return.text("callback_group_type", "MutuallyExclusive");
        publish_return.boolean("published", false);
        publish_return.commit();
        return;
    }

    bool tree_running = false;
    bool tree_finished = false;
    bool tree_success = false;

    if (tree_executor_) {
        tree_running = tree_executor_->running();
        tree_finished = tree_executor_->finished();
        if (tree_finished) {
            tree_success = tree_executor_->success();
        }
    }

    std::ostringstream payload;
    payload << "{"
        << "\"mode_key\":\"" << mode_key_ << "\","
        << "\"mode_name\":\"" << mode_name_ << "\","
        << "\"mode_id\":" << static_cast<int>(id()) << ","
        << "\"active\":" << (active_ ? "true" : "false") << ","
        << "\"registered\":" << (is_registered_ ? "true" : "false") << ","
        << "\"tree_running\":" << (tree_running ? "true" : "false") << ","
        << "\"tree_finished\":" << (tree_finished ? "true" : "false") << ","
        << "\"tree_success\":" << (tree_success ? "true" : "false") << ","
        << "\"emergency_reference_hold_active\":" << (emergency_reference_hold_active_ ? "true" : "false")
        << "}";

    static rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

    iii_drone_interfaces::msg::StringStamped msg;
    msg.stamp = system_clock.now();
    msg.data = payload.str();
    status_publisher_->publish(msg);

    auto publish_return = iii_drone::diagnostics::HilTrace::event("mode_status_publish_return");
    publish_return.text("mode_key", mode_key_);
    publish_return.text("mode_name", mode_name_);
    publish_return.text("trigger", trigger == nullptr ? "unknown" : trigger);
    publish_return.number("mode_id", static_cast<uint64_t>(id()));
    publish_return.number("object_address", reinterpret_cast<uintptr_t>(this));
    publish_return.number("status_publisher_address", reinterpret_cast<uintptr_t>(status_publisher_.get()));
    publish_return.number("status_timer_address", reinterpret_cast<uintptr_t>(status_timer_.get()));
    publish_return.number("lifecycle_activation_generation", lifecycle_activation_generation_);
    publish_return.text("callback_group", "px4_mode_default_mutually_exclusive");
    publish_return.text("callback_group_type", "MutuallyExclusive");
    publish_return.number("stamp_sec", static_cast<uint64_t>(msg.stamp.sec));
    publish_return.number("stamp_nanosec", static_cast<uint64_t>(msg.stamp.nanosec));
    publish_return.boolean("published", true);
    publish_return.commit();

}
