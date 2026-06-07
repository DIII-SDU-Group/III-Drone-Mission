/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>

#include <chrono>
#include <exception>
#include <future>
#include <sstream>

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
    bool allow_activate_when_disarmed
) : px4_ros2::ModeBase(
        node, 
        Settings(
            mode_name,
            allow_activate_when_disarmed
        ),
        "/"
),  mode_name_(mode_name),
    mode_key_(mode_key),
    is_owned_mode_(is_owned_mode) { 

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

    status_publisher_ = node.create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "/mission/modes/" + mode_key_ + "/status",
        rclcpp::SystemDefaultsQoS()
    );
    status_timer_ = node.create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            publishStatus();
        }
    );

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
    command.timestamp = static_cast<uint64_t>(node().get_clock()->now().nanoseconds() / 1000);
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
    command.param1 = static_cast<float>(px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER);
    command.target_system = 1;
    command.target_component = 1;
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

    if (on_next_activate_callback_) {

        on_next_activate_callback_();

    }

    on_next_activate_callback_ = nullptr;

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

        maneuver_reference_client_->SetReferenceModeHover();

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

        tree_executor_->StopExecution();

    } else {

        RCLCPP_WARN(node().get_logger(), "ManeuverMode::onDeactivate(): Partial deactivation of mode %s", mode_name_.c_str());

    }

    stay_alive_on_next_deactivate_ = false;

    on_next_activate_callback_ = nullptr;

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

    on_next_activate_callback_ = callback;

}

void ManeuverMode::StopControls() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::StopControls(): Stopping controls for mode %s", mode_name_.c_str());

    stop_controls_ = true;

}

void ManeuverMode::StartControls() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::StartControls(): Starting controls for mode %s", mode_name_.c_str());

    stop_controls_ = false;

}

void ManeuverMode::StopExecution() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::StopExecution(): Stopping execution for mode %s", mode_name_.c_str());

    tree_executor_->StopExecution();

    on_next_activate_callback_ = nullptr;

    active_ = false;
    emergency_reference_hold_active_ = false;
    publishStatus();

}

void ManeuverMode::updateSetpoint(float dt) { 

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
                tree_executor_->StopExecution(false);
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
            return;
        }

        tree_completion_reported_ = true;
        const bool tree_success = tree_executor_->success();

        RCLCPP_INFO(
            node().get_logger(), 
            "ManeuverMode::updateSetpoint(): Tree execution finished %s",
            tree_success ? "successfully" : "unsuccessfully"
        );
        
        completed(
            tree_success ? px4_ros2::Result::Success : px4_ros2::Result::ModeFailureOther
        );

    }
}

std::string ManeuverMode::mode_name() const { return mode_name_; }

std::string ManeuverMode::mode_key() const { return mode_key_; }

bool ManeuverMode::is_registered() const { return is_registered_; }

bool ManeuverMode::active() const { return active_; }

void ManeuverMode::publishStatus() {

    if (!status_publisher_) {
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

}
