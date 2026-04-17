/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>

#include <exception>
#include <future>

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
    is_owned_mode_(is_owned_mode) { 

    traj_setpoint_ = std::make_shared<iii_drone::px4::TrajectorySetpoint>(*this);

    dt_ = dt;

    register_offboard_mode_client_ = node.create_client<iii_drone_interfaces::srv::RegisterOffboardMode>(
        "/control/maneuver_controller/register_offboard_mode",
        rclcpp::ServicesQoS()
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

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Unregister(): Mode %s deregistered", mode_name_.c_str());
    
}

void ManeuverMode::sendRegisterOffboardModeRequest(
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
            return;
        }
        RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Service not available, waiting again...");

        if (++cnt >= max_attempts) {
            if (force) {
                RCLCPP_WARN(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Service not available after %d attempts, continuing", max_attempts);
                return;
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
            return;
        }

        RCLCPP_FATAL(
            node().get_logger(),
            "ManeuverMode::sendRegisterOffboardModeRequest(): Failed to %s mode %s as offboard mode",
            deregister ? "deregister" : "register",
            mode_name_.c_str()
        );
        throw std::runtime_error("ManeuverMode::sendRegisterOffboardModeRequest(): Failed to register mode as offboard mode");
    }

}

void ManeuverMode::onActivate() {

    RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::onActivate(): Activating mode %s", mode_name_.c_str());

    if (!tree_executor_->running()) {

        RCLCPP_INFO(node().get_logger(), "ManeuverMode::onActivate(): Starting mode %s", mode_name_.c_str());

        maneuver_reference_client_->SetReferenceModeHover();

        setSetpointUpdateRate(1./dt_);

        tree_completion_reported_ = false;

        tree_executor_->StartExecution();

        stop_controls_ = false;
    
    } else {

        RCLCPP_WARN(node().get_logger(), "ManeuverMode::onActivate(): Resuming mode %s", mode_name_.c_str());

    }

    if (on_next_activate_callback_) {

        on_next_activate_callback_();

    }

    on_next_activate_callback_ = nullptr;

}

void ManeuverMode::onDeactivate() { 

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

}

void ManeuverMode::updateSetpoint(float dt) { 

    if (!stop_controls_) {

        Reference reference = maneuver_reference_client_->GetReference(
            dt,
            [this]() {
                RCLCPP_ERROR(
                    node().get_logger(), 
                    "ManeuverMode::updateSetpoint(): Reference not available for mode %s, halting behavior tree execution", 
                    mode_name_.c_str()
                );
                tree_executor_->StopExecution(false);
                maneuver_reference_client_->SetReferenceModeHover();
            }
        );

        traj_setpoint_->update(reference);

    }

    if (tree_executor_->finished() && !tree_completion_reported_) {

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
