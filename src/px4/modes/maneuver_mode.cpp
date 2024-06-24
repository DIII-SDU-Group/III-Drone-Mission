/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>

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
    bool allow_activate_when_disarmed
) : px4_ros2::ModeBase(
        node, 
        Settings(
            mode_name,
            allow_activate_when_disarmed
        )
),  mode_name_(mode_name) { 

    // RCLCPP_DEBUG(node.get_logger(), "ManeuverMode::ManeuverMode(): Initializing mode %s", mode_name.c_str());

    traj_setpoint_ = std::make_shared<iii_drone::px4::TrajectorySetpoint>(*this);

    dt_ = dt;

    register_offboard_mode_callback_group_ = node.create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    register_offboard_mode_client_ = node.create_client<iii_drone_interfaces::srv::RegisterOffboardMode>(
        "/control/maneuver_controller/register_offboard_mode",
        rmw_qos_profile_services_default,
        register_offboard_mode_callback_group_
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

    if (!doRegister()) {
        RCLCPP_FATAL(node().get_logger(), "ManeuverMode::Register(): Failed to register mode %s with PX4", mode_name_.c_str());
        throw std::runtime_error("ManeuverMode::Register(): Failed to register mode with PX4");
    }

    sendRegisterOffboardModeRequest(false);

    is_registered_ = true;

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Register(): Mode %s registered", mode_name_.c_str());

}

void ManeuverMode::Unregister() {

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Unregister(): Unregistering mode %s", mode_name_.c_str());

    if (!is_registered_) {
        RCLCPP_WARN(node().get_logger(), "ManeuverMode::Unregister(): Mode %s not registered", mode_name_.c_str());
        return;
    }

    maneuver_reference_client_.reset();

    tree_executor_.reset();

    sendRegisterOffboardModeRequest(true);

    if (!doUnregister()) {
        RCLCPP_FATAL(node().get_logger(), "ManeuverMode::Unregister(): Failed to unregister mode %s with PX4", mode_name_.c_str());
        throw std::runtime_error("ManeuverMode::Unregister(): Failed to unregister mode with PX4");
    }

    is_registered_ = false;

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::Unregister(): Mode %s deregistered", mode_name_.c_str());
    
}

void ManeuverMode::sendRegisterOffboardModeRequest(bool deregister) {

    auto request = std::make_shared<iii_drone_interfaces::srv::RegisterOffboardMode::Request>();
    request->mode_id = id();
    request->deregister = deregister;

    while (!register_offboard_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Service not available, waiting again...");
    }

    Atomic<bool> done = false;

    auto result = register_offboard_mode_client_->async_send_request(
        request,
        [&done](rclcpp::Client<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedFuture) {
            done = true;
        }
    );

    rclcpp::Time start_time = node().now();

    auto rate = rclcpp::Rate(0.1);

    while (!done) {

        if (node().now() - start_time > rclcpp::Duration::from_seconds(5)) {

            RCLCPP_FATAL(node().get_logger(), "ManeuverMode::sendRegisterOffboardModeRequest(): Failed to register mode %s as offboard mode", mode_name_.c_str());
            
            throw std::runtime_error("ManeuverMode::sendRegisterOffboardModeRequest(): Failed to register mode as offboard mode");

        }

        rate.sleep();

    }
}

void ManeuverMode::onActivate() {

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::onActivate(): Activating mode %s", mode_name_.c_str());

    maneuver_reference_client_->SetReferenceModeHover();

    setSetpointUpdateRate(1./dt_);

    tree_executor_->StartExecution();

}

void ManeuverMode::onDeactivate() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::onDeactivate(): Deactivating mode %s", mode_name_.c_str());

    tree_executor_->StopExecution();

}

void ManeuverMode::updateSetpoint(float dt) { 

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

    if (tree_executor_->finished()) {

        RCLCPP_INFO_ONCE(
            node().get_logger(), 
            "ManeuverMode::updateSetpoint(): Tree execution finished %s",
            tree_executor_->success() ? "successfully" : "unsuccessfully"
        );
        
        completed(
            tree_executor_->success() ? px4_ros2::Result::Success : px4_ros2::Result::ModeFailureOther
        );

    }
}

std::string ManeuverMode::mode_name() const { return mode_name_; }