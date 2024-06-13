/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/px4/modes/maneuver_mode.hpp>

using namespace iii_drone::px4;
using namespace iii_drone::control;
using namespace iii_drone::utils;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ManeuverMode::ManeuverMode(
    rclcpp::Node & node,
    std::string mode_name,
    iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    float dt
) : px4_ros2::ModeBase(
        node, 
        Settings(
            mode_name,
            false
        )
),  maneuver_reference_client_(maneuver_reference_client),
    mode_name_(mode_name) { 

    // RCLCPP_DEBUG(node.get_logger(), "ManeuverMode::ManeuverMode(): Initializing mode %s", mode_name.c_str());

    traj_setpoint_ = std::make_shared<iii_drone::px4::TrajectorySetpoint>(*this);

    dt_ = dt;

}

void ManeuverMode::RegisterAsOffboardMode() {

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::RegisterAsOffboardMode(): Registering mode %s as offboard mode", mode_name_.c_str());

    rclcpp::Client<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedPtr client 
        = node().create_client<iii_drone_interfaces::srv::RegisterOffboardMode>("/control/maneuver_controller/register_offboard_mode");

    auto request = std::make_shared<iii_drone_interfaces::srv::RegisterOffboardMode::Request>();
    request->mode_id = id();
    request->deregister = false;

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node().get_logger(), "ManeuverMode::RegisterAsOffboardMode(): Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(node().get_logger(), "ManeuverMode::RegisterAsOffboardMode(): Service not available, waiting again...");
    }

    Atomic<bool> done = false;

    auto result = client->async_send_request(
        request,
        [&done](rclcpp::Client<iii_drone_interfaces::srv::RegisterOffboardMode>::SharedFuture) {
            done = true;
        }
    );

    rclcpp::Time start_time = node().now();

    while (!done) {

        rclcpp::spin_some(node().get_node_base_interface());

        if (node().now() - start_time > rclcpp::Duration::from_seconds(5)) {

            RCLCPP_FATAL(node().get_logger(), "ManeuverMode::RegisterAsOffboardMode(): Failed to register mode %s as offboard mode", mode_name_.c_str());
            
            throw std::runtime_error("ManeuverMode::RegisterAsOffboardMode(): Failed to register mode as offboard mode");

        }

    }

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::RegisterAsOffboardMode(): Mode %s registered as offboard mode", mode_name_.c_str());

}

void ManeuverMode::onActivate() {

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::onActivate(): Activating mode %s", mode_name_.c_str());

    maneuver_reference_client_->SetReferenceModeHover();

    setSetpointUpdateRate(1./dt_);

}

void ManeuverMode::onDeactivate() { 

    RCLCPP_INFO(node().get_logger(), "ManeuverMode::onDeactivate(): Deactivating mode %s", mode_name_.c_str());

}

void ManeuverMode::updateSetpoint(float dt) { 

    Reference reference = maneuver_reference_client_->GetReference(dt);

    // RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::updateSetpoint(): Updating setpoint with reference:");
    // RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::updateSetpoint(): position: [%f, %f, %f]", reference.position().x(), reference.position().y(), reference.position().z());
    // RCLCPP_DEBUG(node().get_logger(), "ManeuverMode::updateSetpoint(): yaw: %f", reference.yaw());

    traj_setpoint_->update(reference);

}