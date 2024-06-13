/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/px4/setpoints/trajectory_setpoint.hpp>

using namespace iii_drone::px4;
using namespace iii_drone::control;
using namespace iii_drone::adapters::px4;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TrajectorySetpoint::TrajectorySetpoint(px4_ros2::Context & context) : px4_ros2::SetpointBase(context), 
    node_(context.node()) {

    trajectory_setpoint_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 
        1
    );

}

px4_ros2::SetpointBase::Configuration TrajectorySetpoint::getConfiguration() {

    px4_ros2::SetpointBase::Configuration config{};

    config.rates_enabled = true;
    config.attitude_enabled = true;
    config.acceleration_enabled = true;
    config.velocity_enabled = true;
    config.position_enabled = true;
    config.altitude_enabled = true;
    config.climb_rate_enabled = true;

    return config;

}

void TrajectorySetpoint::update(const Reference & reference) {

    onUpdate();

    TrajectorySetpointAdapter adapter(reference);

    px4_msgs::msg::TrajectorySetpoint msg = adapter.ToMsg();

    // RCLCPP_DEBUG(node_.get_logger(), "TrajectorySetpoint::update(): Publishing trajectory setpoint:");
    // RCLCPP_DEBUG(node_.get_logger(), "TrajectorySetpoint::update(): position: [%f, %f, %f]", msg.position[0], msg.position[1], msg.position[2]);
    // RCLCPP_DEBUG(node_.get_logger(), "TrajectorySetpoint::update(): velocity: [%f, %f, %f]", msg.velocity[0], msg.velocity[1], msg.velocity[2]);
    // RCLCPP_DEBUG(node_.get_logger(), "TrajectorySetpoint::update(): acceleration: [%f, %f, %f]", msg.acceleration[0], msg.acceleration[1], msg.acceleration[2]);
    // RCLCPP_DEBUG(node_.get_logger(), "TrajectorySetpoint::update(): yaw: %f", msg.yaw);
    // RCLCPP_DEBUG(node_.get_logger(), "TrajectorySetpoint::update(): yaw_rate: %f", msg.yawspeed);

    trajectory_setpoint_pub_->publish(adapter.ToMsg());

}
