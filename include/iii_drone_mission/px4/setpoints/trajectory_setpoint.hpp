#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/control/reference.hpp>

#include <iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/trajectory_setpoint.hpp>

/*****************************************************************************/
// PX4-ROS2:

#include <px4_ros2/common/setpoint_base.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {

namespace px4 {

    class TrajectorySetpoint : public px4_ros2::SetpointBase {
    public:
        explicit TrajectorySetpoint(px4_ros2::Context & context);

        ~TrajectorySetpoint() override = default;

        Configuration getConfiguration() override;

        void update(const iii_drone::control::Reference & reference);

    private:
        rclcpp::Node & node_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;


    };
    
} // namespace px4
} // namespace iii_drone