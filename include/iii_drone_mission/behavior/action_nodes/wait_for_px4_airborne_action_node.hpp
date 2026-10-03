#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <memory>
#include <optional>
#include <string>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// PX4:

#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/latest_message_subscription.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    /**
     * @brief Succeeds once PX4 has reported the vehicle airborne (not landed,
     * not maybe landed, no ground contact) while commanding at least
     * min_thrust, continuously for hold_ms; fails after timeout_ms otherwise.
     *
     * Gates the gripper release on the cable: PX4 must not consider the
     * vehicle landed (and cut thrust or disarm) while the gripper opens, and
     * must be applying thrust. Far above the ground PX4 reports airborne as
     * soon as it arms while its takeoff state machine still commands zero
     * thrust, so the land detector alone is not enough. Samples older than
     * kMaxLandSampleAgeS (PX4 republishes at least at 1 Hz) or
     * kMaxThrustSampleAgeS (every control cycle) count as unknown, i.e. not
     * airborne.
     */
    class WaitForPX4AirborneActionNode : public BT::StatefulActionNode {
    public:
        WaitForPX4AirborneActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            std::shared_ptr<rclcpp::Node> node
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

        static constexpr double kMaxLandSampleAgeS = 2.5;
        static constexpr double kMaxThrustSampleAgeS = 1.0;

        /**
         * @brief Whether a land-detector sample received at receive_time
         * shows the vehicle airborne at now.
         */
        static bool Airborne(
            const px4_msgs::msg::VehicleLandDetected & sample,
            const rclcpp::Time & receive_time,
            const rclcpp::Time & now
        );

        /**
         * @brief Whether PX4's position controller output received at
         * receive_time commands at least min_thrust upward at now.
         */
        static bool Thrusting(
            const px4_msgs::msg::VehicleLocalPositionSetpoint & sample,
            const rclcpp::Time & receive_time,
            const rclcpp::Time & now,
            double min_thrust
        );

    private:
        std::shared_ptr<rclcpp::Node> node_;
        LatestMessageSubscription<px4_msgs::msg::VehicleLandDetected> land_detected_;
        LatestMessageSubscription<px4_msgs::msg::VehicleLocalPositionSetpoint> position_setpoint_;
        rclcpp::Time start_time_;
        std::optional<rclcpp::Time> airborne_since_;
        double hold_s_ = 1.0;
        double timeout_s_ = 8.0;
        double min_thrust_ = 0.1;
    };

} // namespace behavior
} // namespace iii_drone
