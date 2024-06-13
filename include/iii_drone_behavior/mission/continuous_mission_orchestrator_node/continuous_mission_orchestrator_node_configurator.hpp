#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {

namespace mission {

namespace continuous_mission_orchestrator_node {

    /**
     * @brief Class for handling parameters for ContinuousMissionOrchestrator.
    */
    class ContinuousMissionOrchestratorConfigurator : public configuration::Configurator {

    public:
        /**
         * @brief Constructor
         *
         * @param node Reference to the handling node
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        ContinuousMissionOrchestratorConfigurator(
            rclcpp::Node *node,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Constructor
         * 
         * @param node Reference to the handling node
         * @param qos QoS profile for the parameter event subscription
         * @param after_parameter_change_callback Callback function called after successful parameter change, default is nullptr
         */
        ContinuousMissionOrchestratorConfigurator(
            rclcpp::Node *node, 
            const rclcpp::QoS & qos,
            std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback = nullptr
        );

        /**
         * @brief Get the under cable target distance
         * 
         * @return Under cable target distance
         */
        const float under_cable_target_distance() const;

        /**
         * @brief Get the use charger gripper info flag
         * 
         * @return Use charger gripper info flag
         */
        const bool use_charger_gripper_info() const;

        /**
         * @brief Get the cable landing max retries
         * 
         * @return Cable landing max retries
         */
        const int cable_landing_max_retries() const;

        /**
         * @brief Get the battery voltage low threshold
         * 
         * @return Battery voltage low threshold
         */
        const float battery_voltage_low_threshold() const;

        /**
         * @brief Get the battery voltage high threshold
         * 
         * @return Battery voltage high threshold
         */
        const float battery_voltage_high_threshold() const;

        /**
         * @brief Get the charging power low threshold
         * 
         * @return Charging power low threshold
         */
        const float charging_power_low_threshold() const;

        /**
         * @brief Get the action timeout in milliseconds
         * 
         * @return Action timeout in milliseconds
         */
        const int action_timeout_ms() const;

        /**
         * @brief Get the charging minimum duration in seconds
         * 
         * @return Charging minimum duration in seconds
         */
        const int charging_minimum_duration_s() const;

        /**
         * @brief Get the charging maximum duration in seconds
         * 
         * @return Charging maximum duration in seconds
         */
        const int charging_maximum_duration_s() const;

    private:
        /**
         * @brief Declares the node specific parameters
         * 
         * @return void
         */
        void declareNodeParameters();

    };

} // namespace continuous_mission_orchestrator_node
} // namespace mission
} // namespace iii_drone
