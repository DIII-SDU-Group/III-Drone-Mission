/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/mission/continuous_mission_orchestrator_node/continuous_mission_orchestrator_node_configurator.hpp"

using namespace iii_drone::mission::continuous_mission_orchestrator_node;
using namespace iii_drone::configuration;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ContinuousMissionOrchestratorConfigurator::ContinuousMissionOrchestratorConfigurator(
    rclcpp::Node *node,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

}

ContinuousMissionOrchestratorConfigurator::ContinuousMissionOrchestratorConfigurator(
    rclcpp::Node *node, 
    const rclcpp::QoS & qos,
    std::function<void(const rclcpp::Parameter &)> after_parameter_change_callback
) : Configurator(
        node, 
        qos,
        after_parameter_change_callback
    ) {

    declareNodeParameters();

}

const float ContinuousMissionOrchestratorConfigurator::under_cable_target_distance() const {

    return GetParameter("/mission/continuous_mission_orchestrator/under_cable_target_distance").as_double();

}

const bool ContinuousMissionOrchestratorConfigurator::use_charger_gripper_info() const {

    return GetParameter("/mission/continuous_mission_orchestrator/use_charger_gripper_info").as_bool();

}

const int ContinuousMissionOrchestratorConfigurator::cable_landing_max_retries() const {

    return GetParameter("/mission/continuous_mission_orchestrator/cable_landing_max_retries").as_int();

}

const float ContinuousMissionOrchestratorConfigurator::battery_voltage_low_threshold() const {

    return GetParameter("/mission/continuous_mission_orchestrator/battery_voltage_low_threshold").as_double();

}

const float ContinuousMissionOrchestratorConfigurator::battery_voltage_high_threshold() const {

    return GetParameter("/mission/continuous_mission_orchestrator/battery_voltage_high_threshold").as_double();

}

const float ContinuousMissionOrchestratorConfigurator::charging_power_low_threshold() const {

    return GetParameter("/mission/continuous_mission_orchestrator/charging_power_low_threshold").as_double();

}

const int ContinuousMissionOrchestratorConfigurator::action_timeout_ms() const {

    return GetParameter("/mission/continuous_mission_orchestrator/action_timeout_ms").as_int();

}

const int ContinuousMissionOrchestratorConfigurator::charging_minimum_duration_s() const {

    return GetParameter("/mission/continuous_mission_orchestrator/charging_minimum_duration_s").as_int();

}

const int ContinuousMissionOrchestratorConfigurator::charging_maximum_duration_s() const {

    return GetParameter("/mission/continuous_mission_orchestrator/charging_maximum_duration_s").as_int();

}

void ContinuousMissionOrchestratorConfigurator::declareNodeParameters() {

    DeclareParameter<float>("/mission/continuous_mission_orchestrator/under_cable_target_distance");
    DeclareParameter<bool>("/mission/continuous_mission_orchestrator/use_charger_gripper_info");
    DeclareParameter<int>("/mission/continuous_mission_orchestrator/cable_landing_max_retries");
    DeclareParameter<float>("/mission/continuous_mission_orchestrator/battery_voltage_low_threshold");
    DeclareParameter<float>("/mission/continuous_mission_orchestrator/battery_voltage_high_threshold");
    DeclareParameter<float>("/mission/continuous_mission_orchestrator/charging_power_low_threshold");
    DeclareParameter<int>("/mission/continuous_mission_orchestrator/action_timeout_ms");
    DeclareParameter<int>("/mission/continuous_mission_orchestrator/charging_minimum_duration_s");
    DeclareParameter<int>("/mission/continuous_mission_orchestrator/charging_maximum_duration_s");

}
