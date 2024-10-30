/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/px4/px4_handler.hpp>

using namespace iii_drone::px4;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PX4Handler::PX4Handler(
    iii_drone::mission::MissionSpecification::SharedPtr mission_specification,
    iii_drone::behavior::TreeProvider::SharedPtr tree_provider,
    float dt,
    rclcpp_lifecycle::LifecycleNode * node,
    iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    iii_drone::configuration::ParameterBundle::SharedPtr mode_provider_parameters,
    iii_drone::configuration::ParameterBundle::SharedPtr mode_executor_parameters,
    rclcpp::executors::MultiThreadedExecutor & executor
) {

    mission_specification_ = mission_specification;
    tree_provider_ = tree_provider;
    node_ = node;
    maneuver_reference_client_ = maneuver_reference_client;
    mode_provider_parameters_ = mode_provider_parameters;
    mode_executor_parameters_ = mode_executor_parameters;

    // Mode provider
    mode_provider_ = std::make_shared<iii_drone::px4::ModeProvider>(
        tree_provider_,
        mission_specification_,
        node_,
        dt
    );

    mode_provider_->Configure(
        maneuver_reference_client_,
        mode_provider_parameters_
    );
        


}