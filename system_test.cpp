/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configurator.hpp>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/mission/mission_executor.hpp>

/*****************************************************************************/
// Namespaces:

using namespace iii_drone::configuration;
using namespace iii_drone::mission;

/*****************************************************************************/
// Main:
/*****************************************************************************/

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("mission_executor");

    // ModeTest mode_test(*node, "test_mode");

    // RCLCPP_INFO(node->get_logger(), "Registering mode...");

    // mode_test.doRegister();

    Configurator<rclcpp::Node>::SharedPtr configurator = std::make_shared<Configurator<rclcpp::Node>>(node.get(), node->get_name());
    configurator->DeclareParameter("/mission/mission_specification_file", rclcpp::ParameterType::PARAMETER_STRING);
    configurator->DeclareParameter("/mission/use_nans_when_hovering", rclcpp::ParameterType::PARAMETER_BOOL);
    configurator->DeclareParameter("/mission/max_failed_attempts_during_maneuver", rclcpp::ParameterType::PARAMETER_INTEGER);
    configurator->DeclareParameter("/mission/wait_for_maneuver_start_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER);
    configurator->DeclareParameter("/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE);
    configurator->DeclareParameter("/mission/get_reference_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER);
    configurator->DeclareParameter("/mission/manual_stick_input_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE);
    configurator->DeclareParameter("/mission/mission_done_select_mode", rclcpp::ParameterType::PARAMETER_STRING);
    configurator->CreateConfiguration("maneuver_reference_client", {
        iii_drone::configuration::configuration_entry_t("/mission/use_nans_when_hovering", rclcpp::ParameterType::PARAMETER_BOOL),
        iii_drone::configuration::configuration_entry_t("/mission/max_failed_attempts_during_maneuver", rclcpp::ParameterType::PARAMETER_INTEGER),
        iii_drone::configuration::configuration_entry_t("/mission/wait_for_maneuver_start_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER),
        iii_drone::configuration::configuration_entry_t("/mission/get_reference_timeout_ms", rclcpp::ParameterType::PARAMETER_INTEGER),
    });
    configurator->CreateConfiguration("mode_provider", {
        iii_drone::configuration::configuration_entry_t("/control/dt", rclcpp::ParameterType::PARAMETER_DOUBLE),
    });
    configurator->CreateConfiguration("mode_executor", {
        iii_drone::configuration::configuration_entry_t("/mission/manual_stick_input_threshold", rclcpp::ParameterType::PARAMETER_DOUBLE),
        iii_drone::configuration::configuration_entry_t("/mission/mission_done_select_mode", rclcpp::ParameterType::PARAMETER_STRING),
    });
    configurator->validate();

    tf2_ros::Buffer::SharedPtr tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    MissionExecutor mission_executor(
        node.get(), 
        configurator, 
        tf_buffer
    );

    RCLCPP_INFO(node->get_logger(), "Spinning node...");

    // Multi threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    mission_executor.FinalizeInitialization(executor);

    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Spinning executor...");

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
