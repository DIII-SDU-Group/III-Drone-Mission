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

// #include <px4_ros2/components/mode.hpp>
// #include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

// class ModeTest : public px4_ros2::ModeBase {
// public:
//     ModeTest(
//         rclcpp::Node & node,
//         std::string mode_name
//     ) : px4_ros2::ModeBase(
//             node, 
//             Settings(
//                 mode_name,
//                 true
//             )
//     ), mode_name_(mode_name) {
//         RCLCPP_INFO(node.get_logger(), "ModeTest::ModeTest(): Initializing mode %s", mode_name.c_str());

//         _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
//     }

//     void onActivate() {

//         RCLCPP_INFO(node().get_logger(), "ModeTest::onActivate(): Activating mode %s", mode_name_.c_str());

//     }

//     void onDeactivate() {

//         RCLCPP_INFO(node().get_logger(), "ModeTest::onDeactivate(): Deactivating mode %s", mode_name_.c_str());

//     }

//     private:
//         std::string mode_name_;

//         std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
// };

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("mission_executor");

    // ModeTest mode_test(*node, "test_mode");

    // RCLCPP_INFO(node->get_logger(), "Registering mode...");

    // mode_test.doRegister();

    Configurator<rclcpp::Node>::SharedPtr configurator = std::make_shared<Configurator<rclcpp::Node>>(node.get());

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