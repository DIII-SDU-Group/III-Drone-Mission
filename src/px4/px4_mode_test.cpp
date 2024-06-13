#include <iii_drone_core/utils/history.hpp>

#include <iii_drone_configuration/configurator.hpp>
#include <iii_drone_configuration/parameter_bundle.hpp>

#include <iii_drone_core/adapters/px4/vehicle_odometry_adapter.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <rclcpp/rclcpp.hpp>

#include <iii_drone_core/behavior/trees/take_charging_position_tree.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace iii_drone::adapters::px4;
using namespace iii_drone::utils;
using namespace iii_drone::configuration;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::behavior;

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("px4_mode_test");

    History<VehicleOdometryAdapter>::SharedPtr vehicle_odometry_adapter_history = std::make_shared<History<VehicleOdometryAdapter>>();

	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        px4_sub_qos,
        [&](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            vehicle_odometry_adapter_history->Store(VehicleOdometryAdapter(*msg));
        }
    );

    ManeuverReferenceClient::SharedPtr maneuver_reference_client = std::make_shared<ManeuverReferenceClient>(
        node.get(),
        vehicle_odometry_adapter_history,
        false,
        5,
        20,
        [&]() -> void {
            RCLCPP_ERROR(node->get_logger(), "ManeuverReferenceClient::on_fail_during_maneuver(): Failed to get reference.");
        }
    );

    Configurator::SharedPtr configurator = std::make_shared<Configurator>(node.get());

    tf2_ros::Buffer::SharedPtr tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    TakeChargingPositionTree take_charging_position_tree(
        configurator,
        node.get(),
        tf_buffer,
        maneuver_reference_client
    );
    

    RCLCPP_INFO(node->get_logger(), "Spinning node...");

    // Multi threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}