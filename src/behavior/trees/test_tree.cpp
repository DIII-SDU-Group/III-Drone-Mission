#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <iii_drone_core/behavior/action_nodes/hover_by_object_maneuver_action_node.hpp>
#include <iii_drone_core/behavior/action_nodes/hover_maneuver_action_node.hpp>
#include <iii_drone_core/control/maneuver/maneuver_reference_client.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;

    auto node = std::make_shared<rclcpp::Node>("test_tree");

    iii_drone::control::maneuver::ManeuverReferenceClient::SharedPtr maneuver_reference_client = std::make_shared<iii_drone::control::maneuver::ManeuverReferenceClient>(
        node.get(),
        nullptr,
        false,
        1,
        5,
        nullptr
    );

    BT::RosNodeParams params;

    params.nh = node;
    params.default_port_value = "/control/maneuver_controller/hover_by_object";
    params.server_timeout = std::chrono::milliseconds(5000);
    params.wait_for_server_timeout = std::chrono::milliseconds(5000);

    BT::NodeConfig config;

    factory.registerNodeType<iii_drone::behavior::HoverByObjectManeuverActionNode>(
        "HoverByObject",
        params,
        maneuver_reference_client
    );

    auto tree = factory.createTreeFromFile("/home/ffn/Workspace/ros2_humble_ws/src/III-Drone-Core/behavior_trees/test_tree.xml");

    BT::NodeStatus status = tree.tickWhileRunning();

    rclcpp::shutdown();

    std::cout << "Status: " << status << std::endl;

    return 0;
}