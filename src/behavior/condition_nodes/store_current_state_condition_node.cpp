/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/store_current_state_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::control;
using namespace iii_drone::adapters::px4;
using namespace iii_drone::types;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

StoreCurrentStateConditionNode::StoreCurrentStateConditionNode(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params
) : BT::RosTopicSubNode<px4_msgs::msg::VehicleOdometry>(
    name, 
    conf, 
    params,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().best_effort()
) { }

BT::PortsList StoreCurrentStateConditionNode::providedPorts() {
    return {
        BT::OutputPort<State>("current_state", "Current state"),
        BT::OutputPort<point_t>("current_position", "Current position"),
        BT::OutputPort<float>("current_yaw", "Current yaw")
    };
}

BT::NodeStatus StoreCurrentStateConditionNode::onTick(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> & msg) {

    if (!msg) {
        RCLCPP_ERROR(logger(), "StoreCurrentStateConditionNode::onTick(): Message is null");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(logger(), "StoreCurrentStateConditionNode::onTick(): Received message");

    VehicleOdometryAdapter adapter(*msg);

    State current_state = adapter.ToState();

    setOutput("current_state", current_state);
    setOutput("current_position", current_state.position());
    setOutput("current_yaw", (float)current_state.yaw());

    return BT::NodeStatus::SUCCESS;

}
