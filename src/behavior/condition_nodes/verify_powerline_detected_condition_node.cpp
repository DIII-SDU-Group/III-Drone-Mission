/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/condition_nodes/verify_powerline_detected_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

VerifyPowerlineDetectedConditionNode::VerifyPowerlineDetectedConditionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosTopicSubNode<iii_drone_interfaces::msg::Powerline>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh) { }

PortsList VerifyPowerlineDetectedConditionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<unsigned int>("required_n_lines"),
        OutputPort<unsigned int>("n_lines")
    });

}

NodeStatus VerifyPowerlineDetectedConditionNode::onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) {

    if (!last_msg) {
        setOutput("n_lines", 0);
        return NodeStatus::FAILURE;
    }

    unsigned int required_n_lines;
    unsigned int n_lines;

    getInput("required_n_lines", required_n_lines);

    n_lines = last_msg->lines.size();

    setOutput("n_lines", n_lines);

    if (n_lines >= required_n_lines) {
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "VerifyPowerlineDetectedConditionNode::onTick(): %s: Powerline detected",
            name().c_str()
        );
        return NodeStatus::SUCCESS;
    } else {
        return NodeStatus::FAILURE;
    }

}