/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/log_message_action_node.hpp>

using namespace BT;
using namespace iii_drone::behavior;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

LogMessageActionNode::LogMessageActionNode(
    const std::string & name,
    const NodeConfig& config,
    std::shared_ptr<rclcpp::Node> node
) : SyncActionNode(
        name, 
        config
    ),
    node_(node) {}

PortsList LogMessageActionNode::providedPorts() {
    return {
        InputPort<std::string>("message"),
        InputPort<log_level_t>("log_level")
    };
}

NodeStatus LogMessageActionNode::tick() {

    std::string message;
    log_level_t log_level;

    getInput("message", message);

    getInput("log_level", log_level);

    switch(log_level) {
        case log_level_t::LOG_LEVEL_DEBUG:
            RCLCPP_DEBUG(node_->get_logger(), message.c_str());
            break;
        case log_level_t::LOG_LEVEL_INFO:
            RCLCPP_INFO(node_->get_logger(), message.c_str());
            break;
        case log_level_t::LOG_LEVEL_WARN:
            RCLCPP_WARN(node_->get_logger(), message.c_str());
            break;
        case log_level_t::LOG_LEVEL_ERROR:
            RCLCPP_ERROR(node_->get_logger(), message.c_str());
            break;
        case log_level_t::LOG_LEVEL_FATAL:
            RCLCPP_FATAL(node_->get_logger(), message.c_str());
            break;
        default:
            return NodeStatus::FAILURE;
    }

    return NodeStatus::SUCCESS;

}