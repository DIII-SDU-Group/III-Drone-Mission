/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/get_powerline_overview_action_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GetPowerlineOverviewActionNode::GetPowerlineOverviewActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::GetPowerlineOverview>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh) { }

PortsList GetPowerlineOverviewActionNode::providedPorts() {

    return providedBasicPorts({
        OutputPort<iii_drone_interfaces::msg::Powerline>("stored_powerline")
    });

}

bool GetPowerlineOverviewActionNode::setRequest(Request::SharedPtr & request) {

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "GetPowerlineOverviewActionNode::setRequest(): Setting request"
    );

    return true;

}

NodeStatus GetPowerlineOverviewActionNode::onResponseReceived(const Response::SharedPtr & response) {

    if (!response->success) {

        RCLCPP_WARN(
            node_ptr_->get_logger(),
            "GetPowerlineOverviewActionNode::onResponseReceived(): Failed to get powerline overview"
        );

        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "GetPowerlineOverviewActionNode::onResponseReceived(): Powerline overview received"
    );

    setOutput("stored_powerline", response->stored_powerline);

    return NodeStatus::SUCCESS;

}