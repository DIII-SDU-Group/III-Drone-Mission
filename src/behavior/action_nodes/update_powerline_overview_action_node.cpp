/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/update_powerline_overview_action_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

UpdatePowerlineOverviewActionNode::UpdatePowerlineOverviewActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::UpdatePowerlineOverview>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh.lock()) { }

bool UpdatePowerlineOverviewActionNode::setRequest(Request::SharedPtr & request) {
    (void)request;

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "UpdatePowerlineOverviewActionNode::setRequest(): Setting request"
    );

    return true;

}

NodeStatus UpdatePowerlineOverviewActionNode::onResponseReceived(const Response::SharedPtr & response) {

    if (response->success) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "UpdatePowerlineOverviewActionNode::onResponseReceived(): Success"
        );
        return NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "UpdatePowerlineOverviewActionNode::onResponseReceived(): Failure"
        );
        return NodeStatus::FAILURE;
    }

}
