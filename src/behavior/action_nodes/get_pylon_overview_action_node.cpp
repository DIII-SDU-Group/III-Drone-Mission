/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/get_pylon_overview_action_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GetPylonOverviewActionNode::GetPylonOverviewActionNode(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::GetPylonOverview>(
        name,
        conf,
        params
),  node_ptr_(params.nh.lock()) { }

PortsList GetPylonOverviewActionNode::providedPorts() {
    return providedBasicPorts({
        OutputPort<iii_drone_interfaces::msg::PylonOverview>("stored_pylon_overview")
    });
}

bool GetPylonOverviewActionNode::setRequest(Request::SharedPtr & request) {
    (void)request;
    RCLCPP_INFO(node_ptr_->get_logger(), "GetPylonOverviewActionNode::setRequest(): Setting request");
    return true;
}

NodeStatus GetPylonOverviewActionNode::onResponseReceived(const Response::SharedPtr & response) {
    if (!response->success || !response->valid) {
        RCLCPP_WARN(
            node_ptr_->get_logger(),
            "GetPylonOverviewActionNode::onResponseReceived(): Failed to get valid pylon overview: %s",
            response->message.c_str()
        );
        return NodeStatus::FAILURE;
    }

    setOutput("stored_pylon_overview", response->stored_pylon_overview);
    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "GetPylonOverviewActionNode::onResponseReceived(): Pylon overview received with %lu pylon(s)",
        response->stored_pylon_overview.pylons.size()
    );
    return NodeStatus::SUCCESS;
}
