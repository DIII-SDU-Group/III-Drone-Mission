/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/mode_executor_schedule_request_action_node.hpp>

using namespace iii_drone::behavior;
using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ModeExecutorScheduleRequestActionNode::ModeExecutorScheduleRequestActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::ModeExecutorScheduleRequest>(
        name, 
        conf, 
        params
),  node_ptr_(params.nh) { }

PortsList ModeExecutorScheduleRequestActionNode::providedPorts() {

    return providedBasicPorts({
        InputPort<mode_executor_schedule_request_t>("schedule_request_type"),
        InputPort<float>("takeoff_altitude")
    });

}

bool ModeExecutorScheduleRequestActionNode::setRequest(Request::SharedPtr & request) {

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "ModeExecutorScheduleRequestActionNode::setRequest(): Setting request."
    );

    mode_executor_schedule_request_t srt;

    if (!getInput("schedule_request_type", srt)) {

        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ModeExecutorScheduleRequestActionNode::setRequest(): No schedule request type provided."
        );

        return false;

    }

    switch(srt) {
        
        case MODE_EXECUTOR_SCHEDULE_REQUEST_CLEAR:

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorScheduleRequestActionNode::setRequest(): Setting schedule request clear."
            );

            request->request = iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_CLEAR;

            return true;

        case MODE_EXECUTOR_SCHEDULE_REQUEST_TAKEOFF:

            if (!getInput("takeoff_altitude", request->takeoff_altitude)) {

                RCLCPP_ERROR(
                    node_ptr_->get_logger(),
                    "ModeExecutorScheduleRequestActionNode::setRequest(): No takeoff altitude provided for schedule request type takeoff."
                );

                return false;

            }

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorScheduleRequestActionNode::setRequest(): Setting schedule request takeoff."
            );

            request->request = iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_TAKEOFF;

            return true;

        case MODE_EXECUTOR_SCHEDULE_REQUEST_LAND:

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorScheduleRequestActionNode::setRequest(): Setting schedule request land."
            );

            request->request = iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_LAND;

            return true;

        case MODE_EXECUTOR_SCHEDULE_REQUEST_ARM:

            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ModeExecutorScheduleRequestActionNode::setRequest(): Setting schedule request arm."
            );

            request->request = iii_drone_interfaces::srv::ModeExecutorScheduleRequest::Request::REQUEST_ARM;

            return true;

    }

    return false;

}

NodeStatus ModeExecutorScheduleRequestActionNode::onResponseReceived(const Response::SharedPtr & response) {

    if (response->success) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ModeExecutorScheduleRequestActionNode::onResponseReceived(): Success"
        );
        return NodeStatus::SUCCESS;
    } else {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ModeExecutorScheduleRequestActionNode::onResponseReceived(): Failure"
        );
        return NodeStatus::FAILURE;
    }

}