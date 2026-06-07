/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/rosbag_recording_action_nodes.hpp>

#include <sstream>
#include <future>
#include <utility>

using namespace iii_drone::behavior;
using namespace BT;

namespace {

std::vector<std::string> splitTopicList(const std::string & topics_csv) {
    std::vector<std::string> topics;
    std::stringstream stream(topics_csv);
    std::string item;

    while (std::getline(stream, item, ',')) {
        const auto first = item.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) {
            continue;
        }
        const auto last = item.find_last_not_of(" \t\n\r");
        topics.push_back(item.substr(first, last - first + 1));
    }

    return topics;
}

}  // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

StartRosbagRecordingActionNode::StartRosbagRecordingActionNode(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::StartRosbagRecording>(
        name,
        conf,
        params
),  node_ptr_(params.nh.lock()) { }

PortsList StartRosbagRecordingActionNode::providedPorts() {
    return providedBasicPorts({
        InputPort<std::string>("recording_id", std::string(""), "Recording id. Empty lets the recorder generate one."),
        InputPort<std::string>("output_dir", std::string(""), "Output directory. Empty uses the recorder artifact root."),
        InputPort<bool>("all_topics", true, "Record all topics."),
        InputPort<std::string>("topics", std::string(""), "Comma-separated topic list used when all_topics is false."),
        InputPort<bool>("include_hidden_topics", true, "Record hidden topics.")
    });
}

bool StartRosbagRecordingActionNode::setRequest(Request::SharedPtr & request) {
    getInput("recording_id", request->recording_id);
    getInput("output_dir", request->output_dir);
    getInput("all_topics", request->all_topics);
    getInput("include_hidden_topics", request->include_hidden_topics);

    std::string topics_csv;
    getInput("topics", topics_csv);
    request->topics = splitTopicList(topics_csv);
    return true;
}

NodeStatus StartRosbagRecordingActionNode::onResponseReceived(const Response::SharedPtr & response) {
    if (!response->success) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "StartRosbagRecordingActionNode::onResponseReceived(): failed: %s",
            response->message.c_str()
        );
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Started rosbag recording '%s' at %s",
        response->recording_id.c_str(),
        response->output_dir.c_str()
    );
    return NodeStatus::SUCCESS;
}

StopRosbagRecordingActionNode::StopRosbagRecordingActionNode(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params
) : RosServiceNode<iii_drone_interfaces::srv::StopRosbagRecording>(
        name,
        conf,
        params
),  node_ptr_(params.nh.lock()) { }

PortsList StopRosbagRecordingActionNode::providedPorts() {
    return providedBasicPorts({
        InputPort<std::string>("recording_id", std::string(""), "Recording id. Empty stops the active recording."),
        InputPort<double>("timeout_sec", 10.0, "Graceful stop timeout in seconds.")
    });
}

bool StopRosbagRecordingActionNode::setRequest(Request::SharedPtr & request) {
    getInput("recording_id", request->recording_id);
    getInput("timeout_sec", request->timeout_sec);
    return true;
}

NodeStatus StopRosbagRecordingActionNode::onResponseReceived(const Response::SharedPtr & response) {
    if (!response->success) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "StopRosbagRecordingActionNode::onResponseReceived(): failed: %s",
            response->message.c_str()
        );
        return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "Stopped rosbag recording '%s': %s",
        response->recording_id.c_str(),
        response->message.c_str()
    );
    return NodeStatus::SUCCESS;
}

RosbagRecordingScopeDecorator::RosbagRecordingScopeDecorator(
    const std::string & name,
    const NodeConfig & conf,
    rclcpp::Node::SharedPtr node,
    std::string start_service_name,
    std::string stop_service_name,
    std::chrono::milliseconds server_timeout,
    std::chrono::milliseconds wait_for_server_timeout
) : DecoratorNode(name, conf),
    node_(std::move(node)),
    server_timeout_(server_timeout),
    wait_for_server_timeout_(wait_for_server_timeout)
{
    start_client_ = node_->create_client<iii_drone_interfaces::srv::StartRosbagRecording>(start_service_name);
    stop_client_ = node_->create_client<iii_drone_interfaces::srv::StopRosbagRecording>(stop_service_name);
}

PortsList RosbagRecordingScopeDecorator::providedPorts() {
    return {
        InputPort<std::string>("recording_id", std::string(""), "Recording id. Empty lets the recorder generate one."),
        InputPort<std::string>("output_dir", std::string(""), "Output directory. Empty uses the recorder artifact root."),
        InputPort<bool>("all_topics", true, "Record all topics."),
        InputPort<std::string>("topics", std::string(""), "Comma-separated topic list used when all_topics is false."),
        InputPort<bool>("include_hidden_topics", true, "Record hidden topics."),
        InputPort<double>("stop_timeout_sec", 10.0, "Graceful stop timeout in seconds.")
    };
}

NodeStatus RosbagRecordingScopeDecorator::tick() {
    if (!recording_started_) {
        stopActiveRecording(false);
        if (!startRecording()) {
            return NodeStatus::FAILURE;
        }
    }

    setStatus(NodeStatus::RUNNING);
    const NodeStatus child_status = child_node_->executeTick();

    if (isStatusCompleted(child_status)) {
        const bool stopped = stopActiveRecording(true);
        resetChild();
        return stopped ? child_status : NodeStatus::FAILURE;
    }

    return child_status;
}

void RosbagRecordingScopeDecorator::halt() {
    if (recording_started_) {
        stopActiveRecording(false);
    }
    DecoratorNode::halt();
}

bool RosbagRecordingScopeDecorator::stopActiveRecording(bool require_success) {
    if (!stop_client_->wait_for_service(wait_for_server_timeout_)) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RosbagRecordingScopeDecorator::stopActiveRecording(): stop service unavailable"
        );
        return !require_success;
    }

    double timeout_sec = 10.0;
    getInput("stop_timeout_sec", timeout_sec);

    auto request = std::make_shared<iii_drone_interfaces::srv::StopRosbagRecording::Request>();
    request->timeout_sec = timeout_sec;

    auto future = stop_client_->async_send_request(request);
    if (future.wait_for(server_timeout_) != std::future_status::ready) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RosbagRecordingScopeDecorator::stopActiveRecording(): stop service timed out"
        );
        return !require_success;
    }

    const auto response = future.get();
    recording_started_ = false;

    if (!response->success) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RosbagRecordingScopeDecorator::stopActiveRecording(): stop failed: %s",
            response->message.c_str()
        );
        return !require_success;
    }

    return true;
}

bool RosbagRecordingScopeDecorator::startRecording() {
    if (!start_client_->wait_for_service(wait_for_server_timeout_)) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RosbagRecordingScopeDecorator::startRecording(): start service unavailable"
        );
        return false;
    }

    auto request = std::make_shared<iii_drone_interfaces::srv::StartRosbagRecording::Request>();
    getInput("recording_id", request->recording_id);
    getInput("output_dir", request->output_dir);
    getInput("all_topics", request->all_topics);
    getInput("include_hidden_topics", request->include_hidden_topics);
    std::string topics_csv;
    getInput("topics", topics_csv);
    request->topics = splitTopicList(topics_csv);

    auto future = start_client_->async_send_request(request);
    if (future.wait_for(server_timeout_) != std::future_status::ready) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RosbagRecordingScopeDecorator::startRecording(): start service timed out"
        );
        return false;
    }

    const auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "RosbagRecordingScopeDecorator::startRecording(): start failed: %s",
            response->message.c_str()
        );
        return false;
    }

    recording_started_ = true;
    RCLCPP_INFO(
        node_->get_logger(),
        "RosbagRecordingScopeDecorator::startRecording(): recording '%s' at %s",
        response->recording_id.c_str(),
        response->output_dir.c_str()
    );
    return true;
}
