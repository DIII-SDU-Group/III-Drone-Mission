/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/rosbag_recorder_node/rosbag_recorder_node.hpp>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cctype>
#include <ctime>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace iii_drone::mission::rosbag_recorder_node;

namespace {

std::string nowString() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&time, &tm);

    std::ostringstream stream;
    stream << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return stream.str();
}

bool redirectToFile(const std::filesystem::path & path, int fd) {
    const int out_fd = ::open(path.c_str(), O_CREAT | O_WRONLY | O_APPEND, 0644);
    if (out_fd < 0) {
        return false;
    }
    if (::dup2(out_fd, fd) < 0) {
        ::close(out_fd);
        return false;
    }
    ::close(out_fd);
    return true;
}

} // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

RosbagRecorderNode::RosbagRecorderNode(
    std::string node_name,
    std::string node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(node_name, node_namespace, options) {
    declare_parameter<std::string>("artifact_root", "/tmp/iii_drone/rosbags");
    declare_parameter<std::string>("log_root", "/tmp/iii_drone/rosbag_recorder/logs");
    declare_parameter<double>("default_stop_timeout_sec", 10.0);

    RCLCPP_INFO(get_logger(), "RosbagRecorderNode::RosbagRecorderNode()");
}

RosbagRecorderNode::~RosbagRecorderNode() {
    std::string message;
    bool was_running = false;
    stopRecording(default_stop_timeout_sec_, message, was_running);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosbagRecorderNode::on_configure(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "RosbagRecorderNode::on_configure()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_configure(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return ret;
    }

    artifact_root_ = get_parameter("artifact_root").as_string();
    log_root_ = get_parameter("log_root").as_string();
    default_stop_timeout_sec_ = get_parameter("default_stop_timeout_sec").as_double();

    try {
        std::filesystem::create_directories(artifact_root_);
        std::filesystem::create_directories(log_root_);
    } catch (const std::filesystem::filesystem_error & e) {
        RCLCPP_ERROR(
            get_logger(),
            "RosbagRecorderNode::on_configure(): Failed to create recorder directories: %s",
            e.what()
        );
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosbagRecorderNode::on_cleanup(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "RosbagRecorderNode::on_cleanup()");

    std::string message;
    bool was_running = false;
    stopRecording(default_stop_timeout_sec_, message, was_running);

    start_recording_srv_.reset();
    stop_recording_srv_.reset();
    recording_status_srv_.reset();

    return rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosbagRecorderNode::on_activate(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "RosbagRecorderNode::on_activate()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_activate(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return ret;
    }

    start_recording_srv_ = create_service<iii_drone_interfaces::srv::StartRosbagRecording>(
        "start_recording",
        std::bind(&RosbagRecorderNode::startRecordingCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
    stop_recording_srv_ = create_service<iii_drone_interfaces::srv::StopRosbagRecording>(
        "stop_recording",
        std::bind(&RosbagRecorderNode::stopRecordingCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );
    recording_status_srv_ = create_service<iii_drone_interfaces::srv::GetRosbagRecordingStatus>(
        "recording_status",
        std::bind(&RosbagRecorderNode::recordingStatusCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosbagRecorderNode::on_deactivate(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "RosbagRecorderNode::on_deactivate()");

    std::string message;
    bool was_running = false;
    stopRecording(default_stop_timeout_sec_, message, was_running);

    start_recording_srv_.reset();
    stop_recording_srv_.reset();
    recording_status_srv_.reset();

    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosbagRecorderNode::on_shutdown(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "RosbagRecorderNode::on_shutdown()");

    std::string message;
    bool was_running = false;
    stopRecording(default_stop_timeout_sec_, message, was_running);

    return rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RosbagRecorderNode::on_error(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_FATAL(get_logger(), "RosbagRecorderNode::on_error(): Lifecycle transition failed.");

    std::string message;
    bool was_running = false;
    stopRecording(default_stop_timeout_sec_, message, was_running);

    return rclcpp_lifecycle::LifecycleNode::on_error(state);
}

void RosbagRecorderNode::startRecordingCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::StartRosbagRecording::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::StartRosbagRecording::Response> response
) {
    (void)request_header;

    if (isRecording()) {
        response->success = true;
        response->message = "reusing active rosbag recording";
        response->recording_id = recording_id_;
        response->output_dir = output_dir_.string();
        response->pid = static_cast<std::uint32_t>(child_pid_);
        response->was_running = true;
        return;
    }

    if (!request->all_topics && request->topics.empty()) {
        response->success = false;
        response->message = "no topics requested";
        return;
    }

    const auto prefix = request->recording_id.empty() ? (request->owner.empty() ? "recording" : request->owner) : request->recording_id;
    const auto base_recording_id = makeRecordingId(prefix);
    recording_id_ = base_recording_id;
    for (std::size_t suffix = 2; std::filesystem::exists(artifact_root_ / recording_id_); ++suffix) {
        recording_id_ = base_recording_id + "_" + std::to_string(suffix);
    }
    output_dir_ = artifact_root_ / recording_id_;
    started_at_ = nowString();
    owner_ = request->owner.empty() ? "unknown" : request->owner;
    last_error_.clear();

    const auto log_dir = log_root_ / recording_id_;
    try {
        std::filesystem::create_directories(output_dir_.parent_path());
        std::filesystem::create_directories(log_dir);
    } catch (const std::filesystem::filesystem_error & e) {
        response->success = false;
        response->message = std::string("failed to create recording directories: ") + e.what();
        last_error_ = response->message;
        response->recording_id = recording_id_;
        response->output_dir = output_dir_.string();
        clearRecordingState();
        return;
    }

    std::vector<std::string> args = {"ros2", "bag", "record"};
    if (request->all_topics) {
        args.push_back("--all");
    } else {
        for (const auto & topic : request->topics) {
            args.push_back(topic);
        }
    }
    if (request->include_hidden_topics) {
        args.push_back("--include-hidden-topics");
    }
    args.push_back("-o");
    args.push_back(output_dir_.string());

    child_pid_ = ::fork();
    if (child_pid_ < 0) {
        response->success = false;
        response->message = "fork failed";
        last_error_ = response->message;
        clearRecordingState();
        return;
    }

    if (child_pid_ == 0) {
        ::setsid();
        redirectToFile(log_dir / "stdout.log", STDOUT_FILENO);
        redirectToFile(log_dir / "stderr.log", STDERR_FILENO);

        std::vector<char *> argv;
        argv.reserve(args.size() + 1);
        for (auto & arg : args) {
            argv.push_back(arg.data());
        }
        argv.push_back(nullptr);

        ::execvp(argv[0], argv.data());
        _exit(127);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    int status = 0;
    if (::waitpid(child_pid_, &status, WNOHANG) == child_pid_) {
        response->success = false;
        response->message = "ros2 bag process exited immediately";
        last_error_ = response->message;
        clearRecordingState();
        return;
    }

    response->success = true;
    response->message = "recording started";
    response->recording_id = recording_id_;
    response->output_dir = output_dir_.string();
    response->pid = static_cast<std::uint32_t>(child_pid_);
    response->was_running = false;

    RCLCPP_INFO(
        get_logger(),
        "Started rosbag recording '%s' at %s (pid=%d)",
        recording_id_.c_str(),
        output_dir_.c_str(),
        child_pid_
    );
}

void RosbagRecorderNode::stopRecordingCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::StopRosbagRecording::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::StopRosbagRecording::Response> response
) {
    (void)request_header;

    response->recording_id = recording_id_;
    response->output_dir = output_dir_.string();

    if (!request->recording_id.empty() && request->recording_id != recording_id_) {
        response->success = false;
        response->message = "requested recording id does not match active recording";
        response->was_running = isRecording();
        return;
    }

    bool was_running = false;
    std::string message;
    const double timeout_sec = request->timeout_sec > 0.0 ? request->timeout_sec : default_stop_timeout_sec_;
    response->success = stopRecording(timeout_sec, message, was_running);
    response->message = message;
    response->was_running = was_running;
}

void RosbagRecorderNode::recordingStatusCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::GetRosbagRecordingStatus::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::GetRosbagRecordingStatus::Response> response
) {
    (void)request_header;
    (void)request;
    fillStatus(*response);
}

bool RosbagRecorderNode::isRecording() {
    if (child_pid_ <= 0) {
        return false;
    }

    int status = 0;
    const pid_t result = ::waitpid(child_pid_, &status, WNOHANG);
    if (result == 0) {
        return true;
    }
    if (result == child_pid_) {
        RCLCPP_WARN(get_logger(), "Rosbag recording process exited unexpectedly.");
        clearRecordingState();
        return false;
    }
    return false;
}

bool RosbagRecorderNode::stopRecording(double timeout_sec, std::string & message, bool & was_running) {
    was_running = isRecording();
    if (!was_running) {
        message = "no active recording";
        clearRecordingState();
        return true;
    }

    const pid_t pid = child_pid_;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));

    ::kill(-pid, SIGINT);

    int status = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const pid_t result = ::waitpid(pid, &status, WNOHANG);
        if (result == pid) {
            message = "recording stopped";
            clearRecordingState();
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ::kill(-pid, SIGTERM);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (::waitpid(pid, &status, WNOHANG) == pid) {
        message = "recording stopped after SIGTERM";
        clearRecordingState();
        return true;
    }

    ::kill(-pid, SIGKILL);
    ::waitpid(pid, &status, 0);
    message = "recording killed after timeout";
    clearRecordingState();
    return true;
}

void RosbagRecorderNode::clearRecordingState() {
    child_pid_ = -1;
    recording_id_.clear();
    output_dir_.clear();
    started_at_.clear();
}

std::string RosbagRecorderNode::makeRecordingId(const std::string & prefix) const {
    return sanitizeRecordingId(prefix) + "_" + nowString();
}

std::string RosbagRecorderNode::sanitizeRecordingId(const std::string & recording_id) const {
    std::string sanitized;
    sanitized.reserve(recording_id.size());
    for (const char c : recording_id) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
            sanitized.push_back(c);
        } else {
            sanitized.push_back('_');
        }
    }
    return sanitized.empty() ? "recording" : sanitized;
}

std::uint64_t RosbagRecorderNode::outputSizeBytes() const {
    if (output_dir_.empty() || !std::filesystem::exists(output_dir_)) {
        return 0;
    }

    std::uint64_t size = 0;
    for (const auto & entry : std::filesystem::recursive_directory_iterator(output_dir_)) {
        if (entry.is_regular_file()) {
            size += entry.file_size();
        }
    }
    return size;
}

void RosbagRecorderNode::fillStatus(iii_drone_interfaces::srv::GetRosbagRecordingStatus::Response & response) {
    response.recording = isRecording();
    response.recording_id = recording_id_;
    response.output_dir = output_dir_.string();
    response.artifact_root = artifact_root_.string();
    response.pid = child_pid_ > 0 ? static_cast<std::uint32_t>(child_pid_) : 0;
    response.started_at = started_at_;
    response.size_bytes = outputSizeBytes();
    try {
        const auto probe = output_dir_.empty() ? artifact_root_ : output_dir_.parent_path();
        response.free_space_bytes = std::filesystem::space(probe).available;
    } catch (const std::filesystem::filesystem_error & error) {
        response.free_space_bytes = 0;
        last_error_ = std::string("failed to query rosbag free space: ") + error.what();
    }
    response.owner = owner_;
    response.error = last_error_;
    response.message = response.recording ? "recording active" : "no active recording";
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<RosbagRecorderNode>();

    executor.add_node(node->get_node_base_interface());

    try {
        executor.spin();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(node->get_logger(), "RosbagRecorderNode main loop failed: %s", e.what());
        node.reset();
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return 0;
}
