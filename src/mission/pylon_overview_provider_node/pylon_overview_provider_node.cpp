#include <iii_drone_mission/mission/pylon_overview_provider_node/pylon_overview_provider_node.hpp>

#include <cmath>
#include <filesystem>

using namespace iii_drone::mission::pylon_overview_provider_node;

namespace {
constexpr std::size_t kRequiredPylonCount = 2;
constexpr double kMaximumCoordinateMagnitudeM = 10000.0;

bool finiteCoordinate(double value) {
    return std::isfinite(value) && std::abs(value) < kMaximumCoordinateMagnitudeM;
}
}

PylonOverviewProviderNode::PylonOverviewProviderNode(
    std::string node_name,
    std::string node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(node_name, node_namespace, options)
{
    const auto default_path = iii_drone::mission::overview_gnss::defaultOverviewDirectory() / "pylon_overview.yaml";
    declare_parameter<std::string>("gnss_persistence_path", default_path.string());
    declare_parameter<double>("capture_max_horizontal_speed_mps", 0.3);
    declare_parameter<double>("capture_stationary_dwell_s", 1.0);
    declare_parameter<double>("capture_pose_max_age_s", 0.5);
    gnss_persistence_path_ = get_parameter("gnss_persistence_path").as_string();
    has_persisted_gnss_pylons_ = std::filesystem::exists(gnss_persistence_path_);

    status_pub_ = create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "stored_pylon_status",
        10
    );
    overview_status_pub_ = create_publisher<iii_drone_interfaces::msg::PylonOverviewStatus>(
        "overview_status",
        10
    );

    status_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
            iii_drone_interfaces::msg::StringStamped msg;
            msg.stamp = rclcpp::Clock().now();

            std::lock_guard<std::mutex> lock(mutex_);
            if (validLocked()) {
                msg.data = "Pylon overview stored";
            } else if (has_persisted_gnss_pylons_) {
                msg.data = "Pylon overview stored on disk (GNSS)";
            } else {
                msg.data = "No valid pylon overview stored";
            }

            status_pub_->publish(msg);
            overview_status_pub_->publish(statusLocked());
        }
    );
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PylonOverviewProviderNode::on_configure(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PylonOverviewProviderNode::on_configure()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_configure(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return ret;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    gnss_persistence_path_ = get_parameter("gnss_persistence_path").as_string();
    capture_max_horizontal_speed_mps_ = get_parameter("capture_max_horizontal_speed_mps").as_double();
    capture_stationary_dwell_s_ = get_parameter("capture_stationary_dwell_s").as_double();
    capture_pose_max_age_s_ = get_parameter("capture_pose_max_age_s").as_double();
    has_persisted_gnss_pylons_ = std::filesystem::exists(gnss_persistence_path_);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pylons_.clear();
        overview_source_ = has_persisted_gnss_pylons_ ? "gnss_only_unavailable" : "none";
        stationary_dwell_active_ = false;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PylonOverviewProviderNode::on_cleanup(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PylonOverviewProviderNode::on_cleanup()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return ret;
    }

    tf_listener_.reset();
    tf_buffer_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PylonOverviewProviderNode::on_activate(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PylonOverviewProviderNode::on_activate()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_activate(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return ret;
    }

    store_pylon_overview_srv_ = create_service<iii_drone_interfaces::srv::StorePylonOverview>(
        "store_pylon_overview",
        std::bind(
            &PylonOverviewProviderNode::storePylonOverviewCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        )
    );

    get_pylon_overview_srv_ = create_service<iii_drone_interfaces::srv::GetPylonOverview>(
        "get_pylon_overview",
        std::bind(
            &PylonOverviewProviderNode::getPylonOverviewCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        )
    );

    clear_pylon_overview_srv_ = create_service<iii_drone_interfaces::srv::ClearPylonOverview>(
        "clear_pylon_overview",
        std::bind(
            &PylonOverviewProviderNode::clearPylonOverviewCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        )
    );
    capture_current_pylon_srv_ = create_service<iii_drone_interfaces::srv::CaptureCurrentPylon>(
        "capture_current_pylon",
        std::bind(
            &PylonOverviewProviderNode::captureCurrentPylonCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3
        )
    );

    vehicle_global_position_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position",
        rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) -> void
        {
            latest_global_position_.Store(*msg);
        }
    );
    vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) -> void
        {
            const auto now = std::chrono::steady_clock::now();
            const double horizontal_speed = std::hypot(msg->velocity[0], msg->velocity[1]);
            std::lock_guard<std::mutex> lock(mutex_);
            latest_vehicle_odometry_.Store(*msg);
            last_odometry_received_ = now;
            if (std::isfinite(horizontal_speed) && horizontal_speed <= capture_max_horizontal_speed_mps_) {
                if (!stationary_dwell_active_) {
                    stationary_since_ = now;
                    stationary_dwell_active_ = true;
                }
            } else {
                stationary_dwell_active_ = false;
            }
        }
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PylonOverviewProviderNode::on_deactivate(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PylonOverviewProviderNode::on_deactivate()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return ret;
    }

    store_pylon_overview_srv_.reset();
    get_pylon_overview_srv_.reset();
    clear_pylon_overview_srv_.reset();
    capture_current_pylon_srv_.reset();
    vehicle_global_position_sub_.reset();
    vehicle_odometry_sub_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PylonOverviewProviderNode::on_shutdown(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PylonOverviewProviderNode::on_shutdown()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);
    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
        return ret;
    }

    auto shutdown_thread = std::thread([]() {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    });
    shutdown_thread.detach();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PylonOverviewProviderNode::on_error(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_FATAL(get_logger(), "PylonOverviewProviderNode::on_error(): Lifecycle transition failed.");
    return rclcpp_lifecycle::LifecycleNode::on_error(state);
}

bool PylonOverviewProviderNode::validLocked() const
{
    return pylons_.size() == kRequiredPylonCount;
}

iii_drone_interfaces::msg::PylonOverview PylonOverviewProviderNode::overviewLocked() const
{
    iii_drone_interfaces::msg::PylonOverview overview;
    overview.stamp = rclcpp::Clock().now();
    overview.frame_id = frame_id_;
    overview.pylons.reserve(pylons_.size());

    for (const auto & item : pylons_) {
        overview.pylons.push_back(item.second);
    }

    return overview;
}

iii_drone_interfaces::msg::PylonOverviewStatus PylonOverviewProviderNode::statusLocked() const
{
    iii_drone_interfaces::msg::PylonOverviewStatus status;
    status.stamp = rclcpp::Clock().now();
    status.valid = validLocked();
    status.pylon_count = static_cast<uint32_t>(pylons_.size());
    status.overview_in_frame = !pylons_.empty();
    status.overview_gnss_only = pylons_.empty() && has_persisted_gnss_pylons_;
    status.overview_source = overview_source_;
    status.persistence_file_present = has_persisted_gnss_pylons_;
    status.overview = overviewLocked();
    for (const auto & item : pylons_) {
        status.pylon_ids.push_back(item.first);
    }
    if (!status.valid) {
        status.degraded_reason = status.overview_gnss_only
            ? "GNSS pylon data cannot be reprojected into the active world frame"
            : "pylon overview requires exactly two captured endpoints";
    }
    return status;
}

void PylonOverviewProviderNode::replaceOverviewLocked(
    const iii_drone_interfaces::msg::PylonOverview & overview
)
{
    frame_id_ = overview.frame_id;
    pylons_.clear();
    for (const auto & pylon : overview.pylons) {
        pylons_[pylon.id] = pylon;
    }
}

bool PylonOverviewProviderNode::persistOverview(
    const iii_drone_interfaces::msg::PylonOverview & overview
)
{
    const auto reference = iii_drone::mission::overview_gnss::makeReference(
        latest_global_position_.Load(),
        tf_buffer_,
        get_logger()
    );
    if (!reference.has_value()) {
        return false;
    }

    if (!iii_drone::mission::overview_gnss::persistPylonOverview(
            overview,
            reference.value(),
            gnss_persistence_path_,
            get_logger())) {
        return false;
    }

    has_persisted_gnss_pylons_ = true;
    RCLCPP_INFO(
        get_logger(),
        "PylonOverviewProviderNode: persisted GNSS pylon overview to %s",
        gnss_persistence_path_.string().c_str()
    );
    return true;
}

bool PylonOverviewProviderNode::loadPersistedOverviewToMemoryLocked()
{
    if (!has_persisted_gnss_pylons_ && !std::filesystem::exists(gnss_persistence_path_)) {
        return false;
    }

    const auto reference = iii_drone::mission::overview_gnss::makeReference(
        latest_global_position_.Load(),
        tf_buffer_,
        get_logger()
    );
    if (!reference.has_value()) {
        return false;
    }

    const auto loaded_overview = iii_drone::mission::overview_gnss::loadPylonOverview(
        gnss_persistence_path_,
        reference.value(),
        get_logger()
    );
    if (!loaded_overview.has_value()) {
        return false;
    }

    replaceOverviewLocked(loaded_overview.value());
    overview_source_ = "loaded_gnss_to_world";
    has_persisted_gnss_pylons_ = true;
    RCLCPP_INFO(
        get_logger(),
        "PylonOverviewProviderNode: loaded GNSS pylon overview from %s into current world frame",
        gnss_persistence_path_.string().c_str()
    );
    return true;
}

void PylonOverviewProviderNode::storePylonOverviewCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::StorePylonOverview::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::StorePylonOverview::Response> response
)
{
    (void)request_header;
    response->success = false;

    if (request->frame_id != "world") {
        response->message = "pylon overview storage currently requires frame_id=world";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
    }

    if (!finiteCoordinate(request->x) || !finiteCoordinate(request->y)) {
        response->message = "pylon coordinates must be finite and within configured bounds";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
    }

    iii_drone_interfaces::msg::Pylon pylon;
    pylon.id = request->id;
    pylon.x = request->x;
    pylon.y = request->y;

    iii_drone_interfaces::msg::PylonOverview candidate_overview;
    std::string previous_frame_id;
    std::string previous_source;
    std::map<int32_t, iii_drone_interfaces::msg::Pylon> previous_pylons;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        previous_frame_id = frame_id_;
        previous_source = overview_source_;
        previous_pylons = pylons_;
        frame_id_ = request->frame_id;
        pylons_[request->id] = pylon;
        overview_source_ = "external_world_input";
        candidate_overview = overviewLocked();
    }

    if (!persistOverview(candidate_overview)) {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_id_ = previous_frame_id;
        overview_source_ = previous_source;
        pylons_ = previous_pylons;
        response->message = "failed to persist pylon overview in GNSS coordinates";
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
    }

    response->stored_pylon_overview = candidate_overview;
    response->success = true;
    response->message = "pylon stored";

    RCLCPP_INFO(
        get_logger(),
        "PylonOverviewProviderNode::storePylonOverviewCallback(): Stored pylon id=%d x=%.3f y=%.3f",
        request->id,
        request->x,
        request->y
    );
}

void PylonOverviewProviderNode::captureCurrentPylonCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::CaptureCurrentPylon::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::CaptureCurrentPylon::Response> response
)
{
    (void)request_header;
    response->success = false;
    if (request->id != 1 && request->id != 2) {
        response->message = "pylon slot must be 1 or 2";
        return;
    }

    px4_msgs::msg::VehicleOdometry odometry;
    std::string previous_frame_id;
    std::string previous_source;
    std::map<int32_t, iii_drone_interfaces::msg::Pylon> previous_pylons;
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (last_odometry_received_ == std::chrono::steady_clock::time_point{} ||
            std::chrono::duration<double>(now - last_odometry_received_).count() > capture_pose_max_age_s_) {
            response->message = "aircraft pose/velocity is unavailable or stale";
            return;
        }
        if (!stationary_dwell_active_ ||
            std::chrono::duration<double>(now - stationary_since_).count() < capture_stationary_dwell_s_) {
            response->message = "horizontal speed must remain below the capture threshold for the full dwell time";
            return;
        }
        if (pylons_.count(request->id) > 0 && !request->replace_existing) {
            response->message = "pylon slot is occupied; explicit replacement confirmation is required";
            return;
        }
        odometry = latest_vehicle_odometry_.Load();
        if (!finiteCoordinate(odometry.position[0]) || !finiteCoordinate(odometry.position[1])) {
            response->message = "aircraft pose contains invalid coordinates";
            return;
        }
        previous_frame_id = frame_id_;
        previous_source = overview_source_;
        previous_pylons = pylons_;
        frame_id_ = "world";
        iii_drone_interfaces::msg::Pylon pylon;
        pylon.id = request->id;
        pylon.x = odometry.position[0];
        pylon.y = -odometry.position[1];
        pylons_[request->id] = pylon;
        overview_source_ = "operator_capture_memory_world";
        response->captured_pylon = pylon;
        response->stored_pylon_overview = overviewLocked();
    }

    if (!persistOverview(response->stored_pylon_overview)) {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_id_ = previous_frame_id;
        overview_source_ = previous_source;
        pylons_ = previous_pylons;
        response->message = "capture rejected because a fresh GNSS/TF persistence reference is unavailable";
        return;
    }

    response->success = true;
    response->message = "current aircraft position captured as pylon endpoint";
    response->captured_at = rclcpp::Clock().now();
    response->gnss_reference_valid = true;
    response->persistence_source = "operator_capture_gnss";
}

void PylonOverviewProviderNode::getPylonOverviewCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::GetPylonOverview::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::GetPylonOverview::Response> response
)
{
    (void)request_header;
    (void)request;

    std::lock_guard<std::mutex> lock(mutex_);
    const bool had_gnss_on_disk = has_persisted_gnss_pylons_ || std::filesystem::exists(gnss_persistence_path_);

    if (!validLocked()) {
        loadPersistedOverviewToMemoryLocked();
    }

    response->valid = validLocked();
    response->success = true;
    response->message = response->valid
        ? "valid pylon overview available"
        : "pylon overview requires exactly two pylons";
    response->overview_in_frame = response->valid;
    response->overview_gnss_only = !response->valid && had_gnss_on_disk;
    response->overview_source = response->valid
        ? overview_source_
        : (had_gnss_on_disk ? "gnss_only_unavailable" : overview_source_);
    response->stored_pylon_overview = overviewLocked();
}

void PylonOverviewProviderNode::clearPylonOverviewCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::ClearPylonOverview::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::ClearPylonOverview::Response> response
)
{
    (void)request_header;
    (void)request;

    std::lock_guard<std::mutex> lock(mutex_);
    pylons_.clear();
    overview_source_ = "none";
    if (std::filesystem::exists(gnss_persistence_path_)) {
        std::error_code error;
        std::filesystem::remove(gnss_persistence_path_, error);
        if (error) {
            RCLCPP_WARN(
                get_logger(),
                "Failed to remove persisted GNSS pylon overview %s: %s",
                gnss_persistence_path_.string().c_str(),
                error.message().c_str()
            );
        }
    }
    has_persisted_gnss_pylons_ = false;

    response->success = true;
    response->message = "pylon overview cleared";
    response->stored_pylon_overview = overviewLocked();

    RCLCPP_INFO(get_logger(), "PylonOverviewProviderNode::clearPylonOverviewCallback(): Cleared pylon overview");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PylonOverviewProviderNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
