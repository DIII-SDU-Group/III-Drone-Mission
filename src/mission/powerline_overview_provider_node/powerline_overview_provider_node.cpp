/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/powerline_overview_provider_node/powerline_overview_provider_node.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>

using namespace iii_drone::mission::powerline_overview_provider_node;
using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

namespace {
constexpr std::size_t kMinimumPowerlineOverviewLines = 4;
constexpr std::size_t kRequiredStablePowerlineOverviewSamples = 5;
constexpr double kMaxStableLinePositionDeltaM = 0.30;
constexpr double kMinimumPowerlineOverviewVerticalSpanM = 2.0;
constexpr double kMaximumPowerlineOverviewVerticalSpanM = 12.0;
constexpr double kMaximumPowerlineOverviewCoordinateMagnitudeM = 100.0;

bool isFinitePoint(const point_t & point) {
    return std::isfinite(point.x())
        && std::isfinite(point.y())
        && std::isfinite(point.z())
        && std::abs(point.x()) < kMaximumPowerlineOverviewCoordinateMagnitudeM
        && std::abs(point.y()) < kMaximumPowerlineOverviewCoordinateMagnitudeM
        && std::abs(point.z()) < kMaximumPowerlineOverviewCoordinateMagnitudeM;
}

bool isOverviewGeometryPlausible(const PowerlineAdapter & adapter, std::string & reason) {
    std::vector<SingleLineAdapter> visible_lines = adapter.GetVisibleLineAdapters();
    if (visible_lines.size() < kMinimumPowerlineOverviewLines) {
        reason = "not enough visible lines";
        return false;
    }

    if (!std::isfinite(adapter.projection_plane().normal.x())
        || !std::isfinite(adapter.projection_plane().normal.y())
        || !std::isfinite(adapter.projection_plane().normal.z())
        || adapter.projection_plane().normal.norm() < 0.5) {
        reason = "invalid projection-plane normal";
        return false;
    }

    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const SingleLineAdapter & line : visible_lines) {
        if (!isFinitePoint(line.position()) || !isFinitePoint(line.projected_position())) {
            reason = "non-finite line point";
            return false;
        }
        min_z = std::min(min_z, static_cast<double>(line.position().z()));
        max_z = std::max(max_z, static_cast<double>(line.position().z()));
    }

    const double vertical_span = max_z - min_z;
    if (vertical_span < kMinimumPowerlineOverviewVerticalSpanM) {
        reason = "vertical span too small";
        return false;
    }
    if (vertical_span > kMaximumPowerlineOverviewVerticalSpanM) {
        reason = "vertical span too large";
        return false;
    }

    reason = "";
    return true;
}

bool isOverviewStableAgainstPrevious(
    const PowerlineAdapter & previous,
    const PowerlineAdapter & current,
    double & max_delta
) {
    max_delta = 0.0;
    std::vector<SingleLineAdapter> current_visible_lines = current.GetVisibleLineAdapters();

    for (const SingleLineAdapter & current_line : current_visible_lines) {
        if (!previous.HasLine(current_line.id())) {
            return false;
        }

        const SingleLineAdapter previous_line = previous.GetLine(current_line.id());
        const double delta = (current_line.position() - previous_line.position()).norm();
        max_delta = std::max(max_delta, delta);

        if (delta > kMaxStableLinePositionDeltaM) {
            return false;
        }
    }

    return true;
}
}

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineOverviewProviderNode::PowerlineOverviewProviderNode(
    std::string node_name,
    std::string node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(node_name, node_namespace, options)
{
    const auto default_path = iii_drone::mission::overview_gnss::defaultOverviewDirectory() / "powerline_overview.yaml";
    declare_parameter<std::string>("gnss_persistence_path", default_path.string());
    gnss_persistence_path_ = get_parameter("gnss_persistence_path").as_string();
    has_persisted_gnss_powerline_ = std::filesystem::exists(gnss_persistence_path_);

    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	const char * log_level_env = std::getenv("POWERLINE_OVERVIEW_PROVIDER_LOG_LEVEL");
	std::string log_level = log_level_env == nullptr ? "" : log_level_env;

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			set_logger_level(RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			set_logger_level(RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			set_logger_level(RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			set_logger_level(RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			set_logger_level(RCUTILS_LOG_SEVERITY_FATAL);
		}

	}

    cb_group_1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    stored_powerline_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "stored_powerline_points",
        10
    );

    stored_powerline_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "stored_powerline_pose",
        10
    );

    stored_powerline_status_pub_ = create_publisher<iii_drone_interfaces::msg::StringStamped>(
        "stored_powerline_status",
        10
    );

    stored_powerline_status_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() -> void {

            iii_drone_interfaces::msg::StringStamped status_msg;
            status_msg.stamp = rclcpp::Clock().now();
            if (has_stored_powerline_) {
                status_msg.data = "Powerline stored";
            } else if (has_persisted_gnss_powerline_) {
                status_msg.data = "Powerline stored on disk (GNSS)";
            } else {
                status_msg.data = "No powerline stored";
            }

            stored_powerline_status_pub_->publish(status_msg);

        }
    );

}

PowerlineOverviewProviderNode::~PowerlineOverviewProviderNode()
{

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PowerlineOverviewProviderNode::on_configure(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::on_configure()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_configure(state);

    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    gnss_persistence_path_ = get_parameter("gnss_persistence_path").as_string();
    has_persisted_gnss_powerline_ = std::filesystem::exists(gnss_persistence_path_);
    has_stored_powerline_ = false;

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PowerlineOverviewProviderNode::on_cleanup(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::on_cleanup()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    tf_buffer_.reset();
    tf_listener_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PowerlineOverviewProviderNode::on_activate(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::on_activate()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_activate(state);

    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    pl_mapper_command_client_ = create_client<iii_drone_interfaces::srv::PLMapperCommand>(
        "/perception/pl_mapper/pl_mapper_command",
        rclcpp::ServicesQoS(),
        cb_group_1_
    );

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_1_;

    powerline_sub_ = create_subscription<iii_drone_interfaces::msg::Powerline>(
        "/perception/pl_mapper/powerline",
        10,
        [this](const iii_drone_interfaces::msg::Powerline::SharedPtr msg) -> void
        {
            latest_powerline_.Store(*msg);
        },
        sub_options
    );

    vehicle_global_position_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
        "/fmu/out/vehicle_global_position",
        rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) -> void
        {
            latest_global_position_.Store(*msg);
        },
        sub_options
    );

    update_powerline_overview_srv_ = create_service<iii_drone_interfaces::srv::UpdatePowerlineOverview>(
        "update_powerline_overview",
        std::bind(&PowerlineOverviewProviderNode::updatePowerlineOverviewCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );

    get_powerline_overview_srv_ = create_service<iii_drone_interfaces::srv::GetPowerlineOverview>(
        "get_powerline_overview",
        std::bind(&PowerlineOverviewProviderNode::getPowerlineOverviewCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );

    stored_powerline_points_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]() -> void {

            if (!has_stored_powerline_) {
                return;
            }

            PowerlineAdapter stored_pl_adapter = stored_powerline_adapter_.Load();

            PointCloudAdapter pc_adapter(
                rclcpp::Clock().now(),
                "world",
                stored_pl_adapter.GetPoints()
            );

            sensor_msgs::msg::PointCloud2 stored_pl_points_msg = pc_adapter.ToMsg();

            stored_powerline_points_pub_->publish(stored_pl_points_msg);

            plane_t proj_plane = stored_pl_adapter.projection_plane();
            vector_t pl_dir = proj_plane.normal;

            // Find yaw angle between pl_dir (xy) and 0,0:
            double yaw = atan2(pl_dir.y(), pl_dir.x());

            euler_angles_t euler_angles(0.0, 0.0, yaw);
            quaternion_t quaternion = eulToQuat(euler_angles);
            pose_t pl_pose;
            pl_pose.orientation = quaternion;
            pl_pose.position = proj_plane.p;

            geometry_msgs::msg::PoseStamped pl_pose_msg;
            pl_pose_msg.header.stamp = rclcpp::Clock().now();
            pl_pose_msg.header.frame_id = "world";
            pl_pose_msg.pose = poseMsgFromPose(pl_pose);

            stored_powerline_pose_pub_->publish(pl_pose_msg);

        }
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PowerlineOverviewProviderNode::on_deactivate(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::on_deactivate()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    pl_mapper_command_client_.reset();
    powerline_sub_.reset();
    vehicle_global_position_sub_.reset();
    update_powerline_overview_srv_.reset();
    get_powerline_overview_srv_.reset();
    stored_powerline_points_timer_->cancel();
    stored_powerline_points_timer_.reset();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PowerlineOverviewProviderNode::on_shutdown(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::on_shutdown()");

    auto ret = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

    if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
    {
        return ret;
    }

    // Thread shutting down rclcpp in 1 second:
    auto shutdown_thread = std::thread([this]() -> void
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    });

    shutdown_thread.detach();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn PowerlineOverviewProviderNode::on_error(
    const rclcpp_lifecycle::State & state
)
{
    RCLCPP_FATAL(get_logger(), "PowerlineOverviewProviderNode::on_error(): Lifecycle transition failed.");
    return rclcpp_lifecycle::LifecycleNode::on_error(state);
}

void PowerlineOverviewProviderNode::updatePowerlineOverviewCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::UpdatePowerlineOverview::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::UpdatePowerlineOverview::Response> response
)
{
    (void)request_header;
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback()");
    response->success = false;

    if (!pl_mapper_command_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Service not available");
        return;
    }

    auto plm_cmd_req = std::make_shared<iii_drone_interfaces::srv::PLMapperCommand::Request>();

    plm_cmd_req->pl_mapper_cmd.command = plm_cmd_req->pl_mapper_cmd.PL_MAPPER_CMD_START;
    plm_cmd_req->pl_mapper_cmd.reset = false;

    auto future = pl_mapper_command_client_->async_send_request(plm_cmd_req);

    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) 
    {
        RCLCPP_ERROR(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Service call failed");
        return;
    }

    rclcpp::Time start_time = rclcpp::Clock().now();

    int timeout_s = request->timeout_s;

    rclcpp::Rate rate(5);
    std::optional<PowerlineAdapter> previous_stable_candidate;
    std::size_t stable_sample_count = 0;
    const int64_t request_start_cached_stamp_ns =
        rclcpp::Time(latest_powerline_.Load().stamp).nanoseconds();
    int64_t latest_evaluated_stamp_ns = -1;
    std::string last_wait_reason = "no fresh powerline sample received";

    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Waiting for powerline data...");

    while((rclcpp::Clock().now() - start_time).seconds() < timeout_s && rclcpp::ok()) {

        iii_drone_interfaces::msg::Powerline latest_pl = latest_powerline_.Load();
        const int64_t latest_stamp_ns = rclcpp::Time(latest_pl.stamp).nanoseconds();

        if (latest_stamp_ns <= request_start_cached_stamp_ns) {
            RCLCPP_DEBUG(
                get_logger(),
                "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Waiting for fresh powerline sample (cached_stamp_ns=%ld, latest_stamp_ns=%ld)",
                request_start_cached_stamp_ns,
                latest_stamp_ns
            );
            last_wait_reason = "waiting for fresh powerline sample";
            rate.sleep();
            continue;
        }

        if (latest_stamp_ns == latest_evaluated_stamp_ns) {
            last_wait_reason = "waiting for a new powerline sample";
            rate.sleep();
            continue;
        }
        latest_evaluated_stamp_ns = latest_stamp_ns;

        if (latest_pl.lines.size() >= kMinimumPowerlineOverviewLines) {

            adapters::PowerlineAdapter powerline_adapter(latest_pl);

            if (!powerline_adapter.Transform("world", tf_buffer_)) {
                RCLCPP_WARN(
                    get_logger(),
                    "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Could not transform powerline to world, waiting for a transformable sample"
                );
                last_wait_reason = "latest powerline sample could not be transformed to world";
                rate.sleep();
                continue;
            }

            std::string geometry_reject_reason;
            if (!isOverviewGeometryPlausible(powerline_adapter, geometry_reject_reason)) {
                stable_sample_count = 0;
                previous_stable_candidate.reset();
                last_wait_reason = "overview geometry rejected: " + geometry_reject_reason;
                RCLCPP_DEBUG(
                    get_logger(),
                    "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Rejecting overview candidate: %s",
                    geometry_reject_reason.c_str()
                );
                rate.sleep();
                continue;
            }

            double max_line_delta = 0.0;
            if (!previous_stable_candidate.has_value()) {
                stable_sample_count = 1;
                previous_stable_candidate = powerline_adapter;
                RCLCPP_DEBUG(
                    get_logger(),
                    "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - First stable overview candidate (%zu/%zu)",
                    stable_sample_count,
                    kRequiredStablePowerlineOverviewSamples
                );
                last_wait_reason = "waiting for stable overview samples";
                rate.sleep();
                continue;
            }

            if (!isOverviewStableAgainstPrevious(previous_stable_candidate.value(), powerline_adapter, max_line_delta)) {
                stable_sample_count = 1;
                previous_stable_candidate = powerline_adapter;
                last_wait_reason = "overview candidate moved or changed ids";
                RCLCPP_DEBUG(
                    get_logger(),
                    "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Overview candidate moved or changed ids (max_delta=%.3f m), restarting stability window",
                    max_line_delta
                );
                rate.sleep();
                continue;
            }

            stable_sample_count++;
            previous_stable_candidate = powerline_adapter;

            RCLCPP_DEBUG(
                get_logger(),
                "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Stable overview candidate (%zu/%zu, max_delta=%.3f m)",
                stable_sample_count,
                kRequiredStablePowerlineOverviewSamples,
                max_line_delta
            );

            if (stable_sample_count < kRequiredStablePowerlineOverviewSamples) {
                last_wait_reason = "waiting for stable overview samples";
                rate.sleep();
                continue;
            }

            stored_powerline_ = powerline_adapter.ToMsg();
            if (!persistStoredPowerlineOverview(stored_powerline_.Load())) {
                RCLCPP_ERROR(
                    get_logger(),
                    "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Stable overview received, but GNSS persistence failed"
                );
                return;
            }
            stored_powerline_adapter_ = powerline_adapter;
            has_stored_powerline_ = true;

            response->success = true;

            RCLCPP_INFO(
                get_logger(),
                "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Stable powerline overview received with %zu visible line(s)",
                powerline_adapter.GetVisibleLineAdapters().size()
            );

            return;

        }

        RCLCPP_DEBUG(
            get_logger(),
            "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Not enough powerline registered (%zu/%zu), waiting for more...",
            latest_pl.lines.size(),
            kMinimumPowerlineOverviewLines
        );
        last_wait_reason = "not enough powerline lines registered";

        rate.sleep();

    }

    RCLCPP_WARN(
        get_logger(),
        "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Powerline overview not stored before timeout: %s",
        last_wait_reason.c_str()
    );

}

void PowerlineOverviewProviderNode::getPowerlineOverviewCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::GetPowerlineOverview::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::GetPowerlineOverview::Response> response
)
{
    (void)request_header;
    (void)request;
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::getPowerlineOverviewCallback()");

    const bool had_in_frame_overview = has_stored_powerline_;
    const bool had_gnss_on_disk = has_persisted_gnss_powerline_ || std::filesystem::exists(gnss_persistence_path_);
    bool loaded_from_gnss = false;

    if (!has_stored_powerline_) {
        loaded_from_gnss = loadPersistedPowerlineOverviewToMemory();
    }

    if (!has_stored_powerline_) {
        RCLCPP_WARN(get_logger(), "PowerlineOverviewProviderNode::getPowerlineOverviewCallback() - No stored powerline available");
        response->success = false;
        response->overview_in_frame = false;
        response->overview_gnss_only = had_gnss_on_disk;
        response->overview_source = had_gnss_on_disk ? "gnss_only_unavailable" : "none";
        return;
    }

    iii_drone_interfaces::msg::Powerline stored_pl = stored_powerline_.Load();

    response->stored_powerline = stored_pl;
    response->success = true;
    response->overview_in_frame = true;
    response->overview_gnss_only = false;
    response->overview_source = had_in_frame_overview
        ? "memory_world"
        : (loaded_from_gnss ? "loaded_gnss_to_world" : "memory_world");

    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::getPowerlineOverviewCallback() - Stored powerline sent");

}

bool PowerlineOverviewProviderNode::persistStoredPowerlineOverview(
    const iii_drone_interfaces::msg::Powerline & powerline
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

    if (!iii_drone::mission::overview_gnss::persistPowerlineOverview(
            powerline,
            reference.value(),
            gnss_persistence_path_,
            get_logger())) {
        return false;
    }

    has_persisted_gnss_powerline_ = true;
    RCLCPP_INFO(
        get_logger(),
        "PowerlineOverviewProviderNode: persisted GNSS powerline overview to %s",
        gnss_persistence_path_.string().c_str()
    );
    return true;
}

bool PowerlineOverviewProviderNode::loadPersistedPowerlineOverviewToMemory()
{
    if (!has_persisted_gnss_powerline_ && !std::filesystem::exists(gnss_persistence_path_)) {
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

    const auto loaded_powerline = iii_drone::mission::overview_gnss::loadPowerlineOverview(
        gnss_persistence_path_,
        reference.value(),
        get_logger()
    );
    if (!loaded_powerline.has_value()) {
        return false;
    }

    PowerlineAdapter adapter(loaded_powerline.value());
    std::string reject_reason;
    if (!isOverviewGeometryPlausible(adapter, reject_reason)) {
        RCLCPP_WARN(
            get_logger(),
            "PowerlineOverviewProviderNode: persisted GNSS powerline overview transformed to implausible world geometry: %s",
            reject_reason.c_str()
        );
        return false;
    }

    stored_powerline_ = loaded_powerline.value();
    stored_powerline_adapter_ = adapter;
    has_stored_powerline_ = true;
    has_persisted_gnss_powerline_ = true;
    RCLCPP_INFO(
        get_logger(),
        "PowerlineOverviewProviderNode: loaded GNSS powerline overview from %s into current world frame",
        gnss_persistence_path_.string().c_str()
    );
    return true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<PowerlineOverviewProviderNode>();

    executor.add_node(node->get_node_base_interface());

    try {
        executor.spin();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(node->get_logger(), "PowerlineOverviewProviderNode main loop failed: %s", e.what());
        node.reset();
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
