/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/powerline_overview_provider_node/powerline_overview_provider_node.hpp>

using namespace iii_drone::mission::powerline_overview_provider_node;
using namespace iii_drone::adapters;
using namespace iii_drone::types;
using namespace iii_drone::math;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

PowerlineOverviewProviderNode::PowerlineOverviewProviderNode(
    std::string node_name,
    std::string node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(node_name, node_namespace, options)
{

	std::string log_level = std::getenv("POWERLINE_OVERVIEW_PROVIDER_LOG_LEVEL");

	if (log_level != "") {

		// Convert to upper case:
		std::transform(
			log_level.begin(), 
			log_level.end(), 
			log_level.begin(), 
			[](unsigned char c){ return std::toupper(c); }
		);

		if (log_level == "DEBUG") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
		} else if (log_level == "INFO") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);
		} else if (log_level == "WARN") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN);
		} else if (log_level == "ERROR") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_ERROR);
		} else if (log_level == "FATAL") {
			rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_FATAL);
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
            status_msg.data = has_stored_powerline_ ? "Powerline stored" : "No powerline stored";

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
        rmw_qos_profile_services_default,
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
    RCLCPP_FATAL(get_logger(), "PowerlineOverviewProviderNode::on_error()");

    throw std::runtime_error("PowerlineOverviewProviderNode::on_error()");
    
}

void PowerlineOverviewProviderNode::updatePowerlineOverviewCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::UpdatePowerlineOverview::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::UpdatePowerlineOverview::Response> response
)
{
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback()");

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

    rclcpp::Rate rate(1);

    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Waiting for powerline data...");

    while((rclcpp::Clock().now() - start_time).seconds() < timeout_s && rclcpp::ok()) {

        iii_drone_interfaces::msg::Powerline latest_pl = latest_powerline_.Load();

        if (latest_pl.lines.size() >= 4) {

            adapters::PowerlineAdapter powerline_adapter(latest_pl);

            powerline_adapter.Transform("world", tf_buffer_);
                
            stored_powerline_ = powerline_adapter.ToMsg();
            stored_powerline_adapter_ = powerline_adapter;
            has_stored_powerline_ = true;

            response->success = true;

            RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Powerline data received");

            return;

        }

        RCLCPP_DEBUG(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Not enough powerline registered, waiting for more...");

        rate.sleep();

    }

    response->success = false;

    RCLCPP_WARN(get_logger(), "PowerlineOverviewProviderNode::updatePowerlineOverviewCallback() - Powerline data not received");

}

void PowerlineOverviewProviderNode::getPowerlineOverviewCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::GetPowerlineOverview::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::GetPowerlineOverview::Response> response
)
{
    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::getPowerlineOverviewCallback()");

    if (!has_stored_powerline_) {
        RCLCPP_WARN(get_logger(), "PowerlineOverviewProviderNode::getPowerlineOverviewCallback() - No stored powerline available");
        response->success = false;
        return;
    }

    iii_drone_interfaces::msg::Powerline stored_pl = stored_powerline_.Load();

    response->stored_powerline = stored_pl;
    response->success = true;

    RCLCPP_INFO(get_logger(), "PowerlineOverviewProviderNode::getPowerlineOverviewCallback() - Stored powerline sent");

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
        node.reset();
    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}