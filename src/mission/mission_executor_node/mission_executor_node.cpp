/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_executor_node/mission_executor_node.hpp>

using namespace iii_drone::configuration;
using namespace iii_drone::mission;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MissionExecutorNode::MissionExecutorNode(
    rclcpp::executors::MultiThreadedExecutor & executor_handle,
    std::string node_name,
    std::string node_namespace,
    const rclcpp::NodeOptions & options
) : rclcpp_lifecycle::LifecycleNode(
    node_name, 
    node_namespace, 
    options
),  executor_handle_(executor_handle) {
    auto set_logger_level = [this](int severity) {
        const rcutils_ret_t ret = rcutils_logging_set_logger_level(this->get_logger().get_name(), severity);
        if (ret != RCUTILS_RET_OK) {
            RCLCPP_WARN(this->get_logger(), "Failed to set logger level, rcutils_ret_t=%d", static_cast<int>(ret));
        }
    };

	std::string log_level = std::getenv("MISSION_EXECUTOR_LOG_LEVEL");

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

    write_behavior_tree_model_xml_service_ = create_service<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML>(
        "write_behavior_tree_model_xml",
        std::bind(&MissionExecutorNode::writeBehaviorTreeModelXmlService, this, std::placeholders::_1, std::placeholders::_2)
    );

    odometry_sub_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    get_reference_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::MissionExecutorNode()");

}

MissionExecutorNode::~MissionExecutorNode() {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::~MissionExecutorNode()");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_configure(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_configure()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_configure(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_configure(): Failed to configure parent class"
        );
        return ret;
    }

    // Configurator
    RCLCPP_DEBUG(
        get_logger(), 
        "MissionExecutorNode::on_configure(): Initializing configurator object"
    );

    configurator_ = std::make_shared<Configurator<rclcpp_lifecycle::LifecycleNode>>(
        this,
        "mission_executor"
    );

    // TF Buffer
    if (tf_buffer_ == nullptr) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // Mission Executor
    mission_executor_ = std::make_shared<MissionExecutor>(
        this, 
        tf_buffer_,
        configurator_->GetParameter("mission_specification_file").as_string(),
        odometry_sub_callback_group_,
        executor_handle_
    );

    mission_executor_->Configure(
        configurator_,
        get_reference_cb_group_
    );

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_configure(): Configured");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_cleanup(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_cleanup()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_cleanup(): Failed to cleanup parent class"
        );
        return ret;
    }

    cleanup();

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_cleanup(): Cleaned up");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_activate(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_activate()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_activate(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_activate(): Failed to activate parent class"
        );
        return ret;
    }

    // Mission Executor
    RCLCPP_INFO(
        get_logger(), 
        "MissionExecutorNode::on_activate(): Starting mission executor"
    );

    mission_executor_->Start(configurator_);

    RCLCPP_INFO(
        get_logger(), 
        "MissionExecutorNode::on_activate(): Activated"
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_deactivate(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_deactivate()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_deactivate(): Failed to deactivate parent class"
        );
        return ret;
    }

    // Mission Executor
    RCLCPP_DEBUG(
        get_logger(), 
        "MissionExecutorNode::on_deactivate(): Deactivating mission executor"
    );
    mission_executor_->Stop();

    RCLCPP_INFO(
        get_logger(), 
        "MissionExecutorNode::on_deactivate(): Deactivated"
    );

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_shutdown(
    const rclcpp_lifecycle::State & state
) {
    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_shutdown()");

    CallbackReturn ret = rclcpp_lifecycle::LifecycleNode::on_shutdown(state);

    if (ret != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(
            get_logger(), 
            "MissionExecutorNode::on_shutdown(): Failed to shutdown parent class"
        );
        return ret;
    }

    cleanup();

    // Create and start thread detached which sleeps for 1 second, then shuts down rclcpp
    std::thread shutdown_thread([this](){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    });
    shutdown_thread.detach();

    RCLCPP_INFO(get_logger(), "MissionExecutorNode::on_shutdown(): Shutdown completed");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn MissionExecutorNode::on_error(
    const rclcpp_lifecycle::State & state
) {
    (void)state;
    
    RCLCPP_FATAL(
        get_logger(), 
        "MissionExecutorNode::on_error(): An error occurred."
    );

    throw std::runtime_error("An error occurred.");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

}

void MissionExecutorNode::writeBehaviorTreeModelXmlService(
    const std::shared_ptr<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML::Request> request,
    std::shared_ptr<iii_drone_interfaces::srv::WriteBehaviorTreeModelXML::Response> response
) {
    (void)response;

    const BT::BehaviorTreeFactory & factory = mission_executor_->factory();

    std::string models_xml = BT::writeTreeNodesModelXML(factory);

    std::string destination = request->destination_file;

    std::ofstream file(destination);

    file << models_xml;

    file.close();

}

void MissionExecutorNode::cleanup() {

    RCLCPP_DEBUG(get_logger(), "MissionExecutorNode::cleanup()");

    // Mission Executor
    if (mission_executor_ != nullptr) {
        RCLCPP_INFO(get_logger(), "MissionExecutorNode::cleanup(): Cleaning up mission executor.");
        mission_executor_->Stop();
        mission_executor_->Cleanup();
        mission_executor_.reset();
        mission_executor_ = nullptr;
    }

    // Configurator
    if (configurator_ != nullptr) {
        RCLCPP_INFO(get_logger(), "MissionExecutorNode::cleanup(): Cleaning up configurator.");
        configurator_.reset();
        configurator_ = nullptr;
    }

    RCLCPP_DEBUG(get_logger(), "MissionExecutorNode::cleanup(): Cleaned up.");

}

int main(int argc, char **argv) {

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<MissionExecutorNode>(
        executor
    );

    executor.add_node(node->get_node_base_interface());

   try {
        
        executor.spin();
        node.reset();

    } catch(const std::exception& e) {
        
        node.reset();

    }
    
	if (rclcpp::ok()) {
		node.reset();
		rclcpp::shutdown();
	}

    return 0;

}
