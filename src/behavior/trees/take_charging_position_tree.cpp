/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/trees/take_charging_position_tree.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::configuration;
using namespace iii_drone::types;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::px4;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TakeChargingPositionTree::TakeChargingPositionTree(
    Configurator::SharedPtr configurator,
    rclcpp::Node * node,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : configurator_(configurator),
    parameters_(configurator->GetParameterBundle("take_charging_position_tree")),
    node_(node),
    tf_buffer_(tf_buffer),
    maneuver_reference_client_(maneuver_reference_client) {

    RCLCPP_INFO(node_->get_logger(), "TakeChargingPositionTree::TakeChargingPositionTree(): Initializing.");

    initPX4();

}

void TakeChargingPositionTree::StartExecution() {

    registerNodes();

    tree_ = factory_.createTreeFromFile(parameters_->GetParameter("tree_xml_file").as_string());

    running_ = true;
    finished_ = false;
    success_ = false;

    execute_thread_ = std::thread(&TakeChargingPositionTree::execute, this);

}

void TakeChargingPositionTree::StopExecution() {

    running_ = false;

    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }

}

void TakeChargingPositionTree::initPX4() {

    take_charging_position_mode_ = std::make_unique<ManeuverMode>(
        *node_,
        "Take Charging Position",
        maneuver_reference_client_,
        parameters_->GetParameter("dt").as_double()
    );

    hover_on_cable_mode_ = std::make_unique<ManeuverMode>(
        *node_,
        "Hover on Cable",
        maneuver_reference_client_,
        parameters_->GetParameter("dt").as_double()
    );

    hover_mode_ = std::make_unique<ManeuverMode>(
        *node_,
        "Hover",
        maneuver_reference_client_,
        parameters_->GetParameter("dt").as_double()
    );

    mode_executor_ = std::make_unique<GenericModeExecutor>(
        *node_,
        *take_charging_position_mode_,
        "Take Charging Position Mode Executor",
        std::bind(&TakeChargingPositionTree::StartExecution, this),
        [this](GenericModeExecutor::DeactivateReason) -> void {
            RCLCPP_INFO(node_->get_logger(), "Take Charging Position Mode Executor deactivated.");
            StopExecution();
        }
    );

    if(!mode_executor_->doRegister()) {

        RCLCPP_FATAL(node_->get_logger(), "Failed to register mode executor.");

        throw std::runtime_error("Failed to register mode executor.");

    }

    if (!hover_on_cable_mode_->doRegister()) {

        RCLCPP_FATAL(node_->get_logger(), "Failed to register mode.");

        throw std::runtime_error("Failed to register mode.");

    }

    if (!hover_mode_->doRegister()) {

        RCLCPP_FATAL(node_->get_logger(), "Failed to register mode.");

        throw std::runtime_error("Failed to register mode.");

    }

    take_charging_position_mode_->RegisterAsOffboardMode();

    hover_on_cable_mode_->RegisterAsOffboardMode();

    hover_mode_->RegisterAsOffboardMode();

}

void TakeChargingPositionTree::execute() {

    unsigned int tick_period_ms = parameters_->GetParameter("tick_period_ms").as_int();

    std::chrono::milliseconds tick_period(tick_period_ms);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    status = tree_.tickOnce(); 
    
    while (running_ && status == BT::NodeStatus::RUNNING) {

        tree_.sleep(tick_period);

        status = tree_.tickOnce(); 

    }

    finished_ = true;

    success_ = status == BT::NodeStatus::SUCCESS && running_;

    running_ = false;

}

void TakeChargingPositionTree::registerNodes() {

    unsigned int server_timeout_ms = parameters_->GetParameter("server_timeout_ms").as_int();
    unsigned int wait_for_server_timeout_ms = parameters_->GetParameter("wait_for_server_timeout_ms").as_int();

    std::chrono::milliseconds server_timeout(server_timeout_ms);
    std::chrono::milliseconds wait_for_server_timeout(wait_for_server_timeout_ms);

    std::shared_ptr<rclcpp::Node> node = node_->shared_from_this();

    factory_ = BT::BehaviorTreeFactory();

    factory_.registerSimpleAction(
        "ScheduleTakeChargingPositionMode", 
        [this](const BT::TreeNode &) {

            mode_executor_->scheduleMode(
                take_charging_position_mode_->id(),
                [this](px4_ros2::Result) { }
            );

            while(!take_charging_position_mode_->isActive()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            return BT::NodeStatus::SUCCESS;

        }
    );

    factory_.registerSimpleAction(
        "ScheduleHoverOnCableMode", 
        [this](const BT::TreeNode &) {

            mode_executor_->scheduleMode(
                hover_on_cable_mode_->id(),
                [this](px4_ros2::Result) { }
            );

            while(!hover_on_cable_mode_->isActive()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            return BT::NodeStatus::SUCCESS;

        }
    );

    factory_.registerSimpleAction(
        "ScheduleHoverMode", 
        [this](const BT::TreeNode & tree_node) {

            mode_executor_->scheduleMode(
                hover_mode_->id(),
                [this](px4_ros2::Result) { }
            );

            while(!hover_mode_->isActive()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            return BT::NodeStatus::SUCCESS;

        }
    );

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/hover";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<HoverManeuverActionNode>(
            "Hover",
            params,
            maneuver_reference_client_
        );

    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/hover_on_cable";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<HoverOnCableManeuverActionNode>(
            "HoverOnCable",
            params,
            maneuver_reference_client_
        );

    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/hover_on_object";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<HoverByObjectManeuverActionNode>(
            "HoverByObject",
            params,
            maneuver_reference_client_
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/fly_to_object";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<FlyToObjectManeuverActionNode>(
            "FlyToObject",
            params,
            maneuver_reference_client_,
            configurator_->GetParameterBundle("fly_to_object_maneuver_action_node"),
            tf_buffer_
        );

    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/fly_to_position";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<FlyToPositionManeuverActionNode>(
            "FlyToPosition",
            params,
            maneuver_reference_client_
        );

    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/cable_landing";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<CableLandingManeuverActionNode>(
            "CableLanding",
            params,
            maneuver_reference_client_
        );

    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/cable_takeoff";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<CableTakeoffManeuverActionNode>(
            "CableTakeoff",
            params,
            maneuver_reference_client_,
            configurator_->GetParameterBundle("cable_takeoff_maneuver_action_node")
        );

    }

    {

        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/perception/pl_mapper/pl_mapper_command";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<PLMapperCommandActionNode>(
            "PLMapperCommand",
            params
        );

    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/perception/pl_mapper/powerline";

        factory_.registerNodeType<VerifyPowerlineDetectedConditionNode>(
            "VerifyPowerlineDetected",
            params
        );

    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/perception/pl_mapper/powerline";

        factory_.registerNodeType<SelectTargetLineConditionNode>(
            "SelectTargetLine",
            params,
            tf_buffer_,
            configurator_->GetParameterBundle("select_target_line_condition_node")
        );

    }

}