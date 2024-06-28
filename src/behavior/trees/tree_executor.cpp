/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/trees/tree_executor.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::configuration;
using namespace iii_drone::types;
using namespace iii_drone::control::maneuver;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

TreeExecutor::TreeExecutor(
    const std::string & tree_name,
    const std::string & tree_xml_file,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    Configurator<rclcpp::Node>::SharedPtr configurator,
    rclcpp::Node * node,
    BT::Blackboard::Ptr global_blackboard
) : tree_name_(tree_name),
    tree_xml_file_(tree_xml_file),
    maneuver_reference_client_(maneuver_reference_client),
    tf_buffer_(tf_buffer),
    configurator_(configurator),
    node_(node),
    global_blackboard_(global_blackboard)
{

    wordexp_t wordexp_result;
    wordexp(tree_xml_file_.c_str(), &wordexp_result, 0);
    tree_xml_file_ = wordexp_result.we_wordv[0];
    wordfree(&wordexp_result);

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::TreeExecutor(): Initializing %s.", tree_name.c_str());

}

TreeExecutor::~TreeExecutor() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::~TreeExecutor(): Deinitializing %s.", tree_name_.c_str());

    Deinitialize();

}

void TreeExecutor::FinalizeInitialization() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::FinalizeInitialization(): Finalizing initialization for %s.", tree_name_.c_str());

    registerNodes();

    local_blackboard_ = BT::Blackboard::create(global_blackboard_);

}

void TreeExecutor::Deinitialize() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::Deinitialize(): Deinitializing %s.", tree_name_.c_str());

    StopExecution();

    local_blackboard_.reset();

    unregisterNodes();

}

void TreeExecutor::StartExecution() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::FinalizeInitialization(): Creating tree for %s.", tree_name_.c_str());

    tree_ = factory_.createTreeFromFile(tree_xml_file_, local_blackboard_);

    running_ = true;
    finished_ = false;
    success_ = false;

    RCLCPP_INFO(node_->get_logger(), "TreeExecutor::StartExecution(): Starting execution of %s.", tree_name_.c_str());

    execute_thread_ = std::thread(&TreeExecutor::execute, this);

}

void TreeExecutor::StopExecution(bool wait) {

    running_ = false;

    if (wait && execute_thread_.joinable()) {
        execute_thread_.join();
    }

}

bool TreeExecutor::running() const {
    return running_;
}

bool TreeExecutor::finished() const {
    return finished_;
}

bool TreeExecutor::success() const {
    return success_;
}

void TreeExecutor::execute() {

    unsigned int tick_period_ms = configurator_->GetParameter("tick_period_ms").as_int();

    std::chrono::milliseconds tick_period(tick_period_ms);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    status = tree_.tickOnce(); 
    
    while (running_ && status == BT::NodeStatus::RUNNING) {

        tree_.sleep(tick_period);

        status = tree_.tickOnce(); 

    }

    tree_.haltTree();

    finished_ = true;

    success_ = status == BT::NodeStatus::SUCCESS && running_;

    running_ = false;

    tree_ = BT::Tree();

}

void TreeExecutor::registerNodes() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::registerNodes(): Registering nodes for %s.", tree_name_.c_str());

    unsigned int server_timeout_ms = configurator_->GetParameter("server_timeout_ms").as_int();
    unsigned int wait_for_server_timeout_ms = configurator_->GetParameter("wait_for_server_timeout_ms").as_int();

    std::chrono::milliseconds server_timeout(server_timeout_ms);
    std::chrono::milliseconds wait_for_server_timeout(wait_for_server_timeout_ms);

    std::shared_ptr<rclcpp::Node> node = node_->shared_from_this();

    factory_ = BT::BehaviorTreeFactory();

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
            maneuver_reference_client_,
            configurator_->GetParameterBundle("hover_on_cable_maneuver_action_node")
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/control/maneuver_controller/hover_by_object";
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
        params.default_port_value = "/payload/charger_gripper/gripper_command";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<GripperCommandActionNode>(
            "GripperCommand",
            params
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

    {
        factory_.registerNodeType<TargetProvider>(
            "TargetProvider",
            node_,
            tf_buffer_,
            configurator_->GetParameterBundle("target_provider")
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/fmu/out/vehicle_odometry";

        factory_.registerNodeType<StoreCurrentStateConditionNode>(
            "StoreCurrentState",
            params
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/mission/powerline_overview_provider/update_powerline_overview";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<UpdatePowerlineOverviewActionNode>(
            "UpdatePowerlineOverview",
            params
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/mission/powerline_overview_provider/get_powerline_overview";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<GetPowerlineOverviewActionNode>(
            "GetPowerlineOverview",
            params
        );
    }

    {
        factory_.registerNodeType<PowerlineWaypointProviderActionNode>(
            "PowerlineWaypointProvider",
            tf_buffer_,
            node_,
            configurator_->GetParameterBundle("powerline_waypoint_provider_action_node")
        );
    }

    {
        factory_.registerNodeType<BT::LoopNode<iii_drone::types::point_t>>(
            "LoopPoint"
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "powerline_waypoints";

        factory_.registerNodeType<PublishPowerlineWaypointsConditionNode>(
            "PublishPowerlineWaypoints",
            params
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/payload/charger_gripper/gripper_status";

        factory_.registerNodeType<VerifyGripperClosedConditionNode>(
            "VerifyGripperClosed",
            params
        );
    }


}

void TreeExecutor::unregisterNodes() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::unregisterNodes(): Unregistering nodes for %s.", tree_name_.c_str());

    factory_.clearRegisteredBehaviorTrees();
    
    factory_ = BT::BehaviorTreeFactory();

}