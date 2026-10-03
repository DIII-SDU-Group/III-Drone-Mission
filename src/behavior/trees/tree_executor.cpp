/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/trees/tree_executor.hpp>
#include <iii_drone_mission/behavior/behavior_node_registry.hpp>
#include <iii_drone_core/diagnostics/hil_trace.hpp>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>

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
    BT::Blackboard::Ptr global_blackboard,
    std::shared_ptr<iii_drone::mission::RuntimeIntentBuffer> runtime_intent_buffer
) : tree_name_(tree_name),
    tree_xml_file_(tree_xml_file),
    maneuver_reference_client_(maneuver_reference_client),
    tf_buffer_(tf_buffer),
    configurator_(configurator),
    node_(node),
    global_blackboard_(global_blackboard),
    runtime_intent_buffer_(runtime_intent_buffer)
{

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

    StopExecution(true, "TREE_DEINITIALIZE");

    local_blackboard_.reset();

    unregisterNodes();

}

void TreeExecutor::StartExecution() {

    std::lock_guard<std::mutex> lock(execute_thread_mutex_);

    if (running_) {
        throw std::runtime_error(
            "TreeExecutor::StartExecution(): Tree " + tree_name_ + " is already running"
        );
    }

    if (execute_thread_.joinable()) {
        if (!finished_) {
            throw std::runtime_error(
                "TreeExecutor::StartExecution(): Previous execution of " + tree_name_ +
                " is still stopping"
            );
        }
        execute_thread_.join();
    }

    running_ = true;
    finished_ = false;
    success_ = false;
    stop_requested_ = false;
    if (maneuver_reference_client_) maneuver_reference_client_->ResetTerminalRetentionFailure();
    ++execution_generation_;

    RCLCPP_INFO(node_->get_logger(), "TreeExecutor::StartExecution(): Starting execution of %s.", tree_name_.c_str());

    execute_thread_ = std::thread(&TreeExecutor::execute, this);

    auto event = iii_drone::diagnostics::HilTrace::event("behavior_tree_worker_started");
    event.text("tree", tree_name_);
    event.commit();

}

void TreeExecutor::StopExecution(bool wait, const char * diagnostic_reason) {

    const auto stop_start = std::chrono::steady_clock::now();
    auto entry = iii_drone::diagnostics::HilTrace::event("tree_stop_entry");
    entry.text("tree", tree_name_);
    entry.boolean("wait", wait);
    entry.boolean("running", running_);
    entry.boolean("finished", finished_);
    entry.boolean("joinable", execute_thread_.joinable());
    entry.text("diagnostic_reason", diagnostic_reason == nullptr ? "UNSPECIFIED" : diagnostic_reason);
    entry.commit();

    std::lock_guard<std::mutex> lock(execute_thread_mutex_);

    stop_requested_ = true;
    running_ = false;

    if (wait && execute_thread_.joinable()) {
        auto before_join = iii_drone::diagnostics::HilTrace::event("tree_stop_before_join");
        before_join.text("tree", tree_name_);
        before_join.commit();
        const auto join_start = std::chrono::steady_clock::now();
        execute_thread_.join();
        const auto join_end = std::chrono::steady_clock::now();
        auto after_join = iii_drone::diagnostics::HilTrace::event("tree_stop_after_join");
        after_join.text("tree", tree_name_);
        after_join.number(
            "join_duration_ns",
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                join_end - join_start).count()));
        after_join.commit();
    }

    const auto stop_end = std::chrono::steady_clock::now();
    auto exit = iii_drone::diagnostics::HilTrace::event("tree_stop_exit");
    exit.text("tree", tree_name_);
    exit.boolean("wait", wait);
    exit.boolean("running", running_);
    exit.boolean("finished", finished_);
    exit.boolean("stop_requested", stop_requested_);
    exit.text("diagnostic_reason", diagnostic_reason == nullptr ? "UNSPECIFIED" : diagnostic_reason);
    exit.number(
        "duration_ns",
        static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
            stop_end - stop_start).count()));
    exit.commit();

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

const BT::BehaviorTreeFactory & TreeExecutor::factory() const {
    return factory_;
}

void TreeExecutor::execute() {

    auto worker_entry = iii_drone::diagnostics::HilTrace::event("behavior_tree_worker_entry");
    worker_entry.text("tree", tree_name_);
    worker_entry.commit();

    unsigned int tick_period_ms = configurator_->GetParameter("/behavior/tick_period_ms").as_int();

    std::chrono::milliseconds tick_period(tick_period_ms);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    bool execution_exception = false;
    bool halt_exception = false;
    const uint64_t tree_generation = execution_generation_;

    try {

        RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::execute(): Creating tree for %s.", tree_name_.c_str());

        tree_ = factory_.createTreeFromFile(tree_xml_file_, local_blackboard_);

        status_subscribers_.clear();
        if (tree_name_ == "reach_cable" && tree_.rootNode() != nullptr) {
            const BT::TreeNode * root_node = tree_.rootNode();
            BT::applyRecursiveVisitor(
                tree_.rootNode(),
                [this, tree_generation, root_node](BT::TreeNode * node) {
                    status_subscribers_.push_back(node->subscribeToStatusChange(
                        [this, tree_generation, root_node](
                            BT::TimePoint,
                            const BT::TreeNode & changed_node,
                            BT::NodeStatus previous_status,
                            BT::NodeStatus new_status
                        ) {
                            auto transition = iii_drone::diagnostics::HilTrace::event(
                                "behavior_tree_status_transition"
                            );
                            transition.text("tree", tree_name_);
                            transition.number("tree_generation", tree_generation);
                            transition.text("node", changed_node.name());
                            transition.text("full_path", changed_node.fullPath());
                            transition.text("registration", changed_node.registrationName());
                            transition.number("node_uid", changed_node.UID());
                            transition.number("node_type", static_cast<int>(changed_node.type()));
                            transition.text("previous_status", BT::toStr(previous_status, false));
                            transition.text("new_status", BT::toStr(new_status, false));
                            transition.boolean("root", &changed_node == root_node);
                            transition.commit();
                        }
                    ));
                }
            );
        }

        status = tree_.tickOnce(); 

        while (running_ && status == BT::NodeStatus::RUNNING) {

            tree_.sleep(tick_period);

            status = tree_.tickOnce(); 

        }

    } catch (const std::exception & e) {

        execution_exception = true;
        status = BT::NodeStatus::FAILURE;

        RCLCPP_ERROR(
            node_->get_logger(),
            "TreeExecutor::execute(): Tree %s threw during execution: %s",
            tree_name_.c_str(),
            e.what()
        );

    } catch (...) {

        execution_exception = true;
        status = BT::NodeStatus::FAILURE;

        RCLCPP_ERROR(
            node_->get_logger(),
            "TreeExecutor::execute(): Tree %s threw an unknown exception during execution.",
            tree_name_.c_str()
        );

    }

    // BT::Tree's destructor performs the complete halt traversal.  Do not
    // call haltTree() here as well: the old implementation halted every node
    // once explicitly and then a second time from the Tree destructor during
    // the assignment below.  That duplicate traversal can re-enter ROS/BT
    // cleanup paths and corrupt the process heap after repeated cycles.
    // Do not publish completion until the BT tree has been destroyed.  The
    // PX4 mode executor uses finished() as the handoff gate; publishing it
    // before tree_ teardown lets the successor mode construct a new tree
    // while this worker is still destroying ROS action nodes/subscriptions,
    // which can corrupt the process heap during repeated HIL cycles.
    const bool running_before_teardown = running_;
    const bool stop_requested = stop_requested_;
    const bool execution_success =
        !execution_exception && status == BT::NodeStatus::SUCCESS && running_before_teardown;

    std::string exit_reason = "OTHER";
    if (execution_exception) {
        exit_reason = "EXECUTION_EXCEPTION";
    } else if (status == BT::NodeStatus::SUCCESS && running_before_teardown) {
        exit_reason = "ROOT_SUCCESS";
    } else if (status == BT::NodeStatus::FAILURE) {
        exit_reason = "ROOT_FAILURE";
    } else if (!running_before_teardown || stop_requested) {
        exit_reason = "EXTERNAL_STOP";
    }

    auto teardown_begin = iii_drone::diagnostics::HilTrace::event("behavior_tree_teardown_begin");
    teardown_begin.text("tree", tree_name_);
    teardown_begin.number("tree_generation", tree_generation);
    teardown_begin.text("root_status_before_halt", BT::toStr(status, false));
    teardown_begin.boolean("running_before_teardown", running_before_teardown);
    teardown_begin.boolean("execution_exception", execution_exception);
    teardown_begin.boolean("stop_requested", stop_requested);
    teardown_begin.commit();
    try {
        tree_ = BT::Tree();
    } catch (...) {
        halt_exception = true;
        auto teardown_exception = iii_drone::diagnostics::HilTrace::event(
            "behavior_tree_teardown_exception"
        );
        teardown_exception.text("tree", tree_name_);
        teardown_exception.number("tree_generation", tree_generation);
        teardown_exception.commit();
        throw;
    }
    auto teardown_end = iii_drone::diagnostics::HilTrace::event("behavior_tree_teardown_end");
    teardown_end.text("tree", tree_name_);
    teardown_end.number("tree_generation", tree_generation);
    teardown_end.commit();
    status_subscribers_.clear();

    const bool terminal_retention_failed = maneuver_reference_client_ &&
        maneuver_reference_client_->TerminalRetentionFailed();
    if (terminal_retention_failed) exit_reason = "TERMINAL_RETENTION_FAILED";
    success_ = execution_success && !terminal_retention_failed;
    running_ = false;
    finished_ = true;

    auto worker_exit = iii_drone::diagnostics::HilTrace::event("behavior_tree_worker_exit");
    worker_exit.text("tree", tree_name_);
    worker_exit.number("tree_generation", tree_generation);
    worker_exit.boolean("success", success_);
    worker_exit.text("root_status", BT::toStr(status, false));
    worker_exit.boolean("running_before_teardown", running_before_teardown);
    worker_exit.boolean("stop_requested", stop_requested);
    worker_exit.boolean("execution_exception", execution_exception);
    worker_exit.boolean("halt_exception", halt_exception);
    worker_exit.text("exit_reason", halt_exception ? "HALT_EXCEPTION" : exit_reason);
    worker_exit.commit();

}

void TreeExecutor::registerNodes() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::registerNodes(): Registering nodes for %s.", tree_name_.c_str());

    unsigned int server_timeout_ms = configurator_->GetParameter("/behavior/server_timeout_ms").as_int();
    unsigned int wait_for_server_timeout_ms = configurator_->GetParameter("/behavior/wait_for_server_timeout_ms").as_int();

    std::chrono::milliseconds server_timeout(server_timeout_ms);
    std::chrono::milliseconds wait_for_server_timeout(wait_for_server_timeout_ms);

    std::shared_ptr<rclcpp::Node> node = node_->shared_from_this();

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
            configurator_->GetConfiguration("hover_on_cable_maneuver_action_node")
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
        params.default_port_value = "/control/maneuver_controller/follow_waypoint_path";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;
        factory_.registerNodeType<FollowWaypointPathManeuverActionNode>(
            "FollowWaypointPath",
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
            configurator_->GetConfiguration("cable_takeoff_maneuver_action_node")
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
            params,
            tf_buffer_
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
            configurator_->GetConfiguration("select_target_line_condition_node")
        );
    }

    {
        factory_.registerNodeType<TargetProvider>(
            "TargetProvider",
            node_,
            tf_buffer_,
            configurator_->GetConfiguration("target_provider")
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
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/mission/pylon_overview_provider/get_pylon_overview";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<GetPylonOverviewActionNode>(
            "GetPylonOverview",
            params
        );
    }

    {
        factory_.registerNodeType<PowerlineWaypointProviderActionNode>(
            "PowerlineWaypointProvider",
            tf_buffer_,
            node_,
            configurator_->GetConfiguration("powerline_waypoint_provider_action_node")
        );
    }

    {
        factory_.registerNodeType<PhaseWaypointProviderActionNode>(
            "PhaseWaypointProvider",
            node_,
            configurator_->GetConfiguration("phase_waypoint_provider_action_node")
        );
    }

    {
        factory_.registerNodeType<BT::LoopNode<iii_drone::types::point_t>>(
            "LoopPoint"
        );
    }

    {
        factory_.registerNodeType<SplitPointQueueActionNode>(
            "SplitPointQueue"
        );
    }

    RegisterWaypointQueueNodes(factory_);

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

    {
        factory_.registerNodeType<ShouldRechargeBatteryLowConditionNode>(
            "ShouldRechargeBatteryLow",
            node,
            configurator_->GetConfiguration("battery_recharge_condition_node")
        );
    }

    {
        factory_.registerNodeType<WaitForPX4AirborneActionNode>(
            "WaitForPX4Airborne",
            node
        );
    }

    {
        factory_.registerNodeType<CableChargingMonitorActionNode>(
            "CableChargingMonitor",
            node,
            configurator_->GetConfiguration("cable_charging_monitor_action_node"),
            global_blackboard_
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/fmu/out/vehicle_status_v1";

        factory_.registerNodeType<VerifyDisarmedConditionNode>(
            "VerifyDisarmed",
            params
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/perception/pl_mapper/powerline";

        factory_.registerNodeType<GetGripperAlignmentYawConditionNode>(
            "GetGripperAlignmentYaw",
            params,
            tf_buffer_
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/mission/mode_executor/action";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<ModeExecutorActionNode>(
            "ModeExecutorAction",
            params
        );
    }

    {
        factory_.registerNodeType<LogMessageActionNode>(
            "LogMessage",
            node
        );
    }

    {
        factory_.registerNodeType<ApplyPendingIntentUpdatesActionNode>(
            "ApplyPendingIntentUpdates",
            runtime_intent_buffer_,
            global_blackboard_,
            node
        );
    }

    {
        factory_.registerNodeType<SetBlackboardBoolActionNode>(
            "SetBlackboardBool",
            global_blackboard_
        );
    }

    {
        factory_.registerNodeType<BlackboardBoolConditionNode>(
            "BlackboardBool",
            global_blackboard_
        );
    }

    {
        factory_.registerNodeType<SetBlackboardStringActionNode>(
            "SetBlackboardString",
            global_blackboard_
        );
    }

    {
        factory_.registerNodeType<BlackboardStringEqualsConditionNode>(
            "BlackboardStringEquals",
            global_blackboard_
        );
    }

    {
        factory_.registerNodeType<InitializeInspectionWaypointsActionNode>(
            "InitializeInspectionWaypoints",
            global_blackboard_
        );
    }

    {
        factory_.registerNodeType<GetCurrentInspectionWaypointActionNode>(
            "GetCurrentInspectionWaypoint",
            global_blackboard_
        );
    }

    {
        factory_.registerNodeType<AdvanceInspectionWaypointActionNode>(
            "AdvanceInspectionWaypoint",
            global_blackboard_
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/mission/rosbag_recorder/start_recording";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<StartRosbagRecordingActionNode>(
            "StartRosbagRecording",
            params
        );
    }

    {
        BT::RosNodeParams params;

        params.nh = node;
        params.default_port_value = "/mission/rosbag_recorder/stop_recording";
        params.server_timeout = server_timeout;
        params.wait_for_server_timeout = wait_for_server_timeout;

        factory_.registerNodeType<StopRosbagRecordingActionNode>(
            "StopRosbagRecording",
            params
        );
    }

    {
        factory_.registerNodeType<RosbagRecordingScopeDecorator>(
            "RosbagRecordingScope",
            node,
            "/mission/rosbag_recorder/start_recording",
            "/mission/rosbag_recorder/stop_recording",
            server_timeout,
            wait_for_server_timeout
        );
    }

    {
        factory_.registerNodeType<StringEqualsConditionNode>(
            "StringEquals"
        );
    }

    {
        factory_.registerNodeType<RetryUntilSuccessfulOnAbortedDecorator>(
            "RetryUntilSuccessfulOnAborted"
        );
    }

    ValidateRuntimeBehaviorFactory(factory_);

}

void TreeExecutor::unregisterNodes() {

    RCLCPP_DEBUG(node_->get_logger(), "TreeExecutor::unregisterNodes(): Unregistering nodes for %s.", tree_name_.c_str());

    factory_.clearRegisteredBehaviorTrees();

}
