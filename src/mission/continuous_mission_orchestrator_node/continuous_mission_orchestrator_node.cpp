/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "iii_drone_core/mission/continuous_mission_orchestrator_node/continuous_mission_orchestrator_node.hpp"

using namespace iii_drone::mission::continuous_mission_orchestrator_node;
using namespace iii_drone::math;
using namespace iii_drone::types;
using namespace std::placeholders;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

ContinuousMissionOrchestrator::ContinuousMissionOrchestrator(const std::string & node_name, 
                const std::string & node_namespace, const rclcpp::NodeOptions & options) :
        Node(node_name, node_namespace, options) {

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator()");

    // Parameters:
    this->declare_parameter<float>("under_cable_target_distance", 1.5);
    this->declare_parameter<bool>("use_charger_gripper_info", true);
    this->declare_parameter<int>("cable_landing_max_retries", 3);
    this->declare_parameter<float>("battery_voltage_low_threshold", 22.2);
    this->declare_parameter<float>("battery_voltage_high_threshold", 25.);
    this->declare_parameter<float>("charging_power_low_threshold", 0.1);
    this->declare_parameter<int>("action_timeout_ms", 1000);
    this->declare_parameter<int>("charging_minimum_duration_s", 10);
    this->declare_parameter<int>("charging_maximum_duration_s", -1);

    // Action clients:
    this->fly_under_cable_client_ = rclcpp_action::create_client<FlyUnderCable>(
       this, "/control/flight_controller/fly_under_cable"
    );
    this->cable_landing_client_ = rclcpp_action::create_client<CableLanding>(
       this, "/control/flight_controller/cable_landing"
    );
    this->disarm_on_cable_client_ = rclcpp_action::create_client<DisarmOnCable>(
       this, "/control/flight_controller/disarm_on_cable"
    );
    this->arm_on_cable_client_ = rclcpp_action::create_client<ArmOnCable>(
       this, "/control/flight_controller/arm_on_cable"
    );
    this->cable_takeoff_client_ = rclcpp_action::create_client<CableTakeoff>(
       this, "/control/flight_controller/cable_takeoff"
    );

    // Gripper command client:
    this->gripper_command_client_ = this->create_client<iii_drone_interfaces::srv::GripperCommand>(
        "/payload/charger_gripper/gripper_command"
    );

    // Target cable service:
    set_target_cable_id_srv_ = this->create_service<iii_drone_interfaces::srv::SetTargetCableId>(
        "set_target_cable_id",
        std::bind(&ContinuousMissionOrchestrator::setTargetCableIdSrvCallback, this, _1, _2, _3)
    );

    // Charger/gripper status:
    battery_voltage_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/payload/charger_gripper/battery_voltage", 10,
        std::bind(&ContinuousMissionOrchestrator::batteryVoltageCallback, this, std::placeholders::_1));
    charging_power_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/payload/charger_gripper/charging_power", 10,
        std::bind(&ContinuousMissionOrchestrator::chargingPowerCallback, this, std::placeholders::_1));
    charger_operating_mode_sub_ = this->create_subscription<iii_drone_interfaces::msg::ChargerOperatingMode>(
        "/payload/charger_gripper/charger_operating_mode", 10,
        std::bind(&ContinuousMissionOrchestrator::chargerOperatingModeCallback, this, std::placeholders::_1));
    charger_status_sub_ = this->create_subscription<iii_drone_interfaces::msg::ChargerStatus>(
        "/payload/charger_gripper/charger_status", 10,
        std::bind(&ContinuousMissionOrchestrator::chargerStatusCallback, this, std::placeholders::_1));
    gripper_status_sub_ = this->create_subscription<iii_drone_interfaces::msg::GripperStatus>(
        "/payload/charger_gripper/gripper_status", 10,
        std::bind(&ContinuousMissionOrchestrator::gripperStatusCallback, this, std::placeholders::_1));

    // Charging:
    initiate_charging_srv_server_ = this->create_service<iii_drone_interfaces::srv::InitiateCharging>(
        "initiate_charging",
        std::bind(&ContinuousMissionOrchestrator::initiateChargingSrvCallback, this, _1, _2, _3)
    );
    interrupt_charging_srv_server_ = this->create_service<iii_drone_interfaces::srv::InterruptCharging>(
        "interrupt_charging",
        std::bind(&ContinuousMissionOrchestrator::interruptChargingSrvCallback, this, _1, _2, _3)
    );

    // Control state:
    control_state_sub_ = this->create_subscription<iii_drone_interfaces::msg::ControlState>(
        "/control/flight_controller/control_state", 10,
        std::bind(&ContinuousMissionOrchestrator::controlStateCallback, this, std::placeholders::_1));
    

	// tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // State machine:
    state_machine_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ContinuousMissionOrchestrator::stateMachineCallback, this)
    );
    state_pub_ = this->create_publisher<std_msgs::msg::String>("state", 10);

	// Publishing:
	publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ContinuousMissionOrchestrator::publishCallback, this)
    );

}

// FlyUnderCable action:

void ContinuousMissionOrchestrator::flyUnderCableGoalResponseCallback(const GoalHandleFlyUnderCable::SharedPtr & goal_handle) {
    
    bool goal_accepted = false;

    if (!goal_handle) {

        RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::flyUnderCableGoalResponseCallback(): FlyUnderCable goal was rejected by server");

        goal_accepted = false;

    } else {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::flyUnderCableGoalResponseCallback(): Goal accepted by server, waiting for result");

        goal_accepted = true;
    }

    std::lock_guard<std::mutex> lock(fly_under_cable_mutex_);
    fly_under_cable_response_received_ = true;
    fly_under_cable_goal_accepted_ = goal_accepted;
    fly_under_cable_active_ = goal_accepted;

}

void ContinuousMissionOrchestrator::flyUnderCableFeedbackCallback(
    GoalHandleFlyUnderCable::SharedPtr,
    const std::shared_ptr<const FlyUnderCable::Feedback> feedback
) { }

void ContinuousMissionOrchestrator::flyUnderCableResultCallback(const GoalHandleFlyUnderCable::WrappedResult & result) {

    bool fly_under_cable_active = false;
    bool fly_under_cable_success = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::flyUnderCableResultCallback(): FlyUnderCable succeeded");
            fly_under_cable_active = false;
            fly_under_cable_success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::flyUnderCableResultCallback(): FlyUnderCable was aborted");
            fly_under_cable_active = false;
            fly_under_cable_success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::flyUnderCableResultCallback(): FlyUnderCable was canceled");
            fly_under_cable_active = false;
            fly_under_cable_success = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::flyUnderCableResultCallback(): Unknown result code for FlyUnderCable");
            fly_under_cable_active = false;
            fly_under_cable_success = false;
            break;
    }

    std::lock_guard<std::mutex> lock(fly_under_cable_mutex_);

    fly_under_cable_active_ = fly_under_cable_active;
    fly_under_cable_success_ = fly_under_cable_success;

}

void ContinuousMissionOrchestrator::startFlyUnderCable() { 

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::startFlyUnderCable(): Starting FlyUnderCable");

    {
        std::lock_guard<std::mutex> lock(fly_under_cable_mutex_);

        fly_under_cable_response_received_ = false;
        fly_under_cable_active_ = false;
        fly_under_cable_success_ = false;
        fly_under_cable_goal_accepted_ = false;
    }

    auto goal_msg = FlyUnderCable::Goal();
    goal_msg.target_cable_id = getTargetCableId();
    float target_cable_distance;
    this->get_parameter("under_cable_target_distance", target_cable_distance);
    goal_msg.target_cable_distance = target_cable_distance;

    auto goal_options = rclcpp_action::Client<FlyUnderCable>::SendGoalOptions();

    goal_options.goal_response_callback = std::bind(
        &ContinuousMissionOrchestrator::flyUnderCableGoalResponseCallback, this, std::placeholders::_1
    );
    goal_options.feedback_callback = std::bind(
        &ContinuousMissionOrchestrator::flyUnderCableFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2
    );
    goal_options.result_callback = std::bind(
        &ContinuousMissionOrchestrator::flyUnderCableResultCallback, this, std::placeholders::_1
    );

    auto goal_handle_future = fly_under_cable_client_->async_send_goal(goal_msg, goal_options);

}

bool ContinuousMissionOrchestrator::flyUnderCableTerminated() {

    std::lock_guard<std::mutex> lock(fly_under_cable_mutex_);

    return (!fly_under_cable_active_) && fly_under_cable_response_received_;

}

bool ContinuousMissionOrchestrator::flyUnderCableSucceeded() {

    std::lock_guard<std::mutex> lock(fly_under_cable_mutex_);

    return fly_under_cable_goal_accepted_ && fly_under_cable_success_;

}

void ContinuousMissionOrchestrator::cancelFlyUnderCable() {

    RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::cancelFlyUnderCable(): Canceling FlyUnderCable");

    fly_under_cable_client_->async_cancel_all_goals();

}

// CableLanding action:

void ContinuousMissionOrchestrator::cableLandingGoalResponseCallback(const GoalHandleCableLanding::SharedPtr & goal_handle) {
    
    bool goal_accepted = false;

    if (!goal_handle) {

        RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::cableLandingGoalResponseCallback(): CableLanding goal was rejected by server");

        goal_accepted = false;

    } else {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::cableLandingGoalResponseCallback(): Goal accepted by server, waiting for result");

        goal_accepted = true;
    }

    std::lock_guard<std::mutex> lock(cable_landing_mutex_);
    cable_landing_response_received_ = true;
    cable_landing_goal_accepted_ = goal_accepted;
    cable_landing_active_ = goal_accepted;

}

void ContinuousMissionOrchestrator::cableLandingFeedbackCallback(
    GoalHandleCableLanding::SharedPtr,
    const std::shared_ptr<const CableLanding::Feedback> feedback
) { }

void ContinuousMissionOrchestrator::cableLandingResultCallback(const GoalHandleCableLanding::WrappedResult & result) {

    bool cable_landing_active = false;
    bool cable_landing_success = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::cableLandingResultCallback(): CableLanding succeeded");
            cable_landing_active = false;
            cable_landing_success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::cableLandingResultCallback(): CableLanding was aborted");
            cable_landing_active = false;
            cable_landing_success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::cableLandingResultCallback(): CableLanding was canceled");
            cable_landing_active = false;
            cable_landing_success = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::cableLandingResultCallback(): Unknown result code for CableLanding");
            cable_landing_active = false;
            cable_landing_success = false;
            break;
    }

    std::lock_guard<std::mutex> lock(cable_landing_mutex_);

    cable_landing_active_ = cable_landing_active;
    cable_landing_success_ = cable_landing_success;

}

void ContinuousMissionOrchestrator::startCableLanding() { 

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::startCableLanding(): Starting CableLanding");

    {
        std::lock_guard<std::mutex> lock(cable_landing_mutex_);

        cable_landing_response_received_ = false;
        cable_landing_active_ = false;
        cable_landing_success_ = false;
        cable_landing_goal_accepted_ = false;
    }

    auto goal_msg = CableLanding::Goal();
    goal_msg.target_cable_id = getTargetCableId();

    auto goal_options = rclcpp_action::Client<CableLanding>::SendGoalOptions();

    goal_options.goal_response_callback = std::bind(
        &ContinuousMissionOrchestrator::cableLandingGoalResponseCallback, this, std::placeholders::_1
    );
    goal_options.feedback_callback = std::bind(
        &ContinuousMissionOrchestrator::cableLandingFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2
    );
    goal_options.result_callback = std::bind(
        &ContinuousMissionOrchestrator::cableLandingResultCallback, this, std::placeholders::_1
    );

    auto goal_handle_future = cable_landing_client_->async_send_goal(goal_msg, goal_options);

}

bool ContinuousMissionOrchestrator::cableLandingTerminated() {

    std::lock_guard<std::mutex> lock(cable_landing_mutex_);

    return (!cable_landing_active_) && cable_landing_response_received_;

}

bool ContinuousMissionOrchestrator::cableLandingSucceeded() {

    std::lock_guard<std::mutex> lock(cable_landing_mutex_);

    return cable_landing_goal_accepted_ && cable_landing_success_;

}

void ContinuousMissionOrchestrator::cancelCableLanding() {

    RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::cancelCableLanding(): Canceling CableLanding");

    cable_landing_client_->async_cancel_all_goals();

}

// DisarmOnCable action:

void ContinuousMissionOrchestrator::disarmOnCableGoalResponseCallback(const GoalHandleDisarmOnCable::SharedPtr & goal_handle) {
    
    bool goal_accepted = false;

    if (!goal_handle) {

        RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::disarmOnCableGoalResponseCallback(): DisarmOnCable goal was rejected by server");

        goal_accepted = false;

    } else {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::disarmOnCableGoalResponseCallback(): Goal accepted by server, waiting for result");

        goal_accepted = true;
    }

    std::lock_guard<std::mutex> lock(disarm_on_cable_mutex_);
    disarm_on_cable_response_received_ = true;
    disarm_on_cable_goal_accepted_ = goal_accepted;
    disarm_on_cable_active_ = goal_accepted;

}

void ContinuousMissionOrchestrator::disarmOnCableFeedbackCallback(
    GoalHandleDisarmOnCable::SharedPtr,
    const std::shared_ptr<const DisarmOnCable::Feedback> feedback
) { }

void ContinuousMissionOrchestrator::disarmOnCableResultCallback(const GoalHandleDisarmOnCable::WrappedResult & result) {

    bool disarm_on_cable_active = false;
    bool disarm_on_cable_success = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::disarmOnCableResultCallback(): DisarmOnCable succeeded");
            disarm_on_cable_active = false;
            disarm_on_cable_success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::disarmOnCableResultCallback(): DisarmOnCable was aborted");
            disarm_on_cable_active = false;
            disarm_on_cable_success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::disarmOnCableResultCallback(): DisarmOnCable was canceled");
            disarm_on_cable_active = false;
            disarm_on_cable_success = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::disarmOnCableResultCallback(): Unknown result code for DisarmOnCable");
            disarm_on_cable_active = false;
            disarm_on_cable_success = false;
            break;
    }

    std::lock_guard<std::mutex> lock(disarm_on_cable_mutex_);

    disarm_on_cable_active_ = disarm_on_cable_active;
    disarm_on_cable_success_ = disarm_on_cable_success;

}

void ContinuousMissionOrchestrator::startDisarmOnCable() { 

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::startDisarmOnCable(): Starting DisarmOnCable");

    {
        std::lock_guard<std::mutex> lock(disarm_on_cable_mutex_);

        disarm_on_cable_response_received_ = false;
        disarm_on_cable_active_ = false;
        disarm_on_cable_success_ = false;
        disarm_on_cable_goal_accepted_ = false;
    }

    auto goal_msg = DisarmOnCable::Goal();

    auto goal_options = rclcpp_action::Client<DisarmOnCable>::SendGoalOptions();

    goal_options.goal_response_callback = std::bind(
        &ContinuousMissionOrchestrator::disarmOnCableGoalResponseCallback, this, std::placeholders::_1
    );
    goal_options.feedback_callback = std::bind(
        &ContinuousMissionOrchestrator::disarmOnCableFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2
    );
    goal_options.result_callback = std::bind(
        &ContinuousMissionOrchestrator::disarmOnCableResultCallback, this, std::placeholders::_1
    );

    auto goal_handle_future = disarm_on_cable_client_->async_send_goal(goal_msg, goal_options);

}

bool ContinuousMissionOrchestrator::disarmOnCableTerminated() {

    std::lock_guard<std::mutex> lock(disarm_on_cable_mutex_);

    return (!disarm_on_cable_active_) && disarm_on_cable_response_received_;

}

bool ContinuousMissionOrchestrator::disarmOnCableSucceeded() {

    std::lock_guard<std::mutex> lock(disarm_on_cable_mutex_);

    return disarm_on_cable_goal_accepted_ && disarm_on_cable_success_;

}

void ContinuousMissionOrchestrator::cancelDisarmOnCable() {

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::cancelDisarmOnCable(): Canceling DisarmOnCable");

    disarm_on_cable_client_->async_cancel_all_goals();

}

// ArmOnCable action:

void ContinuousMissionOrchestrator::armOnCableGoalResponseCallback(const GoalHandleArmOnCable::SharedPtr & goal_handle) {
    
    bool goal_accepted = false;

    if (!goal_handle) {

        RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::armOnCableGoalResponseCallback(): Goal was rejected by server");

        goal_accepted = false;

    } else {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::armOnCableGoalResponseCallback(): Goal accepted by server, waiting for result");

        goal_accepted = true;
    }

    std::lock_guard<std::mutex> lock(arm_on_cable_mutex_);
    arm_on_cable_response_received_ = true;
    arm_on_cable_goal_accepted_ = goal_accepted;
    arm_on_cable_active_ = goal_accepted;

}

void ContinuousMissionOrchestrator::armOnCableFeedbackCallback(
    GoalHandleArmOnCable::SharedPtr,
    const std::shared_ptr<const ArmOnCable::Feedback> feedback
) { }

void ContinuousMissionOrchestrator::armOnCableResultCallback(const GoalHandleArmOnCable::WrappedResult & result) {

    bool arm_on_cable_active = false;
    bool arm_on_cable_success = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::armOnCableResultCallback(): ArmOnCable succeeded");
            arm_on_cable_active = false;
            arm_on_cable_success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::armOnCableResultCallback(): ArmOnCable was aborted");
            arm_on_cable_active = false;
            arm_on_cable_success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::armOnCableResultCallback(): ArmOnCable was canceled");
            arm_on_cable_active = false;
            arm_on_cable_success = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::armOnCableResultCallback(): Unknown result code for ArmOnCable");
            arm_on_cable_active = false;
            arm_on_cable_success = false;
            break;
    }

    std::lock_guard<std::mutex> lock(arm_on_cable_mutex_);

    arm_on_cable_active_ = arm_on_cable_active;
    arm_on_cable_success_ = arm_on_cable_success;

}

void ContinuousMissionOrchestrator::startArmOnCable() { 

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::startArmOnCable(): Starting ArmOnCable");

    {
        std::lock_guard<std::mutex> lock(arm_on_cable_mutex_);

        arm_on_cable_response_received_ = false;
        arm_on_cable_active_ = false;
        arm_on_cable_success_ = false;
        arm_on_cable_goal_accepted_ = false;
    }

    auto goal_msg = ArmOnCable::Goal();

    auto goal_options = rclcpp_action::Client<ArmOnCable>::SendGoalOptions();

    goal_options.goal_response_callback = std::bind(
        &ContinuousMissionOrchestrator::armOnCableGoalResponseCallback, this, std::placeholders::_1
    );
    goal_options.feedback_callback = std::bind(
        &ContinuousMissionOrchestrator::armOnCableFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2
    );
    goal_options.result_callback = std::bind(
        &ContinuousMissionOrchestrator::armOnCableResultCallback, this, std::placeholders::_1
    );

    auto goal_handle_future = arm_on_cable_client_->async_send_goal(goal_msg, goal_options);

}

bool ContinuousMissionOrchestrator::armOnCableTerminated() {

    std::lock_guard<std::mutex> lock(arm_on_cable_mutex_);

    return (!arm_on_cable_active_) && arm_on_cable_response_received_;

}

bool ContinuousMissionOrchestrator::armOnCableSucceeded() {

    std::lock_guard<std::mutex> lock(arm_on_cable_mutex_);

    return arm_on_cable_goal_accepted_ && arm_on_cable_success_;

}

void ContinuousMissionOrchestrator::cancelArmOnCable() {

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::cancelArmOnCable(): Canceling ArmOnCable");

    arm_on_cable_client_->async_cancel_all_goals();

}

// CableTakeoff action:

void ContinuousMissionOrchestrator::cableTakeoffGoalResponseCallback(const GoalHandleCableTakeoff::SharedPtr & goal_handle) {
    
    bool goal_accepted = false;

    if (!goal_handle) {

        RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::cableTakeoffGoalResponseCallback(): Goal was rejected by server");

        goal_accepted = false;

    } else {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::cableTakeoffGoalResponseCallback(): Goal was accepted by server");

        goal_accepted = true;
    }

    std::lock_guard<std::mutex> lock(cable_takeoff_mutex_);
    cable_takeoff_response_received_ = true;
    cable_takeoff_goal_accepted_ = goal_accepted;
    cable_takeoff_active_ = goal_accepted;

}

void ContinuousMissionOrchestrator::cableTakeoffFeedbackCallback(
    GoalHandleCableTakeoff::SharedPtr,
    const std::shared_ptr<const CableTakeoff::Feedback> feedback
) { }

void ContinuousMissionOrchestrator::cableTakeoffResultCallback(const GoalHandleCableTakeoff::WrappedResult & result) {

    bool cable_takeoff_active = false;
    bool cable_takeoff_success = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::cableTakeoffResultCallback(): CableTakeoff succeeded");  
            cable_takeoff_active = false;
            cable_takeoff_success = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::cableTakeoffResultCallback(): CableTakeoff was aborted");
            cable_takeoff_active = false;
            cable_takeoff_success = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::cableTakeoffResultCallback(): CableTakeoff was canceled");
            cable_takeoff_active = false;
            cable_takeoff_success = false;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::cableTakeoffResultCallback(): Unknown result code for CableTakeoff");
            cable_takeoff_active = false;
            cable_takeoff_success = false;
            break;
    }

    std::lock_guard<std::mutex> lock(cable_takeoff_mutex_);

    cable_takeoff_active_ = cable_takeoff_active;
    cable_takeoff_success_ = cable_takeoff_success;

}

void ContinuousMissionOrchestrator::startCableTakeoff() { 

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::startCableTakeoff(): Starting CableTakeoff");

    {
        std::lock_guard<std::mutex> lock(cable_takeoff_mutex_);

        cable_takeoff_response_received_ = false;
        cable_takeoff_active_ = false;
        cable_takeoff_success_ = false;
        cable_takeoff_goal_accepted_ = false;
    }

    auto goal_msg = CableTakeoff::Goal();
    float target_cable_distance;
    this->get_parameter("under_cable_target_distance", target_cable_distance);
    goal_msg.target_cable_distance = target_cable_distance;

    auto goal_options = rclcpp_action::Client<CableTakeoff>::SendGoalOptions();

    goal_options.goal_response_callback = std::bind(
        &ContinuousMissionOrchestrator::cableTakeoffGoalResponseCallback, this, std::placeholders::_1
    );
    goal_options.feedback_callback = std::bind(
        &ContinuousMissionOrchestrator::cableTakeoffFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2
    );
    goal_options.result_callback = std::bind(
        &ContinuousMissionOrchestrator::cableTakeoffResultCallback, this, std::placeholders::_1
    );

    auto goal_handle_future = cable_takeoff_client_->async_send_goal(goal_msg, goal_options);

}

bool ContinuousMissionOrchestrator::cableTakeoffTerminated() {

    std::lock_guard<std::mutex> lock(cable_takeoff_mutex_);

    return (!cable_takeoff_active_) && cable_takeoff_response_received_;

}

bool ContinuousMissionOrchestrator::cableTakeoffSucceeded() {

    std::lock_guard<std::mutex> lock(cable_takeoff_mutex_);

    return cable_takeoff_goal_accepted_ && cable_takeoff_success_;

}

void ContinuousMissionOrchestrator::cancelCableTakeoff() {

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::cancelCableTakeoff(): Canceling CableTakeoff");

    cable_takeoff_client_->async_cancel_all_goals();

}

// GripperCommand service client:

void ContinuousMissionOrchestrator::gripperCommandResponseCallback(rclcpp::Client<iii_drone_interfaces::srv::GripperCommand>::SharedFuture future) {

    bool gripper_command_success = false;

    auto response = future.get();

    if (response->gripper_command_response == iii_drone_interfaces::srv::GripperCommand::Response::GRIPPER_COMMAND_RESPONSE_SUCCESS) {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::gripperCommandResponseCallback(): GripperCommand service succeeded");

        gripper_command_success = true;

    } else {

        RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::gripperCommandResponseCallback(): GripperCommand service failed");

        gripper_command_success = false;

    }

    std::lock_guard<std::mutex> lock(gripper_command_mutex_);
    gripper_command_response_received_ = true;
    gripper_command_success_ = gripper_command_success;
    gripper_command_active_ = false;

}

void ContinuousMissionOrchestrator::sendGripperCommand(bool close) {

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::sendGripperCommand(): Sending GripperCommand service request to %s gripper", close ? "close" : "open");

    {
        std::lock_guard<std::mutex> lock(gripper_command_mutex_);

        gripper_command_response_received_ = false;
        gripper_command_active_ = true;
        gripper_command_success_ = false;
    }

    auto request = std::make_shared<iii_drone_interfaces::srv::GripperCommand::Request>();
    request->gripper_command = close ? iii_drone_interfaces::srv::GripperCommand::Request::GRIPPER_COMMAND_CLOSE : iii_drone_interfaces::srv::GripperCommand::Request::GRIPPER_COMMAND_OPEN;

    auto future = gripper_command_client_->async_send_request(request, std::bind(&ContinuousMissionOrchestrator::gripperCommandResponseCallback, this, std::placeholders::_1));

}

bool ContinuousMissionOrchestrator::gripperCommandTerminated() {

    std::lock_guard<std::mutex> lock(gripper_command_mutex_);

    return (!gripper_command_active_) && gripper_command_response_received_;

}

bool ContinuousMissionOrchestrator::gripperCommandSucceeded() {

    std::lock_guard<std::mutex> lock(gripper_command_mutex_);

    return gripper_command_success_;

}

bool ContinuousMissionOrchestrator::getLastGripperCommandClose() {

    std::lock_guard<std::mutex> lock(gripper_command_mutex_);

    return last_gripper_command_close_;

}

// Target cable service:

void ContinuousMissionOrchestrator::setTargetCableIdSrvCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::SetTargetCableId::Request> request,
    const std::shared_ptr<iii_drone_interfaces::srv::SetTargetCableId::Response> response
) {

    (void)request_header;

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::setTargetCableIdSrvCallback(): Received SetTargetCableId service request with target_cable_id = %d", request->target_cable_id);

    setTargetCableId(request->target_cable_id);
    setTargetCableIdChanged(true);

}

int8_t ContinuousMissionOrchestrator::getTargetCableId() {

    std::lock_guard<std::mutex> lock(target_cable_id_mutex_);

    return target_cable_id_;

}

void ContinuousMissionOrchestrator::setTargetCableId(int8_t target_cable_id) {

    std::lock_guard<std::mutex> lock(target_cable_id_mutex_);

    target_cable_id_ = target_cable_id;

}

bool ContinuousMissionOrchestrator::targetCableIdChanged() {

    std::lock_guard<std::mutex> lock(target_cable_id_mutex_);

    return target_cable_id_changed_;

}

void ContinuousMissionOrchestrator::setTargetCableIdChanged(bool changed) {

    std::lock_guard<std::mutex> lock(target_cable_id_mutex_);

    target_cable_id_changed_ = changed;

}

// Charger/gripper status:

void ContinuousMissionOrchestrator::batteryVoltageCallback(const std_msgs::msg::Float32::SharedPtr msg) {

    float tlo, thi;
    this->get_parameter("battery_voltage_low_threshold", tlo);
    this->get_parameter("battery_voltage_high_threshold", thi);

    std::lock_guard<std::mutex> lock(battery_voltage_mutex_);

    battery_voltage_ = msg->data;

    if (battery_voltage_ < tlo) {

        battery_voltage_low_ = true;
        battery_voltage_high_ = false;

    } else if (battery_voltage_ > thi) {

        battery_voltage_low_ = false;
        battery_voltage_high_ = true;

    } else {

        battery_voltage_low_ = false;
        battery_voltage_high_ = false;

    }
}

float ContinuousMissionOrchestrator::getBatteryVoltage() {

    std::lock_guard<std::mutex> lock(battery_voltage_mutex_);

    return battery_voltage_;

}

bool ContinuousMissionOrchestrator::batteryVoltageLow() {

    std::lock_guard<std::mutex> lock(battery_voltage_mutex_);

    return battery_voltage_low_;

}

void ContinuousMissionOrchestrator::setBatteryVoltageLow(bool battery_voltage_low) {

    std::lock_guard<std::mutex> lock(battery_voltage_mutex_);

    battery_voltage_low_ = battery_voltage_low;

}

bool ContinuousMissionOrchestrator::batteryVoltageHigh() {

    std::lock_guard<std::mutex> lock(battery_voltage_mutex_);

    return battery_voltage_high_;

}

void ContinuousMissionOrchestrator::setBatteryVoltageHigh(bool battery_voltage_high) {

    std::lock_guard<std::mutex> lock(battery_voltage_mutex_);

    battery_voltage_high_ = battery_voltage_high;

}

void ContinuousMissionOrchestrator::chargingPowerCallback(const std_msgs::msg::Float32::SharedPtr msg) {

    float tlo;
    this->get_parameter("charging_power_low_threshold", tlo);

    std::lock_guard<std::mutex> lock(charging_power_mutex_);

    charging_power_ = msg->data;

    if (charging_power_ < tlo) {

        charging_power_low_ = true;

    } else {

        charging_power_low_ = false;

    }
}

float ContinuousMissionOrchestrator::getChargingPower() {

    std::lock_guard<std::mutex> lock(charging_power_mutex_);

    return charging_power_;

}

bool ContinuousMissionOrchestrator::chargingPowerLow() {

    std::lock_guard<std::mutex> lock(charging_power_mutex_);

    return charging_power_low_;

}

void ContinuousMissionOrchestrator::chargerOperatingModeCallback(const iii_drone_interfaces::msg::ChargerOperatingMode::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(charger_operating_mode_mutex_);

    charger_operating_mode_ = *msg;

}

iii_drone_interfaces::msg::ChargerOperatingMode ContinuousMissionOrchestrator::getChargerOperatingMode() {

    std::lock_guard<std::mutex> lock(charger_operating_mode_mutex_);

    return charger_operating_mode_;

}

void ContinuousMissionOrchestrator::chargerStatusCallback(const iii_drone_interfaces::msg::ChargerStatus::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    charger_status_ = *msg;

    if (charger_status_.charger_status == charger_status_.CHARGER_STATUS_DISABLED) {

        charger_status_disabled_ = true;
        charger_status_charging_ = false;
        charger_status_fully_charged_ = false;

    } else if (charger_status_.charger_status == charger_status_.CHARGER_STATUS_CHARGING) {

        charger_status_disabled_ = false;
        charger_status_charging_ = true;
        charger_status_fully_charged_ = false;

    } else if (charger_status_.charger_status == charger_status_.CHARGER_STATUS_FULLY_CHARGED) {

        charger_status_disabled_ = false;
        charger_status_charging_ = false;
        charger_status_fully_charged_ = true;

    } else {

        charger_status_disabled_ = false;
        charger_status_charging_ = false;
        charger_status_fully_charged_ = false;

    }
}

iii_drone_interfaces::msg::ChargerStatus ContinuousMissionOrchestrator::getChargerStatus() {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    return charger_status_;

}

// getChargerStatusDisabled:
bool ContinuousMissionOrchestrator::chargerStatusDisabled() {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    return charger_status_disabled_;

}

// set
void ContinuousMissionOrchestrator::setChargerStatusDisabled(bool charger_status_disabled) {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    charger_status_disabled_ = charger_status_disabled;

}

// get/set charger status charging:
bool ContinuousMissionOrchestrator::chargerStatusCharging() {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    return charger_status_charging_;

}

void ContinuousMissionOrchestrator::setChargerStatusCharging(bool charger_status_charging) {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    charger_status_charging_ = charger_status_charging;

}

// get/set charger status fully charged:
bool ContinuousMissionOrchestrator::chargerStatusFullyCharged() {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    return charger_status_fully_charged_;

}

void ContinuousMissionOrchestrator::setChargerStatusFullyCharged(bool charger_status_fully_charged) {

    std::lock_guard<std::mutex> lock(charger_status_mutex_);

    charger_status_fully_charged_ = charger_status_fully_charged;

}


void ContinuousMissionOrchestrator::gripperStatusCallback(const iii_drone_interfaces::msg::GripperStatus::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(gripper_status_mutex_);

    gripper_status_ = *msg;

}

iii_drone_interfaces::msg::GripperStatus ContinuousMissionOrchestrator::getGripperStatus() {

    std::lock_guard<std::mutex> lock(gripper_status_mutex_);

    return gripper_status_;

}

bool ContinuousMissionOrchestrator::gripperIsClosed() {

    bool use_charger_gripper_info;
    this->get_parameter("use_charger_gripper_info", use_charger_gripper_info);

    const char* simulation_env = std::getenv("SIMULATION");
    bool simulation = (simulation_env != nullptr) ? (std::string(simulation_env) == "True") : false;

    use_charger_gripper_info &= !simulation;

    if (use_charger_gripper_info) {

        iii_drone_interfaces::msg::GripperStatus gripper_status = getGripperStatus();

        return gripper_status.gripper_status == iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED;

    } else {

        return getLastGripperCommandClose() && gripperCommandSucceeded();

    }
}

bool ContinuousMissionOrchestrator::gripperIsOpen() {

    bool use_charger_gripper_info;
    this->get_parameter("use_charger_gripper_info", use_charger_gripper_info);

    const char* simulation_env = std::getenv("SIMULATION");
    bool simulation = (simulation_env != nullptr) ? (std::string(simulation_env) == "True") : false;

    use_charger_gripper_info &= !simulation;

    if (use_charger_gripper_info) {

        iii_drone_interfaces::msg::GripperStatus gripper_status = getGripperStatus();

        return gripper_status.gripper_status == iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_OPEN;

    } else {

        return !getLastGripperCommandClose() && gripperCommandSucceeded();

    }
}

// Charging:

rclcpp::Time ContinuousMissionOrchestrator::getChargingStartTime() {

    std::lock_guard<std::mutex> lock(charging_start_time_mutex_);

    return charging_start_time_;

}

void ContinuousMissionOrchestrator::initiateChargingSrvCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::InitiateCharging::Request> request,
    const std::shared_ptr<iii_drone_interfaces::srv::InitiateCharging::Response> response
) {

    (void)request_header;

    state_t state = getState();

    if (state != inspecting) {

        RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::initiateChargingSrvCallback(): Received request to initiate charging, but can only initiate charging when in state inspecting.");

        response->success = false;

        return;

    }

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::initiateChargingSrvCallback(): Received request to initiate charging");

    std::lock_guard<std::mutex> lock(initiate_charging_srv_called_mutex_);

    initiate_charging_srv_called_ = true;

    response->success = true;

}

bool ContinuousMissionOrchestrator::initiateChargingSrvCalled() {

    std::lock_guard<std::mutex> lock(initiate_charging_srv_called_mutex_);

    return initiate_charging_srv_called_;

}

void ContinuousMissionOrchestrator::clearInitiateChargingSrvCalled() {

    std::lock_guard<std::mutex> lock(initiate_charging_srv_called_mutex_);

    initiate_charging_srv_called_ = false;

}

void ContinuousMissionOrchestrator::interruptChargingSrvCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::InterruptCharging::Request> request,
    const std::shared_ptr<iii_drone_interfaces::srv::InterruptCharging::Response> response
) {

    (void)request_header;

    state_t state = getState();

    if (state != charging) {

        RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::interruptChargingSrvCallback(): Received request to interrupt charging, but can only interrupt charging when in state charging.");

        response->success = false;

        return;

    }

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::interruptChargingSrvCallback(): Received request to interrupt charging");

    std::lock_guard<std::mutex> lock(interrupt_charging_srv_called_mutex_);

    interrupt_charging_srv_called_ = true;

    response->success = true;

}

bool ContinuousMissionOrchestrator::interruptChargingSrvCalled() {

    std::lock_guard<std::mutex> lock(interrupt_charging_srv_called_mutex_);

    return interrupt_charging_srv_called_;

}

void ContinuousMissionOrchestrator::clearInterruptChargingSrvCalled() {

    std::lock_guard<std::mutex> lock(interrupt_charging_srv_called_mutex_);

    interrupt_charging_srv_called_ = false;

}

void ContinuousMissionOrchestrator::prolongChargingSrvCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<iii_drone_interfaces::srv::ProlongCharging::Request> request,
    const std::shared_ptr<iii_drone_interfaces::srv::ProlongCharging::Response> response
) {

    (void)request_header;

    state_t state = getState();

    if (state != charging) {

        RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::prolongChargingSrvCallback(): Received request to prolong charging, but vehicle is not charging");

        response->success = false;

        return;

    }

    RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::prolongChargingSrvCallback(): Received request to prolong charging");

    std::lock_guard<std::mutex> lock(prolong_charging_mutex_);

    if (request->prolong_mode == iii_drone_interfaces::srv::ProlongCharging::Request::PROLONG_MODE_UNTIL_INTERRUPTED) {

        prolong_charging_until_interrupted_ = true;
        prolong_charging_for_seconds_ = -1;
        prolong_charging_until_new_battery_voltage_ = -1;

    } else if (request->prolong_mode == iii_drone_interfaces::srv::ProlongCharging::Request::PROLONG_MODE_FOR_TIME) {

        RCLCPP_FATAL(this->get_logger(), "ContinuousMissionOrchestrator::prolongChargingSrvCallback(): PROLONG_MODE_FOR_TIME not implemented yet");

        prolong_charging_until_interrupted_ = false;
        prolong_charging_for_seconds_ = request->prolong_time_s;
        prolong_charging_until_new_battery_voltage_ = -1;

    } else if (request->prolong_mode == iii_drone_interfaces::srv::ProlongCharging::Request::PROLONG_MODE_UNTIL_NEW_BATTERY_VOLTAGE) {

        RCLCPP_FATAL(this->get_logger(), "ContinuousMissionOrchestrator::prolongChargingSrvCallback(): PROLONG_MODE_UNTIL_NEW_BATTERY_VOLTAGE not implemented yet");

        prolong_charging_until_interrupted_ = false;
        prolong_charging_for_seconds_ = -1;
        prolong_charging_until_new_battery_voltage_ = request->new_battery_voltage;

    } else if (request->prolong_mode == iii_drone_interfaces::srv::ProlongCharging::Request::PROLONG_MODE_FOR_TIME_OR_NEW_BATTERY_VOLTAGE) {

        RCLCPP_FATAL(this->get_logger(), "ContinuousMissionOrchestrator::prolongChargingSrvCallback(): PROLONG_MODE_FOR_TIME_OR_NEW_BATTERY_VOLTAGE not implemented yet");

        prolong_charging_until_interrupted_ = false;
        prolong_charging_for_seconds_ = request->prolong_time_s;
        prolong_charging_until_new_battery_voltage_ = request->new_battery_voltage;

    } else if (request->prolong_mode == iii_drone_interfaces::srv::ProlongCharging::Request::PROLONG_MODE_CLEAR) {

        prolong_charging_until_interrupted_ = false;
        prolong_charging_for_seconds_ = -1;
        prolong_charging_until_new_battery_voltage_ = -1;

    } else {

        RCLCPP_ERROR(this->get_logger(), "ContinuousMissionOrchestrator::prolongChargingSrvCallback(): Invalid prolong mode");

        response->success = false;

        return;

    }

    response->success = true;

}

bool ContinuousMissionOrchestrator::prolongChargingUntilInterrupted() {

    std::lock_guard<std::mutex> lock(prolong_charging_mutex_);

    return prolong_charging_until_interrupted_;

}

float ContinuousMissionOrchestrator::getProlongChargingForSeconds() {

    std::lock_guard<std::mutex> lock(prolong_charging_mutex_);

    return prolong_charging_for_seconds_;

}

float ContinuousMissionOrchestrator::getProlongChargingUntilNewBatteryVoltage() {

    std::lock_guard<std::mutex> lock(prolong_charging_mutex_);

    return prolong_charging_until_new_battery_voltage_;

}

void ContinuousMissionOrchestrator::clearProlongCharging() {

    std::lock_guard<std::mutex> lock(prolong_charging_mutex_);

    prolong_charging_until_interrupted_ = false;
    prolong_charging_for_seconds_ = -1;
    prolong_charging_until_new_battery_voltage_ = -1;

}

void ContinuousMissionOrchestrator::evaluateChargingStatus() {

    bool start_charging_flag = false;
    bool stop_charging_flag = false;

    int charging_minimum_duration_s;
    int charging_maximum_duration_s;
    bool use_charger_gripper_info;
    bool simulation;
    float charging_duration_s;
    bool has_charged_minimum_seconds;
    bool has_charged_maximum_seconds;

    // Get state:
    state_t state = getState();

    switch (state) {

        case manual:
        case wait_for_target_line:
        case flying_under_cable:

            start_charging_flag = false;
            stop_charging_flag = false;

            clearInitiateChargingSrvCalled();
            clearInterruptChargingSrvCalled();
            clearProlongCharging();

            break;

        case inspecting:
        case landing_on_cable:
        case closing_gripper:
        case disarming_on_cable:

            start_charging_flag = batteryVoltageLow() || initiateChargingSrvCalled();
            stop_charging_flag = false;

            clearInterruptChargingSrvCalled();
            clearProlongCharging();

            break;

        case charging:
        case arming_on_cable:
        case opening_gripper:
        case taking_off_from_cable:

            stop_charging_flag = false;

            charging_minimum_duration_s;
            this->get_parameter("charging_minimum_duration_s", charging_minimum_duration_s);

            charging_maximum_duration_s;
            this->get_parameter("charging_maximum_duration_s", charging_maximum_duration_s);

            use_charger_gripper_info;
            this->get_parameter("use_charger_gripper_info", use_charger_gripper_info);

            const char* simulation_env = std::getenv("SIMULATION");
            simulation = (simulation_env != nullptr) ? (std::string(simulation_env) == "True") : false;

            use_charger_gripper_info &= !simulation;

            charging_duration_s = (this->get_clock()->now() - charging_start_time_).seconds();

            has_charged_minimum_seconds = (charging_minimum_duration_s < 0) || (charging_duration_s >= charging_minimum_duration_s);

            has_charged_maximum_seconds = (charging_maximum_duration_s > 0) && (charging_duration_s >= charging_maximum_duration_s);

            if (!prolongChargingUntilInterrupted()) {

                if (use_charger_gripper_info) {

                    stop_charging_flag |= batteryVoltageHigh() || chargingPowerLow() || !chargerStatusCharging();
                    stop_charging_flag &= has_charged_minimum_seconds;

                }

                stop_charging_flag |= has_charged_maximum_seconds;

            }

            stop_charging_flag |= interruptChargingSrvCalled();

            start_charging_flag = false;

            clearInitiateChargingSrvCalled();
            clearProlongCharging();

            break;

        default:
        case error:
            start_charging_flag = false;
            stop_charging_flag = false;

            clearInitiateChargingSrvCalled();
            clearInterruptChargingSrvCalled();
            clearProlongCharging();

            break;

    }

    if (start_charging_flag) {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::evaluateChargingStatus(): Start charging flag set");

    }

    if (stop_charging_flag) {

        RCLCPP_INFO(this->get_logger(), "ContinuousMissionOrchestrator::evaluateChargingStatus(): Stop charging flag set");

    }

    setStartChargingFlag(start_charging_flag);
    setStopChargingFlag(stop_charging_flag);
    
}


bool ContinuousMissionOrchestrator::getStartChargingFlag() {

    std::lock_guard<std::mutex> lock(start_charging_flag_mutex_);

    return start_charging_flag_;

}

void ContinuousMissionOrchestrator::setStartChargingFlag(bool start_charging_flag) {

    std::lock_guard<std::mutex> lock(start_charging_flag_mutex_);

    start_charging_flag_ = start_charging_flag;

}

bool ContinuousMissionOrchestrator::getStopChargingFlag() {

    std::lock_guard<std::mutex> lock(stop_charging_flag_mutex_);

    return stop_charging_flag_;

}

void ContinuousMissionOrchestrator::setStopChargingFlag(bool stop_charging_flag) {

    std::lock_guard<std::mutex> lock(stop_charging_flag_mutex_);

    stop_charging_flag_ = stop_charging_flag;

}

// Control state:
void ContinuousMissionOrchestrator::controlStateCallback(const iii_drone_interfaces::msg::ControlState::SharedPtr msg) {

    setControlState(*msg);

}

iii_drone_interfaces::msg::ControlState ContinuousMissionOrchestrator::getControlState() {

    std::lock_guard<std::mutex> lock(control_state_mutex_);

    return control_state_;

}

void ContinuousMissionOrchestrator::setControlState(iii_drone_interfaces::msg::ControlState control_state) {

    std::lock_guard<std::mutex> lock(control_state_mutex_);

    control_state_ = control_state;

}

bool ContinuousMissionOrchestrator::isInControllableState() {

    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    return isInControllableState(control_state);

}

bool ContinuousMissionOrchestrator::isInControllableState(iii_drone_interfaces::msg::ControlState control_state) {

    switch(control_state.state) {

        case iii_drone_interfaces::msg::ControlState::CONTROL_STATE_INIT:
        case iii_drone_interfaces::msg::ControlState::CONTROL_STATE_ON_GROUND_NON_OFFBOARD:
        case iii_drone_interfaces::msg::ControlState::CONTROL_STATE_IN_FLIGHT_NON_OFFBOARD:
        case iii_drone_interfaces::msg::ControlState::CONTROL_STATE_ARMING:
        case iii_drone_interfaces::msg::ControlState::CONTROL_STATE_SETTING_OFFBOARD:
        case iii_drone_interfaces::msg::ControlState::CONTROL_STATE_TAKING_OFF:
        case iii_drone_interfaces::msg::ControlState::CONTROL_STATE_LANDING:

            return false;

        default:

            return true;

    }
}

// State machine:
ContinuousMissionOrchestrator::state_t ContinuousMissionOrchestrator::getState() {

    std::lock_guard<std::mutex> lock(state_mutex_);

    return state_;

}

void ContinuousMissionOrchestrator::setState(state_t state) {

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        state_ = state;
    }

    switch(state) {

        case flying_under_cable:

            setTargetCableIdChanged(false);

            break;

        case charging:

            {
                std::lock_guard<std::mutex> lock(charging_start_time_mutex_);

                charging_start_time_ = this->get_clock()->now();
            }

            break;

        default:
            break;

    }
}

void ContinuousMissionOrchestrator::stateMachineCallback() {

    evaluateChargingStatus();

    state_t state = getState();

    switch(state) {

        default:
        case manual:
            stateMachineManual();
            break;

        case wait_for_target_line:
            stateMachineWaitForTargetLine();
            break;

        case flying_under_cable:
            stateMachineFlyingUnderCable();
            break;

        case inspecting:
            stateMachineInspecting();
            break;

        case closing_gripper:
            stateMachineClosingGripper();
            break;

        case landing_on_cable:
            stateMachineLandingOnCable();
            break;

        case disarming_on_cable:
            stateMachineDisarmingOnCable();
            break;

        case charging:
            stateMachineCharging();
            break;

        case arming_on_cable:
            stateMachineArmingOnCable();
            break;

        case opening_gripper:
            stateMachineOpeningGripper();
            break;

        case taking_off_from_cable:
            stateMachineTakingOffFromCable();
            break;

        case error:
            stateMachineError();
            break;

    }

}

void ContinuousMissionOrchestrator::stateMachineManual() {

    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_HOVERING || control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_HOVERING_UNDER_CABLE) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineManual(): Vehicle is controllable, entering wait_for_target_line state");

        setState(wait_for_target_line);

    }
}

void ContinuousMissionOrchestrator::stateMachineWaitForTargetLine() {

    if (!isInControllableState()) {

        RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineWaitForTargetLine(): Vehicle is not controllable, entering manual state");
        
        setTargetCableId(-1);
        
        setState(manual);

        return;

    }

    int8_t target_cable_id = getTargetCableId();

    if (target_cable_id >= 0) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineWaitForTargetLine(): Target cable id %d registered, entering flying_under_cable state", target_cable_id);

        startFlyUnderCable();
        setState(flying_under_cable);

    }
}

void ContinuousMissionOrchestrator::stateMachineFlyingUnderCable() {

    static bool action_timer_started = false;

    // Check control state:
    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (!isInControllableState(control_state)) {

        RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineFlyingUnderCable(): Vehicle is not controllable, entering manual state");

        action_timer_started = false;
            
        cancelFlyUnderCable();

        setTargetCableId(-1);

        setState(manual);

        return;

    }

    if (!flyUnderCableTerminated()) {

        return;

    }

    if (flyUnderCableSucceeded() && control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_HOVERING_UNDER_CABLE) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineFlyingUnderCable(): Vehicle is hovering under cable, entering inspecting state");

        action_timer_started = false;

        setState(inspecting);

    } else if (flyUnderCableSucceeded()) {

        if (!action_timer_started) {

            RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineFlyingUnderCable(): FlyUnderCable action succeeded but vehicle is not hovering under cable, starting action timer");

            action_timer_started = true;

            startActionTimeoutCounter();

        } else if (actionTimeout()) {

            RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineFlyingUnderCable(): FlyUnderCable action succeeded but vehicle is not hovering under cable, action timer timed out, entering error state");

            setState(error);

        }

    } else if (!flyUnderCableSucceeded()) {

        RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineFlyingUnderCable(): FlyUnderCable action failed, resetting target cable id and entering wait_for_target_line state");

        action_timer_started = false;

        setTargetCableId(-1);

        setState(wait_for_target_line);

    }
}

void ContinuousMissionOrchestrator::stateMachineInspecting() {

    // Check control state:
    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (control_state.state != iii_drone_interfaces::msg::ControlState::CONTROL_STATE_HOVERING_UNDER_CABLE) {

        RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineInspecting(): Vehicle is not hovering under cable, resetting target cable id and entering manual state");

        setTargetCableId(-1);

        setState(manual);

        return;

    }

    // Check if target line has changed:
    int8_t target_cable_id = getTargetCableId();

    if (target_cable_id < 0) {

        RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineInspecting(): New target cable id is negative, entering wait_for_target_line state");

        setState(wait_for_target_line);

        return;

    } else if (targetCableIdChanged()) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineInspecting(): New target cable id %d registered, entering flying_under_cable state", target_cable_id);

        startFlyUnderCable();
        setState(flying_under_cable);

        return;

    }

    // // Check start charging flag:
    // if (getStartChargingFlag()) {

    //     RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineInspecting(): Start charging flag set, entering landing_on_cable state");

    //     this->get_parameter("cable_landing_max_retries", cable_landing_counter_);
    //     cable_landing_counter_--;

    //     if (cable_landing_counter_ < 0) {
            
    //         RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Parameter cable_landing_max_retries below 1. Continuing with a single attempt.");
    //         cable_landing_counter_ = 0;
    //     }

    //     startCableLanding();
    //     setState(landing_on_cable);

    //     return;

    // }

    // Check start charging flag:
    if (getStartChargingFlag()) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineInspecting(): Start charging flag set, entering closing_gripper state");

        sendGripperCommand(true);
        setState(closing_gripper);

        return;

    }


}

void ContinuousMissionOrchestrator::stateMachineLandingOnCable() {

    static bool action_timer_started = false;

    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (!isInControllableState(control_state)) {

        RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Vehicle is not controllable, entering manual state");

        action_timer_started = false;

        cancelCableLanding();

        setTargetCableId(-1);

        setState(manual);

        return;

    }

    if (!cableLandingTerminated()) {

        return;

    }

    iii_drone_interfaces::msg::GripperStatus gripper_status = getGripperStatus();
    bool gripper_is_closed = gripper_status.gripper_status == iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED;

    if (cableLandingSucceeded() && control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_ON_CABLE_ARMED && gripper_is_closed) {

        action_timer_started = false;

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded and gripper is closed, entering disarming_on_cable state");

        startDisarmOnCable();
        setState(disarming_on_cable);

    } else if (cableLandingSucceeded()) {

        if (!action_timer_started) {

            RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded but vehicle is not on cable armed or gripper is not closed, starting action timer");

            action_timer_started = true;

            startActionTimeoutCounter();

        } else if (actionTimeout()) {

            RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded but vehicle is not on cable armed or gripper is not closed and action timer expired, entering error state");

            setState(error);

        }

    } else if (!cableLandingSucceeded()) {

        action_timer_started = false;

        if (control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_HOVERING_UNDER_CABLE) {

            if (cable_landing_counter_ <= 0) {

                RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing failed and no retries left, entering error state");

                setState(error);

                return;

            } else {

                RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing failed, %d retr%s left, entering landing_on_cable state", cable_landing_counter_, cable_landing_counter_ == 1 ? "y" : "ies");

                cable_landing_counter_--;

                startCableLanding();

            }

        } else if (control_state.state != iii_drone_interfaces::msg::ControlState::CONTROL_STATE_DURING_CABLE_TAKEOFF && control_state.state != iii_drone_interfaces::msg::ControlState::CONTROL_STATE_DURING_CABLE_LANDING) {

            RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing failed and vehicle is not hovering under cable, not during cable takeoff, and not during cable landing, entering error state");

            setState(error);

        }
    }













    // if (cableLandingSucceeded() && control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_ON_CABLE_ARMED) {

    //     action_timer_started = false;

    //     // if (!gripperIsClosed()) {

    //     //     RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded but gripper is not closed, entering error state");

    //     //     setState(error);

    //     // } else {

    //     //     RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded and gripper is closed, entering disarming_on_cable state");

    //     // }

    //     RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded, entering closing_gripper state");

    //     sendGripperCommand(true);
    //     setState(closing_gripper);


    // } else if (cableLandingSucceeded()) {

    //     if (!action_timer_started) {

    //         RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded but vehicle is not on cable armed, starting action timer");

    //         action_timer_started = true;

    //         startActionTimeoutCounter();

    //     } else if (actionTimeout()) {

    //         RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing succeeded but vehicle is not on cable armed and action timer expired, entering error state");

    //         setState(error);

    //     }

    // } else if (!cableLandingSucceeded()) {

    //     action_timer_started = false;

    //     if (control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_HOVERING_UNDER_CABLE) {

    //         if (cable_landing_counter_ <= 0) {

    //             RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing failed and no retries left, entering error state");

    //             setState(error);

    //             return;

    //         } else {

    //             RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing failed, %d retr%s left, entering landing_on_cable state", cable_landing_counter_, cable_landing_counter_ == 1 ? "y" : "ies");

    //             cable_landing_counter_--;

    //             startCableLanding();

    //         }

    //     } else if (control_state.state != iii_drone_interfaces::msg::ControlState::CONTROL_STATE_DURING_CABLE_LANDING) {

    //         RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineLandingOnCable(): Cable landing failed and vehicle is not hovering under cable, entering error state");

    //         setState(error);

    //     }
    // }
}

void ContinuousMissionOrchestrator::stateMachineClosingGripper() {

    static bool action_timer_started = false;

    if (!isInControllableState()) {

        RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Vehicle is not controllable, entering manual state");

        action_timer_started = false;

        sendGripperCommand(false);

        setTargetCableId(-1);

        setState(manual);

        return;

    }

    if (!gripperCommandTerminated()) {

        return;

    }

    // iii_drone_interfaces::msg::GripperStatus gripper_status = getGripperStatus();
    // bool gripper_is_closed = gripper_status.gripper_status == iii_drone_interfaces::msg::GripperStatus::GRIPPER_STATUS_CLOSED;

    // if (gripperCommandSucceeded() && gripper_is_closed) {

    //     RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Gripper command succeeded and gripper is closed, entering disarming_on_cable state");

    //     action_timer_started = false;

    //     startDisarmOnCable();
    //     setState(disarming_on_cable);

    // } else if (gripperCommandSucceeded()) {

    //     if (!action_timer_started) {

    //         RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Gripper command succeeded but gripper is not closed, starting action timer");

    //         action_timer_started = true;

    //         startActionTimeoutCounter();

    //     } else if (actionTimeout()) {

    //         RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Gripper command succeeded but gripper is not closed and gripper timer expired, entering taking_off_from_cable state");

    //         action_timer_started = false;

    //         startCableTakeoff();
    //         setState(taking_off_from_cable);

    //     }

    if (gripperCommandSucceeded()) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Gripper command succeeded, entering landing_on_cable state");

        this->get_parameter("cable_landing_max_retries", cable_landing_counter_);
        cable_landing_counter_--;

        if (cable_landing_counter_ < 0) {
            
            RCLCPP_WARN(this->get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Parameter cable_landing_max_retries below 1. Continuing with a single attempt.");
            cable_landing_counter_ = 0;
        }

        startCableLanding();
        setState(landing_on_cable);

        return;

    } else {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineClosingGripper(): Gripper command failed, entering error state");

        setState(error);

    }
}


void ContinuousMissionOrchestrator::stateMachineDisarmingOnCable() {

    static bool action_timer_started = false;

    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (!isInControllableState(control_state)) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineDisarmingOnCable(): Vehicle is not controllable, entering error state");

        setState(error);

        return;

    }

    if (!disarmOnCableTerminated()) {

        return;

    }

    if (disarmOnCableSucceeded() && control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_ON_CABLE_DISARMED) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineDisarmingOnCable(): Disarm on cable succeeded and vehicle is disarmed on cable, entering charging state");

        action_timer_started = false;

        setState(charging);

    } else if (disarmOnCableSucceeded()) {

        if (!action_timer_started) {

            RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineDisarmingOnCable(): Disarm on cable succeeded but vehicle is not disarmed on cable, starting action timer");

            action_timer_started = true;

            startActionTimeoutCounter();

        } else if (actionTimeout()) {

            RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineDisarmingOnCable(): Disarm on cable succeeded but vehicle is not disarmed on cable and action timer expired, entering error state");

            setState(error);

        }

    } else if (!disarmOnCableSucceeded()) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineDisarmingOnCable(): Disarm on cable failed, entering error state");

        action_timer_started = false;

        setState(error);

    }
}

void ContinuousMissionOrchestrator::stateMachineCharging() {

    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (control_state.state != iii_drone_interfaces::msg::ControlState::CONTROL_STATE_ON_CABLE_DISARMED) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineCharging(): Vehicle is not disarmed on cable, entering error state");

        setState(error);

        return;

    }

    if (getStopChargingFlag()) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineCharging(): Stop charging flag set, entering arming_on_cable state");

        startArmOnCable();

        setState(arming_on_cable);

        return;

    }

}

void ContinuousMissionOrchestrator::stateMachineArmingOnCable() {

    static bool action_timer_started = false;

    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (!isInControllableState(control_state)) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineArmingOnCable(): Vehicle is not controllable, entering error state");

        setState(error);

        return;

    }

    if (!armOnCableTerminated()) {

        return;

    }

    if (armOnCableSucceeded() && control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_ON_CABLE_ARMED) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineArmingOnCable(): Arm on cable succeeded and vehicle is armed on cable, entering opening_gripper state");

        action_timer_started = false;

        sendGripperCommand(false);
        
        setState(opening_gripper);

    } else if (armOnCableSucceeded()) {

        if (!action_timer_started) {

            RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineArmingOnCable(): Arm on cable succeeded but vehicle is not armed on cable, starting action timer");

            action_timer_started = true;

            startActionTimeoutCounter();

        } else if (actionTimeout()) {

            RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineArmingOnCable(): Arm on cable succeeded but vehicle is not armed on cable and action timer expired, entering error state");

            setState(error);

        }

    } else if (!armOnCableSucceeded()) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineArmingOnCable(): Arm on cable failed, entering error state");

        setState(error);

    }
}

void ContinuousMissionOrchestrator::stateMachineOpeningGripper() {

    static bool action_timer_started = false;

    if (!isInControllableState()) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineOpeningGripper(): Vehicle is not controllable, entering error state");

        setState(error);

        return;

    }

    if (!gripperCommandTerminated()) {

        return;

    }

    if (gripperCommandSucceeded() && gripperIsOpen()) {

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineOpeningGripper(): Gripper command succeeded and gripper is open, entering taking_off_from_cable state");

        action_timer_started = false;

        startCableTakeoff();
        setState(taking_off_from_cable);

    } else if (gripperCommandSucceeded()) {

        if (!action_timer_started) {

            RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineOpeningGripper(): Gripper command succeeded but gripper is not open, starting action timer");

            action_timer_started = true;

            startActionTimeoutCounter();

        } else if (actionTimeout()) {

            RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineOpeningGripper(): Gripper command succeeded but gripper is not open and action timer expired, entering error state");

            setState(error);

        }

    } else if (!gripperCommandSucceeded()) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineOpeningGripper(): Gripper command failed, entering error state");

        setState(error);

    }
}

void ContinuousMissionOrchestrator::stateMachineTakingOffFromCable() {

    static bool action_timer_started = false;

    iii_drone_interfaces::msg::ControlState control_state = getControlState();

    if (!isInControllableState(control_state)) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineTakingOffFromCable(): Vehicle is not controllable, entering error state");

        setState(error);

        return;

    }

    if (!cableTakeoffTerminated()) {

        return;

    }

    if (cableTakeoffSucceeded() && control_state.state == iii_drone_interfaces::msg::ControlState::CONTROL_STATE_HOVERING_UNDER_CABLE) {

        if (getStartChargingFlag()) {

            if (cable_landing_counter_ <= 0) {

                RCLCPP_ERROR(get_logger(), "ContinuousMissionOrchestrator::stateMachineTakingOffFromCable(): Cable takeoff succeeded and vehicle is hovering under cable and start charging flag is still set, but no cable landing retries left, entering error state");

                setState(error);

            } else {

                RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineTakingOffFromCable(): Cable takeoff succeeded and vehicle is hovering under cable and start charging flag is still set, entering landing_on_cable state");

                action_timer_started = false;

                cable_landing_counter_--;
                startCableLanding();
                setState(landing_on_cable);

            }

            return;

        }

        RCLCPP_INFO(get_logger(), "ContinuousMissionOrchestrator::stateMachineTakingOffFromCable(): Cable takeoff succeeded and vehicle is hovering under cable, entering inspecting state");

        action_timer_started = false;

        setState(inspecting);

    } else if (cableTakeoffSucceeded()) {

        if (!action_timer_started) {

            RCLCPP_WARN(get_logger(), "ContinuousMissionOrchestrator::stateMachineTakingOffFromCable(): Cable takeoff succeeded but vehicle is not hovering under cable, starting action timer");

            action_timer_started = true;

            startActionTimeoutCounter();

        } else if (actionTimeout()) {

            RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineTakingOffFromCable(): Cable takeoff succeeded but vehicle is not hovering under cable and action timer expired, entering error state");

            setState(error);

        }

    } else if (!cableTakeoffSucceeded()) {

        RCLCPP_FATAL(get_logger(), "ContinuousMissionOrchestrator::stateMachineTakingOffFromCable(): Cable takeoff failed, entering error state");

        setState(error);

    }
}

void ContinuousMissionOrchestrator::stateMachineError() { 

    RCLCPP_FATAL_THROTTLE(get_logger(), *get_clock(), 1000, "ContinuousMissionOrchestrator::stateMachineError(): Error state");

}

void ContinuousMissionOrchestrator::startActionTimeoutCounter() {

    action_start_time_ = this->now();

}

bool ContinuousMissionOrchestrator::actionTimeout() {

    float action_timeout_ms;
    this->get_parameter("action_timeout_ms", action_timeout_ms);

    return (this->now() - action_start_time_).nanoseconds() / 1000000 > action_timeout_ms;

}

void ContinuousMissionOrchestrator::publishCallback() {

    // Publish state:
    std_msgs::msg::String msg;

    switch (getState())
    {
    case manual:
        msg.data = "manual";
        break;

    case wait_for_target_line:
        msg.data = "wait_for_target_line";
        break;

    case flying_under_cable:
        msg.data = "flying_under_cable";
        break;

    case inspecting:
        msg.data = "inspecting";
        break;

    case landing_on_cable:
        msg.data = "landing_on_cable";
        break;

    case closing_gripper:
        msg.data = "closing_gripper";
        break;

    case disarming_on_cable:
        msg.data = "disarming_on_cable";
        break;

    case charging:
        msg.data = "charging";
        break;

    case arming_on_cable:
        msg.data = "arming_on_cable";
        break;

    case opening_gripper:
        msg.data = "opening_gripper";
        break;

    case taking_off_from_cable:
        msg.data = "taking_off_from_cable";
        break;

    default:
    case error:
        msg.data = "error";
        break;
    
    }

    state_pub_->publish(msg);

}

int main(int argc, char* argv[]) {

	 std::cout << "Starting ContinuousMissionOrchestrator node..." << std::endl;

	rclcpp::init(argc, argv);

    auto node = std::make_shared<ContinuousMissionOrchestrator>();

	rclcpp::executors::SingleThreadedExecutor exe;

    exe.add_node(node);

    exe.spin();

	rclcpp::shutdown();

	return 0;

}