/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/mission/mission_executor.hpp>

#include <algorithm>

using namespace iii_drone::utils;
using namespace iii_drone::mission;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::adapters::px4;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

MissionExecutor::MissionExecutor(
    rclcpp_lifecycle::LifecycleNode * node,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    std::string mission_specification_file,
    rclcpp::CallbackGroup::SharedPtr odometry_sub_callback_group,
    rclcpp::executors::MultiThreadedExecutor & executor
) : node_(node),
    tf_buffer_(tf_buffer),
    odometry_sub_callback_group_(odometry_sub_callback_group),
    executor_(executor)
{

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initializing.");

    mission_specification_ = std::make_shared<MissionSpecification>(
        mission_specification_file,
        node
    );
    runtime_intent_buffer_ = std::make_shared<RuntimeIntentBuffer>();

    // Subscription
    vehicle_odometry_adapter_history_ = std::make_shared<History<VehicleOdometryAdapter>>(2);

    // odometry_sub_callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	rclcpp::QoS px4_sub_qos(rclcpp::KeepLast(1));
	px4_sub_qos.transient_local();
	px4_sub_qos.best_effort();

    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.callback_group = odometry_sub_callback_group_;

    odometry_sub_ = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        px4_sub_qos,
        [&](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            vehicle_odometry_adapter_history_->Store(VehicleOdometryAdapter(*msg));
        },
        sub_opts
    );

    // Tree provider
    tree_provider_ = std::make_shared<iii_drone::behavior::TreeProvider>(
        tf_buffer_,
        mission_specification_,
        runtime_intent_buffer_
    );

    executor_.add_node(tree_provider_);

    RCLCPP_INFO(node->get_logger(), "MissionExecutor::MissionExecutor(): Initialized.");

}

MissionExecutor::~MissionExecutor() {

    RCLCPP_INFO(node_->get_logger(), "MissionExecutor::~MissionExecutor(): Removing tree provider node from executor.");

    if (is_started_) Stop();

    if (is_configured_) Cleanup();

    executor_.remove_node(tree_provider_);

    RCLCPP_INFO(node_->get_logger(), "MissionExecutor::~MissionExecutor(): Destroying MissionExecutor.");

}

void MissionExecutor::Configure(
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator,
    rclcpp::CallbackGroup::SharedPtr get_reference_cb_group
) {

    if (is_configured_) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Configure(): Already configured.");
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Configure()");

    // Maneuver reference client
    maneuver_reference_client_ = std::make_shared<ManeuverReferenceClient>(
        node_,
        vehicle_odometry_adapter_history_,
        configurator->GetConfiguration("maneuver_reference_client"),
        get_reference_cb_group
    );

    tree_provider_->Configure(
        maneuver_reference_client_
    );

    registerIntentServices();

    is_configured_ = true;

}

void MissionExecutor::Cleanup() {

    if (!is_configured_) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Cleanup(): Not configured.");
        return;
    }

    tree_provider_->Cleanup();
    unregisterIntentServices();
    runtime_intent_buffer_->Clear();

    maneuver_reference_client_.reset();
    maneuver_reference_client_ = nullptr;

    is_configured_ = false;

}

void MissionExecutor::Start(
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator
) {

    if (is_started_) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Start(): Already started.");
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Initializing mode provider.");

    // Modes provider
    mode_provider_ = std::make_shared<iii_drone::px4::ModeProvider>(
        tree_provider_,
        mission_specification_,
        node_,
        maneuver_reference_client_,
        configurator->GetConfiguration("mode_provider")
    );

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Initializing mode executor.");

    generic_mode_executor_ = std::make_shared<iii_drone::px4::GenericModeExecutor>(
        *mode_provider_->GetMode(mission_specification_->executor_owned_mode()),
        "mode_executor",
        mission_specification_,
        mode_provider_,
        configurator->GetConfiguration("mode_executor")
    );

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Registering mode executor.");

    if (!generic_mode_executor_->doRegister()) {
        RCLCPP_FATAL(node_->get_logger(), "MissionExecutor::Start(): Mode executor registration failed.");
        throw std::runtime_error("MissionExecutor::Start(): Mode executor registration failed.");
    }

    RCLCPP_DEBUG(node_->get_logger(), "MissionExecutor::Start(): Registering modes.");

    mode_provider_->Register();

    executor_.add_node(mode_provider_->mode_node());

    is_started_ = true;

}

void MissionExecutor::Stop() {

    if (!is_started_ && generic_mode_executor_ == nullptr && mode_provider_ == nullptr) {
        RCLCPP_WARN(node_->get_logger(), "MissionExecutor::Stop(): Already stopped.");
        return;
    }

    rclcpp::Node::SharedPtr mode_node = nullptr;
    if (mode_provider_ != nullptr) {
        mode_node = mode_provider_->mode_node();
    }

    generic_mode_executor_.reset();
    generic_mode_executor_ = nullptr;

    if (mode_provider_ != nullptr) {
        mode_provider_->Stop();
        mode_provider_->Cleanup();
    }

    if (mode_node != nullptr) {
        executor_.remove_node(
            mode_node,
            true
        );
    }

    mode_provider_.reset();
    mode_provider_ = nullptr;

    is_started_ = false;

}

bool MissionExecutor::OverrideMissionSpecification(
    const std::string & mission_specification_file,
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator,
    rclcpp::CallbackGroup::SharedPtr get_reference_cb_group,
    std::string & message
) {

    std::lock_guard<std::mutex> lock(lifecycle_mutex_);

    if (mission_active()) {
        message = "mission specification override rejected because a mission is active";
        return false;
    }

    MissionSpecification::SharedPtr replacement;
    try {
        replacement = std::make_shared<MissionSpecification>(
            mission_specification_file,
            node_
        );
        replacement->GetMissionSpecificationEntry(replacement->executor_owned_mode());
    } catch (const std::exception & exception) {
        message = "mission specification override rejected while loading '" +
            mission_specification_file + "': " + exception.what();
        return false;
    }

    const bool was_configured = is_configured_;
    const bool was_started = is_started_;
    const auto previous_specification = mission_specification_;

    if (tree_provider_ != nullptr && was_configured) {
        tree_provider_->ClearGlobalBlackboard("mission specification override");
    }
    if (runtime_intent_buffer_ != nullptr) {
        runtime_intent_buffer_->Clear();
    }

    if (was_started) {
        Stop();
    }
    if (was_configured) {
        Cleanup();
    }

    if (rebuildWithMissionSpecification(
        replacement,
        was_configured,
        was_started,
        configurator,
        get_reference_cb_group,
        message
    )) {
        message = "mission specification override applied: " +
            replacement->mission_specification_file();
        return true;
    }

    const std::string replacement_failure = message;
    std::string rollback_message;
    if (!rebuildWithMissionSpecification(
        previous_specification,
        was_configured,
        was_started,
        configurator,
        get_reference_cb_group,
        rollback_message
    )) {
        message = "mission specification override failed and rollback failed. New spec failure: " +
            replacement_failure + "; rollback failure: " + rollback_message;
        return false;
    }

    message = "mission specification override failed and previous specification was restored. New spec failure: " +
        replacement_failure;
    return false;

}

bool MissionExecutor::rebuildWithMissionSpecification(
    MissionSpecification::SharedPtr mission_specification,
    bool configure_after_rebuild,
    bool start_after_rebuild,
    iii_drone::configuration::Configurator<rclcpp_lifecycle::LifecycleNode>::SharedPtr configurator,
    rclcpp::CallbackGroup::SharedPtr get_reference_cb_group,
    std::string & message
) {

    if (mission_specification == nullptr) {
        message = "cannot rebuild mission executor with null mission specification";
        return false;
    }

    try {
        if (is_started_) {
            Stop();
        }
        if (is_configured_) {
            Cleanup();
        }

        mission_specification_ = mission_specification;
        tree_provider_->SetMissionSpecification(mission_specification_);
        if (runtime_intent_buffer_ != nullptr) {
            runtime_intent_buffer_->Clear();
        }

        if (configure_after_rebuild) {
            Configure(configurator, get_reference_cb_group);
        }
        if (start_after_rebuild) {
            Start(configurator);
        }
    } catch (const std::exception & exception) {
        message = exception.what();
        return false;
    } catch (...) {
        message = "unknown mission executor rebuild failure";
        return false;
    }

    message = "mission executor rebuilt with " + mission_specification_->mission_specification_file();
    return true;

}

void MissionExecutor::registerIntentServices() {

    unregisterIntentServices();

    for (const auto & intent_service : mission_specification_->intent_services()) {
        RCLCPP_INFO(
            node_->get_logger(),
            "MissionExecutor::registerIntentServices(): Registering runtime intent service %s for flag %s",
            intent_service.service_name.c_str(),
            intent_service.flag_name.c_str()
        );

        intent_services_[intent_service.service_name] = node_->create_service<std_srvs::srv::SetBool>(
            intent_service.service_name,
            [this, intent_service](
                const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                std::shared_ptr<std_srvs::srv::SetBool::Response> response
            ) {
                if (!intentServiceValidForCurrentMode(intent_service)) {
                    response->success = false;
                    response->message = "runtime intent service is not valid for active mode '" + activeModeKey() + "'";
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "MissionExecutor::runtimeIntentServiceCallback(): Rejected %s: %s",
                        intent_service.service_name.c_str(),
                        response->message.c_str()
                    );
                    return;
                }

                const int64_t now_nanoseconds = node_->get_clock()->now().nanoseconds();
                builtin_interfaces::msg::Time stamp;
                stamp.sec = static_cast<int32_t>(now_nanoseconds / 1000000000LL);
                stamp.nanosec = static_cast<uint32_t>(now_nanoseconds % 1000000000LL);

                const uint64_t sequence_id = runtime_intent_buffer_->Enqueue(
                    intent_service.flag_name,
                    request->data,
                    stamp
                );
                response->success = true;
                response->message = "runtime intent enqueued seq=" + std::to_string(sequence_id);
                RCLCPP_INFO(
                    node_->get_logger(),
                    "MissionExecutor::runtimeIntentServiceCallback(): Enqueued runtime intent seq=%llu flag=%s value=%s via %s",
                    static_cast<unsigned long long>(sequence_id),
                    intent_service.flag_name.c_str(),
                    request->data ? "true" : "false",
                    intent_service.service_name.c_str()
                );
            }
        );
    }
}

void MissionExecutor::unregisterIntentServices() {

    intent_services_.clear();

}

bool MissionExecutor::intentServiceValidForCurrentMode(const mission_intent_service_t & intent_service) const {

    if (intent_service.valid_modes.empty()) {
        return true;
    }

    const std::string active_mode_key = activeModeKey();
    return std::find(
        intent_service.valid_modes.begin(),
        intent_service.valid_modes.end(),
        active_mode_key
    ) != intent_service.valid_modes.end();

}

std::string MissionExecutor::activeModeKey() const {

    if (generic_mode_executor_ == nullptr || !generic_mode_executor_->active()) {
        return "";
    }
    return generic_mode_executor_->current_mode_key();

}
