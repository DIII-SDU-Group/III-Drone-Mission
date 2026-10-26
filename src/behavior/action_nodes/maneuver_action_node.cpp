/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp>

#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/action/follow_waypoint_path.hpp>
#include <iii_drone_interfaces/action/fly_to_object.hpp>
#include <iii_drone_interfaces/action/cable_landing.hpp>
#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/action/hover.hpp>
#include <iii_drone_interfaces/action/hover_by_object.hpp>
#include <iii_drone_interfaces/action/hover_on_cable.hpp>
#include <iii_drone_core/diagnostics/hil_trace.hpp>
#include <iii_drone_mission/mission/mission_exit.hpp>

#include <atomic>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using namespace iii_drone::behavior;
using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;

using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

namespace {

std::string actionResultCodeToString(rclcpp_action::ResultCode code) {
    switch (code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            return "SUCCEEDED";
        case rclcpp_action::ResultCode::ABORTED:
            return "ABORTED";
        case rclcpp_action::ResultCode::CANCELED:
            return "CANCELED";
        case rclcpp_action::ResultCode::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

std::string actionNodeErrorCodeToString(BT::ActionNodeErrorCode error) {
    switch (error) {
        case BT::ActionNodeErrorCode::ACTION_ABORTED:
            return "ACTION_ABORTED";
        case BT::ActionNodeErrorCode::ACTION_CANCELLED:
            return "ACTION_CANCELLED";
        case BT::ActionNodeErrorCode::GOAL_REJECTED_BY_SERVER:
            return "GOAL_REJECTED_BY_SERVER";
        case BT::ActionNodeErrorCode::INVALID_GOAL:
            return "INVALID_GOAL";
        case BT::ActionNodeErrorCode::SEND_GOAL_TIMEOUT:
            return "SEND_GOAL_TIMEOUT";
        case BT::ActionNodeErrorCode::SERVER_UNREACHABLE:
            return "SERVER_UNREACHABLE";
        default:
            return "UNKNOWN";
    }
}

std::string goalUuidToString(const rclcpp_action::GoalUUID & uuid) {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : uuid) {
        stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
}

template <typename ResultT, typename = void>
struct has_success_field : std::false_type {};

template <typename ResultT>
struct has_success_field<ResultT, std::void_t<decltype(std::declval<ResultT>().success)>>
    : std::true_type {};

template <typename ResultT, typename = void>
struct has_reason_field : std::false_type {};

template <typename ResultT>
struct has_reason_field<ResultT, std::void_t<decltype(std::declval<ResultT>().reason)>>
    : std::true_type {};

template <typename ResultT>
void appendResultFields(
    iii_drone::diagnostics::HilTrace::Event & event,
    const std::shared_ptr<ResultT> & result
) {
    if (!result) {
        return;
    }
    if constexpr (has_success_field<ResultT>::value) {
        event.boolean("result_success", result->success);
    }
    if constexpr (has_reason_field<ResultT>::value) {
        event.text("result_reason", result->reason);
    }
}

}  // namespace

template <typename ActionT>
ManeuverActionNode<ActionT>::ManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client
) : RosActionNode<ActionT>(
        name, 
        conf, 
        params
),  maneuver_reference_client_(maneuver_reference_client),
    name_(name),
    action_endpoint_(params.default_port_value),
    node_ptr_(params.nh.lock()) {
    if (!node_ptr_) {
        throw std::runtime_error("ManeuverActionNode: ROS node handle expired");
    }
}

template <typename ActionT>
BT::NodeStatus ManeuverActionNode<ActionT>::tick() {
    const bool dispatching = this->status() == BT::NodeStatus::IDLE;
    return iii_drone::mission::guardMissionDispatch(
        dispatching,
        [this, dispatching]() {
            if (dispatching) {
                auto event = iii_drone::diagnostics::HilTrace::event("bt_action_goal_attempt");
                event.text("node", name_);
                event.text("endpoint", action_endpoint_);
                event.commit();
            }
            return RosActionNode<ActionT>::tick();
        },
        [this]() {
            // Mission Exit: PX4 no longer runs this mission. Never send a goal.
            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ManeuverActionNode::tick(): %s: Mission Exit, not dispatching maneuver goal",
                name_.c_str()
            );
            auto event = iii_drone::diagnostics::HilTrace::event("bt_action_goal_blocked_mission_exit");
            event.text("node", name_);
            event.text("endpoint", action_endpoint_);
            event.commit();
            ManeuverActionNode<ActionT>::setOutput("terminal_state", std::string("MISSION_EXIT"));
            return BT::NodeStatus::FAILURE;
        }
    );
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::setGoal(typename ActionT::Goal & goal) {
    successful_result_ownership_failed_ = false;
    // Concrete builders remain responsible only for action-specific BT inputs.
    // A false builder result has no pending client authorization to leak.
    if (!setManeuverGoal(goal)) {
        pending_request_identity_.clear();
        goal_handoff_pending_ = false;
        return false;
    }

    pending_request_identity_ = makeRequestIdentity();
    goal.request_identity = pending_request_identity_;
    const bool attach_to_active_stream = shouldAttachToActiveManeuverStreamOnGoalAccepted();
    if (!maneuver_reference_client_->BeginManeuverGoalHandoff(
            pending_request_identity_, attach_to_active_stream)) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setGoal(): %s: Refusing dispatch without request-bound handoff authorization",
            name_.c_str()
        );
        pending_request_identity_.clear();
        goal_handoff_pending_ = false;
        ManeuverActionNode<ActionT>::setOutput(
            "terminal_state", std::string("HANDOFF_PREPARATION_FAILED")
        );
        return false;
    }
    goal_handoff_pending_ = true;
    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_goal_prepared");
    event.text("node", name_);
    event.text("endpoint", action_endpoint_);
    event.text("request_identity", pending_request_identity_);
    event.boolean("attach_to_active_stream", attach_to_active_stream);
    event.commit();
    return true;
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::onGoalAccepted() {

    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_goal_accepted");
    event.text("node", name_);
    event.text("endpoint", action_endpoint_);
    event.commit();

    RCLCPP_INFO(
        node_ptr_->get_logger(),
        "ManeuverActionNode::onGoalAccepted(): %s: Maneuver action goal accepted",
        name_.c_str()
    );

    ManeuverActionNode<ActionT>::setOutput("terminal_state", std::string("ACCEPTED"));

    // // Check if ActionT is CableLanding:
    // if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::CableLanding>::value) {

    //     maneuver_reference_client_->SetReferenceModeHover();
    //     return NodeStatus::RUNNING;

    // }

    if (!setManeuverRunning()) {
        if (iii_drone::mission::missionExitClosedDispatch()) {
            RCLCPP_INFO(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onGoalAccepted(): %s: Goal accepted after Mission Exit released it, halting maneuver",
                name_.c_str()
            );
        } else {
            RCLCPP_ERROR(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onGoalAccepted(): %s: Failed to start maneuver, halting maneuver",
                name_.c_str()
            );
        }

        this->halt();
    }

}

template <typename ActionT>
BT::NodeStatus ManeuverActionNode<ActionT>::onResultReceived(const typename RosActionNode<ActionT>::WrappedResult & wr) {

    int stop_maneuver_after_timeout_ms = -1;
    ManeuverActionNode<ActionT>::getInput("stop_maneuver_after_timeout_ms", stop_maneuver_after_timeout_ms);

    if (stop_maneuver_after_timeout_ms > 0) {

        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onResultReceived(): %s: Stopping maneuver after timeout",
            name_.c_str()
        );
    
    }

    const BT::NodeStatus returned_status =
        wr.code == rclcpp_action::ResultCode::SUCCEEDED ? NodeStatus::SUCCESS : NodeStatus::FAILURE;

    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_result");
    event.text("node", name_);
    event.text("endpoint", action_endpoint_);
    event.text("goal_id", goalUuidToString(wr.goal_id));
    event.text("result_code", actionResultCodeToString(wr.code));
    event.text("bt_status", BT::toStr(returned_status, false));
    appendResultFields(event, wr.result);
    event.commit();

    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {

        ManeuverActionNode<ActionT>::setOutput("terminal_state", actionResultCodeToString(wr.code));

        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onResultReceived(): %s: Maneuver action succeeded",
            name_.c_str()
        );

        const bool should_stop = shouldStopManeuverOnSuccessfulResult(wr);
        if (successful_result_ownership_failed_) {
            ManeuverActionNode<ActionT>::setOutput(
                "terminal_state", std::string("TERMINAL_OWNERSHIP_FAILED"));
            clearLocalGoalBookkeeping();
            return NodeStatus::FAILURE;
        }
        if (!should_stop) {

            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onResultReceived(): %s: Successful result keeps maneuver reference stream active",
                name_.c_str()
            );

            maneuver_running_ = false;
            goal_handoff_pending_ = false;
            pending_request_identity_.clear();

        } else if (get_final_reference_callback_) {

            Reference ref = get_final_reference_callback_(wr);

            ref = ref.CopyWithNans();

            safeSetManeuverNotRunning(
                ref,
                stop_maneuver_after_timeout_ms,
                "successful result final-reference cleanup"
            );

        } else {
            if (stop_maneuver_after_timeout_ms <= 0 &&
                shouldCompleteSuccessfulNoReferenceGoal()) {
                // The concrete node opted into exact, no-reference completion.
                // An applied moving object stream stays owned until Core
                // certifies the next goal's finite rest transition.
                (void)completeSuccessfulOwnedNoReferenceGoal();
            } else {
                safeSetManeuverNotRunning(
                    stop_maneuver_after_timeout_ms,
                    "successful result cleanup"
                );
            }

        }

        if (successful_result_ownership_failed_) {
            ManeuverActionNode<ActionT>::setOutput(
                "terminal_state", std::string("TERMINAL_OWNERSHIP_FAILED"));
            return NodeStatus::FAILURE;
        }

        return NodeStatus::SUCCESS;

    } else {

        ManeuverActionNode<ActionT>::setOutput("terminal_state", actionResultCodeToString(wr.code));

        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onResultReceived(): %s: Maneuver action failed",
            name_.c_str()
        );

        safeSetManeuverNotRunning("failed result cleanup");
    
        return NodeStatus::FAILURE;
    }

}

template <typename ActionT>
BT::NodeStatus ManeuverActionNode<ActionT>::onFailure(
    BT::ActionNodeErrorCode error,
    const std::optional<typename RosActionNode<ActionT>::WrappedResult> & result
) {
    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_failure");
    event.text("node", name_);
    event.text("endpoint", action_endpoint_);
    event.text("error", actionNodeErrorCodeToString(error));
    event.text("bt_status", "FAILURE");
    if (result.has_value()) {
        event.text("goal_id", goalUuidToString(result->goal_id));
        event.text("result_code", actionResultCodeToString(result->code));
        appendResultFields(event, result->result);
    }
    event.commit();
    return onFailure(error);
}

template <typename ActionT>
BT::NodeStatus ManeuverActionNode<ActionT>::onFailure(BT::ActionNodeErrorCode error) {

    ManeuverActionNode<ActionT>::setOutput("terminal_state", actionNodeErrorCodeToString(error));

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::onFailure(): %s: Maneuver action failed, setting maneuver not running",
        name_.c_str()
    );

    safeSetManeuverNotRunning("action failure cleanup");

    const bool mission_exit = iii_drone::mission::missionExitClosedDispatch();
    if (mission_exit && (
            error == ActionNodeErrorCode::ACTION_ABORTED ||
            error == ActionNodeErrorCode::ACTION_CANCELLED ||
            error == ActionNodeErrorCode::GOAL_REJECTED_BY_SERVER)) {
        // The echo of a Mission Exit: PX4 no longer runs this mission and
        // Core ended the goal on purpose. Server faults stay loud below.
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onFailure(): %s: Maneuver ended by Mission Exit (%s)",
            name_.c_str(),
            actionNodeErrorCodeToString(error).c_str()
        );
        return NodeStatus::FAILURE;
    }

    switch(error) {
        case ActionNodeErrorCode::ACTION_ABORTED:
            RCLCPP_WARN(
                node_ptr_->get_logger(), 
                "ManeuverActionNode::onFailure(): %s: Maneuver action aborted",
                name_.c_str()
            );
            break;
        case ActionNodeErrorCode::ACTION_CANCELLED:
            RCLCPP_WARN(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onFailure(): %s: Maneuver action cancelled",
                name_.c_str()
            );
            break;
        case ActionNodeErrorCode::GOAL_REJECTED_BY_SERVER:
            RCLCPP_WARN(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onFailure(): %s: Maneuver goal rejected by server",
                name_.c_str()
            );
            break;
        case ActionNodeErrorCode::INVALID_GOAL:
            RCLCPP_ERROR(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onFailure(): %s: Maneuver invalid goal",
                name_.c_str()
            );
            break;
        case ActionNodeErrorCode::SEND_GOAL_TIMEOUT:
            RCLCPP_ERROR(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onFailure(): %s: Maneuver send goal timeout",
                name_.c_str()
            );
            break;
        case ActionNodeErrorCode::SERVER_UNREACHABLE:
            RCLCPP_ERROR(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onFailure(): %s: Maneuver server unreachable",
                name_.c_str()
            );
            break;
    }
    
    return NodeStatus::FAILURE;

}

// template <typename ActionT>
// BT::NodeStatus ManeuverActionNode<ActionT>::onFeedback(const typename std::shared_ptr<const typename RosActionNode<ActionT>::Feedback>) {

//     // Check if ActionT is CableLanding:
//     if constexpr (std::is_same<ActionT, iii_drone_interfaces::action::CableLanding>::value) {

//         maneuver_reference_client_->SetReferenceModeHover();
//         return NodeStatus::RUNNING;

//     }

//     setManeuverRunning();

//     return NodeStatus::RUNNING;

// }

template <typename ActionT>
void ManeuverActionNode<ActionT>::onHalt() {

    auto event = iii_drone::diagnostics::HilTrace::event("bt_action_halt");
    event.text("node", name_);
    event.text("endpoint", action_endpoint_);
    event.commit();

    if (iii_drone::mission::missionExitClosedDispatch()) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onHalt(): %s: Halting maneuver for Mission Exit",
            name_.c_str()
        );
    } else {
        // A halt is always a deliberate tree decision (e.g. the recharge
        // ReactiveFallback preempting an inspection maneuver); failures are
        // reported by their own paths.
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onHalt(): %s: Halting maneuver",
            name_.c_str()
        );
    }

    safeSetManeuverNotRunning("halt cleanup");

}

template <typename ActionT>
void ManeuverActionNode<ActionT>::markSuccessfulResultOwnershipFailed() const {
    successful_result_ownership_failed_ = true;
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::clearLocalGoalBookkeeping() {
    maneuver_running_ = false;
    goal_handoff_pending_ = false;
    pending_request_identity_.clear();
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::reportTerminalRetentionFailureForCurrentGoal() {
    return !pending_request_identity_.empty() &&
        maneuver_reference_client_->ReportTerminalRetentionFailure(pending_request_identity_);
}

template <typename ActionT>
BT::PortsList ManeuverActionNode<ActionT>::providedManeuverActionNodePorts(BT::PortsList additional_ports) {

    BT::PortsList ports =ManeuverActionNode<ActionT>::providedBasicPorts({
        InputPort<int>("stop_maneuver_after_timeout_ms", -1, "Stop maneuver after timeout in milliseconds, -1 for immediate stop"),
        OutputPort<std::string>("terminal_state", "Final ROS action result or action-node error state")
    });

    ports.insert(additional_ports.begin(), additional_ports.end());

    return ports;

}

template <typename ActionT>
std::string ManeuverActionNode<ActionT>::makeRequestIdentity() const {
    return nextProcessManeuverRequestIdentity();
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::setManeuverRunning() {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::setManeuverRunning(): %s",
        name_.c_str()
    );

    if (!maneuver_running_) {

        if (goal_handoff_pending_) {
            if (!maneuver_reference_client_->ConfirmManeuverGoalHandoff(pending_request_identity_)) {
                if (iii_drone::mission::missionExitClosedDispatch()) {
                    RCLCPP_INFO(
                        node_ptr_->get_logger(),
                        "ManeuverActionNode::setManeuverRunning(): %s: Pending goal handoff was released by Mission Exit",
                        name_.c_str()
                    );
                } else {
                    RCLCPP_ERROR(
                        node_ptr_->get_logger(),
                        "ManeuverActionNode::setManeuverRunning(): %s: Pending goal handoff was lost",
                        name_.c_str()
                    );
                }
                return false;
            }
            goal_handoff_pending_ = false;
            maneuver_running_ = true;
            return true;
        }

        // Every successful setGoal has already registered an identity. A late
        // acceptance cannot use the legacy global Start/Stop fallback.
        return false;

    } else {

        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setManeuverRunning(): %s: Maneuver already running, returning",
            name_.c_str()
        );

        return false;

    }

}

template <typename ActionT>
void ManeuverActionNode<ActionT>::setManeuverNotRunning(int stop_maneuver_after_timeout_ms) {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::setManeuverNotRunning(stop_maneuver_after_timeout_ms): %s",
        name_.c_str()
    );

    if (!pending_request_identity_.empty()) {
        if (stop_maneuver_after_timeout_ms > 0) {
            maneuver_reference_client_->StopManeuverGoalHandoffAfterTimeout(
                pending_request_identity_, stop_maneuver_after_timeout_ms
            );
        } else {
            maneuver_reference_client_->CancelManeuverGoalHandoff(pending_request_identity_);
        }
    }
    maneuver_running_ = false;
    goal_handoff_pending_ = false;
    pending_request_identity_.clear();

}

template <typename ActionT>
void ManeuverActionNode<ActionT>::setManeuverNotRunning(
    const Reference & reference,
    int stop_maneuver_after_timeout_ms
) {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::setManeuverNotRunning(reference, stop_maneuver_after_timeout_ms): %s",
        name_.c_str()
    );

    if (!pending_request_identity_.empty()) {
        if (stop_maneuver_after_timeout_ms > 0) {
            maneuver_reference_client_->StopManeuverGoalHandoffAfterTimeout(
                pending_request_identity_, reference, stop_maneuver_after_timeout_ms
            );
        } else {
            if (!maneuver_reference_client_->CompleteManeuverGoalHandoff(
                    pending_request_identity_, reference)) {
                RCLCPP_ERROR(node_ptr_->get_logger(),
                    "ManeuverActionNode::setManeuverNotRunning(reference): %s: "
                    "Core command ownership was not completed for request %s",
                    name_.c_str(), pending_request_identity_.c_str());
                markSuccessfulResultOwnershipFailed();
                return;
            }
        }
    }
    maneuver_running_ = false;
    goal_handoff_pending_ = false;
    pending_request_identity_.clear();

}

template <typename ActionT>
void ManeuverActionNode<ActionT>::safeSetManeuverNotRunning(const char * context) {
    safeSetManeuverNotRunning(-1, context);
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::safeSetManeuverNotRunning(
    int stop_maneuver_after_timeout_ms,
    const char * context
) {
    try {
        setManeuverNotRunning(stop_maneuver_after_timeout_ms);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ManeuverActionNode::safeSetManeuverNotRunning(): %s: Cleanup failed during %s: %s",
            name_.c_str(),
            context,
            e.what()
        );
        maneuver_running_ = false;
        goal_handoff_pending_ = false;
        pending_request_identity_.clear();
    }
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::safeSetManeuverNotRunning(
    const Reference & reference,
    int stop_maneuver_after_timeout_ms,
    const char * context
) {
    try {
        setManeuverNotRunning(reference, stop_maneuver_after_timeout_ms);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ManeuverActionNode::safeSetManeuverNotRunning(reference): %s: Cleanup failed during %s: %s",
            name_.c_str(),
            context,
            e.what()
        );
        maneuver_running_ = false;
        goal_handoff_pending_ = false;
        pending_request_identity_.clear();
    }
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::setGetFinalReferenceCallback(std::function<Reference(const typename RosActionNode<ActionT>::WrappedResult &)> callback) {
    get_final_reference_callback_ = callback;
}

template <typename ActionT>
ManeuverReferenceClient::TerminalHoldRetention
ManeuverActionNode<ActionT>::retainCompletedTerminalHoldForCurrentGoal(
    int timeout_ms
) const {
    if (pending_request_identity_.empty())
        return ManeuverReferenceClient::TerminalHoldRetention::Failed;
    return maneuver_reference_client_->RetainCompletedTerminalHold(
        pending_request_identity_, timeout_ms);
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::shouldStopManeuverOnSuccessfulResult(
    const typename RosActionNode<ActionT>::WrappedResult &
) const {
    return true;
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::shouldCompleteSuccessfulNoReferenceGoal() const {
    return false;
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::completeSuccessfulOwnedNoReferenceGoal() {
    if (pending_request_identity_.empty()) {
        markSuccessfulResultOwnershipFailed();
        return false;
    }
    bool completed = false;
    try {
        completed = maneuver_reference_client_->CompleteManeuverGoalHandoff(
            pending_request_identity_);
    } catch (const std::exception & error) {
        RCLCPP_ERROR(node_ptr_->get_logger(),
            "ManeuverActionNode::completeSuccessfulOwnedNoReferenceGoal(): %s: %s",
            name_.c_str(), error.what());
    }
    if (!completed) {
        RCLCPP_ERROR(node_ptr_->get_logger(),
            "ManeuverActionNode::completeSuccessfulOwnedNoReferenceGoal(): %s: "
            "Request %s did not complete command ownership",
            name_.c_str(), pending_request_identity_.c_str());
        markSuccessfulResultOwnershipFailed();
        return false;
    }
    clearLocalGoalBookkeeping();
    return true;
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::shouldAttachToActiveManeuverStreamOnGoalAccepted() const {
    return false;
}

/*****************************************************************************/
// Explicit template instantiation:
/*****************************************************************************/

template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::FlyToPosition>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::FollowWaypointPath>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::FlyToObject>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::CableLanding>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::CableTakeoff>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::Hover>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::HoverByObject>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::HoverOnCable>;
