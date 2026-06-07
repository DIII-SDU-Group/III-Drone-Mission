/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/action_nodes/maneuver_action_node.hpp>

#include <iii_drone_interfaces/action/fly_to_position.hpp>
#include <iii_drone_interfaces/action/fly_to_object.hpp>
#include <iii_drone_interfaces/action/cable_landing.hpp>
#include <iii_drone_interfaces/action/cable_takeoff.hpp>
#include <iii_drone_interfaces/action/hover.hpp>
#include <iii_drone_interfaces/action/hover_by_object.hpp>
#include <iii_drone_interfaces/action/hover_on_cable.hpp>
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
    node_ptr_(params.nh.lock()) {
    if (!node_ptr_) {
        throw std::runtime_error("ManeuverActionNode: ROS node handle expired");
    }
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::onGoalAccepted() {

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
        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onGoalAccepted(): %s: Failed to start maneuver, halting maneuver",
            name_.c_str()
        );

        this->halt();
    }

}

template <typename ActionT>
BT::NodeStatus ManeuverActionNode<ActionT>::onResultReceived(const typename RosActionNode<ActionT>::WrappedResult & wr) {

    int stop_maneuver_after_timeout_ms;
    ManeuverActionNode<ActionT>::getInput("stop_maneuver_after_timeout_ms", stop_maneuver_after_timeout_ms);

    if (stop_maneuver_after_timeout_ms > 0) {

        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onResultReceived(): %s: Stopping maneuver after timeout",
            name_.c_str()
        );
    
    }

    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {

        ManeuverActionNode<ActionT>::setOutput("terminal_state", actionResultCodeToString(wr.code));

        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onResultReceived(): %s: Maneuver action succeeded",
            name_.c_str()
        );

        if (!shouldStopManeuverOnSuccessfulResult(wr)) {

            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "ManeuverActionNode::onResultReceived(): %s: Successful result keeps maneuver reference stream active",
                name_.c_str()
            );

            maneuver_running_ = false;

        } else if (get_final_reference_callback_) {

            Reference ref = get_final_reference_callback_(wr);

            ref = ref.CopyWithNans();

            safeSetManeuverNotRunning(
                ref,
                stop_maneuver_after_timeout_ms,
                "successful result final-reference cleanup"
            );

        } else {

            safeSetManeuverNotRunning(
                stop_maneuver_after_timeout_ms,
                "successful result cleanup"
            );

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
BT::NodeStatus ManeuverActionNode<ActionT>::onFailure(BT::ActionNodeErrorCode error) {

    ManeuverActionNode<ActionT>::setOutput("terminal_state", actionNodeErrorCodeToString(error));

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::onFailure(): %s: Maneuver action failed, setting maneuver not running",
        name_.c_str()
    );

    safeSetManeuverNotRunning("action failure cleanup");

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

    RCLCPP_WARN(
        node_ptr_->get_logger(),
        "ManeuverActionNode::onHalt(): %s: Halting maneuver",
        name_.c_str()
    );

    safeSetManeuverNotRunning("halt cleanup");

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
bool ManeuverActionNode<ActionT>::setManeuverRunning() {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::setManeuverRunning(): %s",
        name_.c_str()
    );

    if (!maneuver_running_) {

        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setManeuverRunning(): %s: Starting maneuver",
            name_.c_str()
        );

        const bool active_stream = maneuver_reference_client_->IsManeuverActive();
        const bool attach_to_active_stream = shouldAttachToActiveManeuverStreamOnGoalAccepted();

        if (attach_to_active_stream && active_stream) {
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "ManeuverActionNode::setManeuverRunning(): %s: Attaching to active maneuver reference stream",
                name_.c_str()
            );
        } else {
            if (active_stream) {
                RCLCPP_DEBUG(
                    node_ptr_->get_logger(),
                    "ManeuverActionNode::setManeuverRunning(): %s: Replacing active maneuver reference stream before starting non-attached successor",
                    name_.c_str()
                );
                maneuver_reference_client_->StopManeuver();
            }

            if (!maneuver_reference_client_->StartManeuver()) {
                RCLCPP_ERROR(
                    node_ptr_->get_logger(),
                    "ManeuverActionNode::setManeuverRunning(): %s: Failed to start maneuver",
                    name_.c_str()
                );
                return false;
            }
        }

        if (!attach_to_active_stream && active_stream) {
            RCLCPP_DEBUG(
                node_ptr_->get_logger(),
                "ManeuverActionNode::setManeuverRunning(): %s: Non-attached successor started after replacing active stream",
                name_.c_str()
            );
        }

        maneuver_running_ = true;

    } else {

        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setManeuverRunning(): %s: Maneuver already running, returning",
            name_.c_str()
        );

        return false;

    }

    return true;
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::setManeuverNotRunning(int stop_maneuver_after_timeout_ms) {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::setManeuverNotRunning(stop_maneuver_after_timeout_ms): %s",
        name_.c_str()
    );

    if (maneuver_running_) {
        
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setManeuverNotRunning(stop_maneuver_after_timeout_ms): %s: Stopping maneuver",
            name_.c_str()
        );

        if (stop_maneuver_after_timeout_ms > 0) {

            maneuver_reference_client_->StopManeuverAfterTimeout(stop_maneuver_after_timeout_ms);

        } else {

            maneuver_reference_client_->StopManeuver();

        }

        maneuver_running_ = false;

    } else {

        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setManeuverNotRunning(stop_maneuver_after_timeout_ms): %s: Maneuver not running, returning",
            name_.c_str()
        );

    }

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

    if (maneuver_running_) {
        
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setManeuverNotRunning(reference, stop_maneuver_after_timeout_ms): %s: Stopping maneuver",
            name_.c_str()
        );

        if (stop_maneuver_after_timeout_ms > 0) {

            maneuver_reference_client_->StopManeuverAfterTimeout(
                reference, 
                stop_maneuver_after_timeout_ms
            );

        } else {

            maneuver_reference_client_->StopManeuver(reference);

        }

        maneuver_running_ = false;

    } else {

        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "ManeuverActionNode::setManeuverNotRunning(reference, stop_maneuver_after_timeout_ms): %s: Maneuver not running, returning",
            name_.c_str()
        );

    }

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
    }
}

template <typename ActionT>
void ManeuverActionNode<ActionT>::setGetFinalReferenceCallback(std::function<Reference(const typename RosActionNode<ActionT>::WrappedResult &)> callback) {
    get_final_reference_callback_ = callback;
}

template <typename ActionT>
bool ManeuverActionNode<ActionT>::shouldStopManeuverOnSuccessfulResult(
    const typename RosActionNode<ActionT>::WrappedResult &
) const {
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
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::FlyToObject>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::CableLanding>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::CableTakeoff>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::Hover>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::HoverByObject>;
template class iii_drone::behavior::ManeuverActionNode<iii_drone_interfaces::action::HoverOnCable>;
