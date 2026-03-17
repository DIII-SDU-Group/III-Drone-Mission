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

        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onResultReceived(): %s: Maneuver action succeeded",
            name_.c_str()
        );

        if (get_final_reference_callback_) {

            Reference ref = get_final_reference_callback_(wr);

            ref = ref.CopyWithNans();

            setManeuverNotRunning(
                ref,
                stop_maneuver_after_timeout_ms
            );

        } else {

            setManeuverNotRunning(stop_maneuver_after_timeout_ms);

        }

        return NodeStatus::SUCCESS;

    } else {

        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "ManeuverActionNode::onResultReceived(): %s: Maneuver action failed",
            name_.c_str()
        );

        setManeuverNotRunning();
    
        return NodeStatus::FAILURE;
    }

}

template <typename ActionT>
BT::NodeStatus ManeuverActionNode<ActionT>::onFailure(BT::ActionNodeErrorCode error) {

    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "ManeuverActionNode::onFailure(): %s: Maneuver action failed, setting maneuver not running",
        name_.c_str()
    );

    setManeuverNotRunning();

    std::string error_msg;
    
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
            error_msg = "ManeuverActionNode::onFailure(): " + name_ + ": Maneuver invalid goal";
            RCLCPP_FATAL(
                node_ptr_->get_logger(),
                error_msg.c_str()
            );
            throw std::runtime_error(error_msg);
        case ActionNodeErrorCode::SEND_GOAL_TIMEOUT:
            error_msg = "ManeuverActionNode::onFailure(): " + name_ + ": Maneuver send goal timeout";
            RCLCPP_FATAL(
                node_ptr_->get_logger(),
                error_msg.c_str()
            );
            throw std::runtime_error(error_msg);
        case ActionNodeErrorCode::SERVER_UNREACHABLE:
            error_msg = "ManeuverActionNode::onFailure(): " + name_ + ": Maneuver server unreachable";
            RCLCPP_FATAL(
                node_ptr_->get_logger(),
                error_msg.c_str()
            );
            throw std::runtime_error(error_msg);
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

    setManeuverNotRunning();

}

template <typename ActionT>
BT::PortsList ManeuverActionNode<ActionT>::providedManeuverActionNodePorts(BT::PortsList additional_ports) {

    BT::PortsList ports =ManeuverActionNode<ActionT>::providedBasicPorts({
        InputPort<int>("stop_maneuver_after_timeout_ms", -1, "Stop maneuver after timeout in milliseconds, -1 for immediate stop")
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

        if (!maneuver_reference_client_->StartManeuver()) {
            RCLCPP_ERROR(
                node_ptr_->get_logger(),
                "ManeuverActionNode::setManeuverRunning(): %s: Failed to start maneuver",
                name_.c_str()
            );
            return false;
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
void ManeuverActionNode<ActionT>::setGetFinalReferenceCallback(std::function<Reference(const typename RosActionNode<ActionT>::WrappedResult &)> callback) {
    get_final_reference_callback_ = callback;
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
