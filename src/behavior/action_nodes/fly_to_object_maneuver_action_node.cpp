/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/action_nodes/fly_to_object_maneuver_action_node.hpp>

using namespace iii_drone::configuration;
using namespace iii_drone::types;
using namespace iii_drone::math;
using namespace iii_drone::behavior;
using namespace iii_drone::control;
using namespace iii_drone::control::maneuver;
using namespace iii_drone::adapters;
using namespace BT;

/*****************************************************************************/
// Implementation:
/*****************************************************************************/

FlyToObjectManeuverActionNode::FlyToObjectManeuverActionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    ManeuverReferenceClient::SharedPtr maneuver_reference_client,
    ParameterBundle::SharedPtr parameter_bundle,
    tf2_ros::Buffer::SharedPtr tf_buffer
) : ManeuverActionNode<iii_drone_interfaces::action::FlyToObject>(
        name, 
        conf, 
        params,
        maneuver_reference_client
),  parameter_bundle_(parameter_bundle),
    tf_buffer_(tf_buffer) {

    setGetFinalReferenceCallback(
        std::bind(
            &FlyToObjectManeuverActionNode::getFinalReference, 
            this, 
            std::placeholders::_1
        )
    );

}

PortsList FlyToObjectManeuverActionNode::providedPorts() {

    return providedManeuverActionNodePorts({
        InputPort<int>("target_cable_id")
    });

}

bool FlyToObjectManeuverActionNode::setGoal(Goal & goal) {
    
    getInput("target_cable_id", goal.target.target_id);

    if (goal.target.target_id < 0) {
        RCLCPP_DEBUG(
            node_ptr_->get_logger(),
            "FlyToObjectManeuverActionNode::setGoal(): %s: Invalid target ID",
            name_.c_str()
        );
        return false;
    }

    goal.target.target_type = iii_drone_interfaces::msg::Target::TARGET_TYPE_CABLE;
    goal.target.reference_frame_id = parameter_bundle_->GetParameter("drone_frame_id").as_string();

    quaternion_t gripper_q_cable(1, 0, 0, 0);
    geometry_msgs::msg::QuaternionStamped gripper_q_cable_msg;
    gripper_q_cable_msg.header.frame_id = parameter_bundle_->GetParameter("gripper_frame_id").as_string();
    gripper_q_cable_msg.quaternion = quaternionMsgFromQuaternion(gripper_q_cable);

    geometry_msgs::msg::QuaternionStamped drone_q_cable_msg;

    try {

        drone_q_cable_msg = tf_buffer_->transform(
            gripper_q_cable_msg,
            parameter_bundle_->GetParameter("drone_frame_id").as_string()
        );

    } catch (tf2::TransformException & e) {

        RCLCPP_ERROR(
            node_ptr_->get_logger(),
            "FlyToObjectManeuverActionNode::setGoal(): %s: Failed to transform gripper quaternion to drone frame: %s",
            name_.c_str(),
            e.what()
        );

        return false;

    }

    quaternion_t drone_q_cable = quaternionFromQuaternionMsg(drone_q_cable_msg.quaternion);

    vector_t drone_v_cable(0, 0, parameter_bundle_->GetParameter("target_cable_distance").as_double());

    goal.target.target_transform = transformMsgFromTransform(
        drone_v_cable,
        drone_q_cable
    );

    return true;

}

Reference FlyToObjectManeuverActionNode::getFinalReference(const typename BT::RosActionNode<iii_drone_interfaces::action::FlyToObject>::WrappedResult & wr) const {

    return ReferenceAdapter(wr.result->target_reference).reference();

}