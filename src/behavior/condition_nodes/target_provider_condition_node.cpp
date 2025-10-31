/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/condition_nodes/target_provider_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::configuration;
using namespace iii_drone::types;
using namespace BT;


/*****************************************************************************/
// Implementation
/*****************************************************************************/

TargetProvider::TargetProvider(
    const std::string & name,
    const NodeConfig & config,
    rclcpp::Node * node,
    tf2_ros::Buffer::SharedPtr tf_buffer,
    iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle
) : ConditionNode(
    name,
    config
),  node_(node),
    tf_buffer_(tf_buffer),
    parameter_bundle_(parameter_bundle) { }

PortsList TargetProvider::providedPorts() {

    return {
        InputPort<target_provider_mode_t>("mode", "Mode of the target provider"),
        InputPort<int>("target_id", "Id of the target"),
        OutputPort<iii_drone_interfaces::msg::Target>("target", "Provided target")
    };

}

NodeStatus TargetProvider::tick() {

    target_provider_mode_t mode;
    getInput("mode", mode);

    switch(mode) {
        case TARGET_PROVIDER_MODE_FLY_TO_CABLE:
            RCLCPP_INFO(node_->get_logger(), "TargetProvider::tick(): Providing fly to object target");
            break;
        case TARGET_PROVIDER_MODE_HOVER_BY_CABLE:
            RCLCPP_INFO(node_->get_logger(), "TargetProvider::tick(): Providing hover by object target");
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "TargetProvider::tick(): Invalid mode %d", mode);
            return NodeStatus::FAILURE;
    }

    int target_id;
    getInput("target_id", target_id);

    if (target_id < 0) {
        RCLCPP_ERROR(node_->get_logger(), "TargetProvider::tick(): Invalid target ID %d", target_id);
        return NodeStatus::FAILURE;
    }

    iii_drone_interfaces::msg::Target target;

    switch (mode) {
    case TARGET_PROVIDER_MODE_FLY_TO_CABLE:
        if (!getFlyToCableTarget(
            target_id,
            target
        )) {
            RCLCPP_ERROR(node_->get_logger(), "TargetProvider::tick(): Failed to get fly to cable target");
            return NodeStatus::FAILURE;
        }
        break;

    case TARGET_PROVIDER_MODE_HOVER_BY_CABLE:
        if (!getHoverByCableTarget(
            target_id,
            target
        )) {
            RCLCPP_ERROR(node_->get_logger(), "TargetProvider::tick(): Failed to get hover by object target");
            return NodeStatus::FAILURE;
        }
        break;

    }

    setOutput("target", target);

    return NodeStatus::SUCCESS;

}

bool TargetProvider::getFlyToCableTarget(
    int target_id,
    iii_drone_interfaces::msg::Target & target
) {

    target.target_id = target_id;
    target.target_type = iii_drone_interfaces::msg::Target::TARGET_TYPE_CABLE;
    target.reference_frame_id = parameter_bundle_->GetParameter("drone_frame_id").as_string();

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
            node_->get_logger(),
            "TargetProvider::getFlyToCableTarget(): Failed to transform gripper quaternion to drone frame: %s",
            e.what()
        );

        return false;

    }

    quaternion_t drone_q_cable = quaternionFromQuaternionMsg(drone_q_cable_msg.quaternion);

    vector_t drone_v_cable(0, 0, parameter_bundle_->GetParameter("target_cable_distance").as_double());

    target.target_transform = transformMsgFromTransform(
        drone_v_cable,
        drone_q_cable
    );

    return true;

}

bool TargetProvider::getHoverByCableTarget(
    int target_id,
    iii_drone_interfaces::msg::Target & target
) {

    return getFlyToCableTarget(
        target_id,
        target
    );

}