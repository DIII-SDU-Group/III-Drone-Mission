/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/behavior/condition_nodes/select_target_line_condition_node.hpp>

using namespace iii_drone::behavior;
using namespace iii_drone::adapters;
using namespace iii_drone::types;

using namespace BT;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

SelectTargetLineConditionNode::SelectTargetLineConditionNode(
    const std::string & name, 
    const NodeConfig & conf,
    const RosNodeParams & params,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    iii_drone::configuration::ParameterBundle::SharedPtr parameter_bundle
) : RosTopicSubNode<iii_drone_interfaces::msg::Powerline>(
        name, 
        conf, 
        params
),  node_(params.nh),
    tf_buffer_(tf_buffer),
    parameter_bundle_(parameter_bundle) { }

PortsList SelectTargetLineConditionNode::providedPorts() {

    return providedBasicPorts({
        OutputPort<int>("target_cable_id")
    });

}

NodeStatus SelectTargetLineConditionNode::onTick(const std::shared_ptr<iii_drone_interfaces::msg::Powerline> & last_msg) {

    if (!last_msg) {
        RCLCPP_DEBUG(node_->get_logger(), "SelectTargetLineConditionNode::onTick(): No powerline message received.");
        return NodeStatus::FAILURE;
    }

    if (last_msg->lines.size() == 0) {
        RCLCPP_DEBUG(node_->get_logger(), "SelectTargetLineConditionNode::onTick(): No lines in powerline message.");
        return NodeStatus::FAILURE;
    }

    std::vector<SingleLineAdapter> lines_above;

    for (auto & line : last_msg->lines) {

        point_t line_point_drone_frame;

        if (line.header.frame_id != parameter_bundle_->GetParameter("drone_frame_id").as_string()) {

            geometry_msgs::msg::PoseStamped line_pose;
            line_pose.header = line.header;
            line_pose.pose = line.pose;

            try {

                geometry_msgs::msg::PoseStamped line_pose_drone_frame = tf_buffer_->transform(line_pose, parameter_bundle_->GetParameter("drone_frame_id").as_string());

                line_point_drone_frame = point_t(
                    line_pose_drone_frame.pose.position.x, 
                    line_pose_drone_frame.pose.position.y, 
                    line_pose_drone_frame.pose.position.z
                );

            } catch (tf2::TransformException & ex) {

                RCLCPP_ERROR(node_->get_logger(), "SelectTargetLineConditionNode::onTick(): Transform error: %s", ex.what());
                return NodeStatus::FAILURE;

            }
            
        } else {

            line_point_drone_frame = point_t(
                line.pose.position.x, 
                line.pose.position.y, 
                line.pose.position.z
            );

        }
        
        if (line_point_drone_frame(2) > parameter_bundle_->GetParameter("line_min_height_above_drone").as_double()) {

            lines_above.push_back(SingleLineAdapter(line));

        }

    }

    if (lines_above.size() == 0) {

        RCLCPP_DEBUG(node_->get_logger(), "SelectTargetLineConditionNode::onTick(): No lines above drone.");

        return NodeStatus::FAILURE;

    }

    select_target_line_method_t select_target_line_method = getSelectTargetLineMethod();

    int target_cable_id = -1;

    switch(select_target_line_method) {

        default:
        case select_target_line_method_t::SELECT_TARGET_LINE_METHOD_RANDOM_ABOVE: {

            int random_index = rand() % lines_above.size();

            SingleLineAdapter selected_line = lines_above[random_index];

            target_cable_id = selected_line.id();

            break;

        }

        case select_target_line_method_t::SELECT_TARGET_LINE_METHOD_CLOSEST_ABOVE: {

            SingleLineAdapter closest_line = lines_above[0];
            double closest_distance = std::numeric_limits<double>::max();

            for (auto & line : lines_above) {

                double distance = line.position().norm();

                if (distance < closest_distance) {

                    closest_line = line;
                    closest_distance = distance;

                }

            }

            target_cable_id = closest_line.id();

            break;

        }

        case select_target_line_method_t::SELECT_TARGET_LINE_METHOD_CLOSEST_XY_ABOVE: {

            SingleLineAdapter closest_line = lines_above[0];
            double closest_distance = std::numeric_limits<double>::max();

            for (auto & line : lines_above) {

                double distance = line.position().head(2).norm();

                if (distance < closest_distance) {

                    closest_line = line;
                    closest_distance = distance;

                }

            }

            target_cable_id = closest_line.id();

            break;

        }

        case select_target_line_method_t::SELECT_TARGET_LINE_METHOD_CLOSEST_Z_ABOVE: {

            SingleLineAdapter closest_line = lines_above[0];
            double closest_distance = std::numeric_limits<double>::max();

            for (auto & line : lines_above) {

                double distance = std::abs(line.position()(2));

                if (distance < closest_distance) {

                    closest_line = line;
                    closest_distance = distance;

                }

            }

            target_cable_id = closest_line.id();

            break;

        }

        case select_target_line_method_t::SELECT_TARGET_LINE_FARTHEST_ABOVE: {

            SingleLineAdapter farthest_line = lines_above[0];
            double farthest_distance = 0.0;

            for (auto & line : lines_above) {

                double distance = line.position().norm();

                if (distance > farthest_distance) {

                    farthest_line = line;
                    farthest_distance = distance;

                }

            }

            target_cable_id = farthest_line.id();

            break;

        }
    }

    setOutput("target_cable_id", target_cable_id);

    RCLCPP_DEBUG(
        node_->get_logger(), 
        "SelectTargetLineConditionNode::onTick(): Selected target line ID: %d", 
        target_cable_id
    );

    return NodeStatus::SUCCESS;

}

SelectTargetLineConditionNode::select_target_line_method_t SelectTargetLineConditionNode::getSelectTargetLineMethod() {

    std::string select_target_line_method_str = parameter_bundle_->GetParameter("select_target_line_method").as_string();

    if (select_target_line_method_str == "random_above") {

        return select_target_line_method_t::SELECT_TARGET_LINE_METHOD_RANDOM_ABOVE;

    } else if (select_target_line_method_str == "closest_above") {

        return select_target_line_method_t::SELECT_TARGET_LINE_METHOD_CLOSEST_ABOVE;

    } else if (select_target_line_method_str == "closest_xy_above") {

        return select_target_line_method_t::SELECT_TARGET_LINE_METHOD_CLOSEST_XY_ABOVE;

    } else if (select_target_line_method_str == "closest_z_above") {

        return select_target_line_method_t::SELECT_TARGET_LINE_METHOD_CLOSEST_Z_ABOVE;

    } else if (select_target_line_method_str == "farthest_above") {

        return select_target_line_method_t::SELECT_TARGET_LINE_FARTHEST_ABOVE;

    } else {

        RCLCPP_ERROR(node_->get_logger(), "SelectTargetLineConditionNode::getSelectTargetLineMethod(): Invalid select_target_line_method parameter: %s", select_target_line_method_str.c_str());
        return select_target_line_method_t::SELECT_TARGET_LINE_METHOD_RANDOM_ABOVE;

    }

}