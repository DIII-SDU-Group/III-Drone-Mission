/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_core/px4/mode_executors/generic_mode_executor.hpp>

using namespace iii_drone::px4;

/*****************************************************************************/
// Implementation
/*****************************************************************************/

GenericModeExecutor::GenericModeExecutor(
    rclcpp::Node & node, 
    px4_ros2::ModeBase & owned_mode,
    std::string mode_executor_name,
    std::function<void()> on_activate_callback,
    std::function<void(DeactivateReason)> on_deactivate_callback
) : ModeExecutorBase(
    node, 
    px4_ros2::ModeExecutorBase::Settings(), 
    owned_mode
),  node_(node),
    mode_executor_name_(mode_executor_name),
    on_activate_callback_(on_activate_callback),
    on_deactivate_callback_(on_deactivate_callback) {

    RCLCPP_DEBUG(node_.get_logger(), "GenericModeExecutor::GenericModeExecutor(): Initializing mode executor %s", mode_executor_name_.c_str());

}
    
void GenericModeExecutor::onActivate() {

    RCLCPP_DEBUG(node_.get_logger(), "GenericModeExecutor::onActivate(): Activating mode executor %s", mode_executor_name_.c_str());

    on_activate_callback_();

}

void GenericModeExecutor::onDeactivate(DeactivateReason reason) {

    RCLCPP_DEBUG(node_.get_logger(), "GenericModeExecutor::onDeactivate(): Deactivating mode executor %s", mode_executor_name_.c_str());

    on_deactivate_callback_(reason);

}