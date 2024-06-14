#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    class MissionExecutor : public rclcpp::Node {
    public:

        void RegisterNodes(rclcpp::executors::MultiThreadedExecutor &executor);

    };

} // namespace mission
} // namespace iii_drone
