#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <iii_drone_core/utils/types.hpp>
#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/pylon_overview.hpp>

namespace iii_drone {
namespace mission {
namespace overview_gnss {

struct GnssPoint {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};

struct WorldGnssReference {
    GnssPoint drone_gnss;
    iii_drone::types::point_t drone_world = iii_drone::types::point_t::Zero();
};

std::filesystem::path defaultOverviewDirectory();

bool validGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition & position);

std::optional<WorldGnssReference> makeReference(
    const px4_msgs::msg::VehicleGlobalPosition & global_position,
    const tf2_ros::Buffer::SharedPtr & tf_buffer,
    const rclcpp::Logger & logger
);

GnssPoint worldToGnss(
    const iii_drone::types::point_t & world_point,
    const WorldGnssReference & reference
);

iii_drone::types::point_t gnssToWorld(
    const GnssPoint & gnss_point,
    const WorldGnssReference & reference
);

bool persistPowerlineOverview(
    const iii_drone_interfaces::msg::Powerline & powerline,
    const WorldGnssReference & reference,
    const std::filesystem::path & path,
    const rclcpp::Logger & logger
);

std::optional<iii_drone_interfaces::msg::Powerline> loadPowerlineOverview(
    const std::filesystem::path & path,
    const WorldGnssReference & reference,
    const rclcpp::Logger & logger
);

bool persistPylonOverview(
    const iii_drone_interfaces::msg::PylonOverview & overview,
    const WorldGnssReference & reference,
    const std::filesystem::path & path,
    const rclcpp::Logger & logger
);

std::optional<iii_drone_interfaces::msg::PylonOverview> loadPylonOverview(
    const std::filesystem::path & path,
    const WorldGnssReference & reference,
    const rclcpp::Logger & logger
);

} // namespace overview_gnss
} // namespace mission
} // namespace iii_drone
