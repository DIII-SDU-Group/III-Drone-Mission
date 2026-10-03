#include <iii_drone_mission/mission/overview_gnss_persistence.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>

#include <iii_drone_core/utils/types.hpp>

namespace iii_drone {
namespace mission {
namespace overview_gnss {

namespace {
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr const char * kPowerlineOverviewFormat = "iii_drone_powerline_overview_gnss_v2";
constexpr const char * kPylonOverviewFormat = "iii_drone_pylon_overview_gnss_v2";

YAML::Node gnssPointToYaml(const GnssPoint & point)
{
    YAML::Node node;
    node["latitude_deg"] = point.latitude_deg;
    node["longitude_deg"] = point.longitude_deg;
    node["altitude_m"] = point.altitude_m;
    return node;
}

GnssPoint gnssPointFromYaml(const YAML::Node & node)
{
    GnssPoint point;
    point.latitude_deg = node["latitude_deg"].as<double>();
    point.longitude_deg = node["longitude_deg"].as<double>();
    point.altitude_m = node["altitude_m"].as<double>();
    return point;
}

YAML::Node quaternionToYaml(const geometry_msgs::msg::Quaternion & quaternion)
{
    YAML::Node node;
    node["w"] = quaternion.w;
    node["x"] = quaternion.x;
    node["y"] = quaternion.y;
    node["z"] = quaternion.z;
    return node;
}

geometry_msgs::msg::Quaternion quaternionFromYaml(const YAML::Node & node)
{
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = node["w"].as<double>();
    quaternion.x = node["x"].as<double>();
    quaternion.y = node["y"].as<double>();
    quaternion.z = node["z"].as<double>();
    return quaternion;
}

YAML::Node vectorToYaml(const geometry_msgs::msg::Vector3 & vector)
{
    YAML::Node node;
    node["east"] = -vector.y;
    node["north"] = vector.x;
    node["up"] = vector.z;
    return node;
}

geometry_msgs::msg::Vector3 vectorFromYaml(const YAML::Node & node)
{
    geometry_msgs::msg::Vector3 vector;
    vector.x = node["north"].as<double>();
    vector.y = -node["east"].as<double>();
    vector.z = node["up"].as<double>();
    return vector;
}

std::filesystem::path configBaseDirectory()
{
    if (const char * config_base = std::getenv("CONFIG_BASE_DIR"); config_base != nullptr && std::string(config_base) != "") {
        return std::filesystem::path(config_base);
    }
    if (const char * home = std::getenv("HOME"); home != nullptr && std::string(home) != "") {
        return std::filesystem::path(home) / ".config" / "iii_drone";
    }
    return std::filesystem::path("/tmp/iii_drone/config");
}
}

std::filesystem::path defaultOverviewDirectory()
{
    return configBaseDirectory() / "overviews";
}

bool validGlobalPosition(const px4_msgs::msg::VehicleGlobalPosition & position)
{
    return position.lat_lon_valid
        && position.alt_valid
        && std::isfinite(position.lat)
        && std::isfinite(position.lon)
        && std::isfinite(position.alt)
        && std::abs(position.lat) <= 90.0
        && std::abs(position.lon) <= 180.0;
}

std::optional<WorldGnssReference> makeReference(
    const px4_msgs::msg::VehicleGlobalPosition & global_position,
    const tf2_ros::Buffer::SharedPtr & tf_buffer,
    const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr & clock
)
{
    if (!validGlobalPosition(global_position)) {
        RCLCPP_WARN_THROTTLE(
            logger,
            *clock,
            10000,
            "GNSS overview persistence unavailable: latest PX4 global position is invalid"
        );
        return std::nullopt;
    }
    if (!tf_buffer) {
        RCLCPP_WARN_THROTTLE(
            logger,
            *clock,
            10000,
            "GNSS overview persistence unavailable: TF buffer is not initialized"
        );
        return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer->lookupTransform("world", "drone", tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
            logger,
            *clock,
            10000,
            "GNSS overview persistence unavailable: cannot lookup world->drone transform: %s",
            error.what()
        );
        return std::nullopt;
    }

    WorldGnssReference reference;
    reference.drone_gnss.latitude_deg = global_position.lat;
    reference.drone_gnss.longitude_deg = global_position.lon;
    reference.drone_gnss.altitude_m = global_position.alt;
    reference.drone_world.x() = transform.transform.translation.x;
    reference.drone_world.y() = transform.transform.translation.y;
    reference.drone_world.z() = transform.transform.translation.z;
    return reference;
}

GnssPoint worldToGnss(
    const iii_drone::types::point_t & world_point,
    const WorldGnssReference & reference
)
{
    // The ROS world frame is generated from PX4 local NED as x=north, y=-east, z=up.
    // Persist in GNSS ENU, then restore using the same convention.
    const double north_m = static_cast<double>(world_point.x() - reference.drone_world.x());
    const double east_m = static_cast<double>(-(world_point.y() - reference.drone_world.y()));
    const double up_m = static_cast<double>(world_point.z() - reference.drone_world.z());
    const double latitude_rad = reference.drone_gnss.latitude_deg * kDegToRad;

    GnssPoint point;
    point.latitude_deg = reference.drone_gnss.latitude_deg + (north_m / kEarthRadiusM) * kRadToDeg;
    point.longitude_deg = reference.drone_gnss.longitude_deg + (east_m / (kEarthRadiusM * std::cos(latitude_rad))) * kRadToDeg;
    point.altitude_m = reference.drone_gnss.altitude_m + up_m;
    return point;
}

iii_drone::types::point_t gnssToWorld(
    const GnssPoint & gnss_point,
    const WorldGnssReference & reference
)
{
    const double latitude_rad = reference.drone_gnss.latitude_deg * kDegToRad;
    const double north_m = (gnss_point.latitude_deg - reference.drone_gnss.latitude_deg) * kDegToRad * kEarthRadiusM;
    const double east_m = (gnss_point.longitude_deg - reference.drone_gnss.longitude_deg) * kDegToRad * kEarthRadiusM * std::cos(latitude_rad);
    const double up_m = gnss_point.altitude_m - reference.drone_gnss.altitude_m;

    iii_drone::types::point_t point;
    point.x() = static_cast<float>(reference.drone_world.x() + north_m);
    point.y() = static_cast<float>(reference.drone_world.y() - east_m);
    point.z() = static_cast<float>(reference.drone_world.z() + up_m);
    return point;
}

bool persistPowerlineOverview(
    const iii_drone_interfaces::msg::Powerline & powerline,
    const WorldGnssReference & reference,
    const std::filesystem::path & path,
    const rclcpp::Logger & logger
)
{
    try {
        std::filesystem::create_directories(path.parent_path());
        YAML::Node root;
        root["format"] = kPowerlineOverviewFormat;
        root["stored_frame_id"] = "gnss_wgs84";
        root["reference"]["drone_gnss"] = gnssPointToYaml(reference.drone_gnss);

        iii_drone::types::point_t plane_point;
        plane_point.x() = powerline.projection_plane.point.x;
        plane_point.y() = powerline.projection_plane.point.y;
        plane_point.z() = powerline.projection_plane.point.z;
        root["projection_plane"]["point"] = gnssPointToYaml(worldToGnss(plane_point, reference));
        root["projection_plane"]["normal"] = vectorToYaml(powerline.projection_plane.normal);

        YAML::Node lines(YAML::NodeType::Sequence);
        for (const auto & line : powerline.lines) {
            YAML::Node item;
            item["id"] = line.id;
            item["in_field_of_view"] = line.in_field_of_view;
            item["orientation"] = quaternionToYaml(line.pose.orientation);

            iii_drone::types::point_t position;
            position.x() = line.pose.position.x;
            position.y() = line.pose.position.y;
            position.z() = line.pose.position.z;
            item["position"] = gnssPointToYaml(worldToGnss(position, reference));

            iii_drone::types::point_t projected_position;
            projected_position.x() = line.projected_position.x;
            projected_position.y() = line.projected_position.y;
            projected_position.z() = line.projected_position.z;
            item["projected_position"] = gnssPointToYaml(worldToGnss(projected_position, reference));
            lines.push_back(item);
        }
        root["lines"] = lines;

        std::ofstream stream(path);
        stream << root;
        return true;
    } catch (const std::exception & error) {
        RCLCPP_WARN(logger, "Failed to persist GNSS powerline overview to %s: %s", path.string().c_str(), error.what());
        return false;
    }
}

std::optional<iii_drone_interfaces::msg::Powerline> loadPowerlineOverview(
    const std::filesystem::path & path,
    const WorldGnssReference & reference,
    const rclcpp::Logger & logger
)
{
    if (!std::filesystem::exists(path)) {
        return std::nullopt;
    }
    try {
        YAML::Node root = YAML::LoadFile(path.string());
        if (root["format"].as<std::string>() != kPowerlineOverviewFormat) {
            RCLCPP_WARN(logger, "Ignoring unsupported powerline overview persistence format in %s", path.string().c_str());
            return std::nullopt;
        }

        iii_drone_interfaces::msg::Powerline powerline;
        powerline.stamp = rclcpp::Clock().now();

        const auto plane_point = gnssToWorld(gnssPointFromYaml(root["projection_plane"]["point"]), reference);
        powerline.projection_plane.point = iii_drone::types::pointMsgFromPoint(plane_point);
        powerline.projection_plane.normal = vectorFromYaml(root["projection_plane"]["normal"]);

        for (const auto & item : root["lines"]) {
            iii_drone_interfaces::msg::SingleLine line;
            line.header.stamp = powerline.stamp;
            line.header.frame_id = "world";
            line.id = item["id"].as<int32_t>();
            line.in_field_of_view = item["in_field_of_view"].as<bool>();
            line.pose.position = iii_drone::types::pointMsgFromPoint(gnssToWorld(gnssPointFromYaml(item["position"]), reference));
            line.pose.orientation = quaternionFromYaml(item["orientation"]);
            line.projected_position = iii_drone::types::pointMsgFromPoint(gnssToWorld(gnssPointFromYaml(item["projected_position"]), reference));
            powerline.lines.push_back(line);
        }
        return powerline;
    } catch (const std::exception & error) {
        RCLCPP_WARN(logger, "Failed to load GNSS powerline overview from %s: %s", path.string().c_str(), error.what());
        return std::nullopt;
    }
}

bool persistPylonOverview(
    const iii_drone_interfaces::msg::PylonOverview & overview,
    const WorldGnssReference & reference,
    const std::filesystem::path & path,
    const rclcpp::Logger & logger
)
{
    try {
        std::filesystem::create_directories(path.parent_path());
        YAML::Node root;
        root["format"] = kPylonOverviewFormat;
        root["stored_frame_id"] = "gnss_wgs84";
        root["reference"]["drone_gnss"] = gnssPointToYaml(reference.drone_gnss);

        YAML::Node pylons(YAML::NodeType::Sequence);
        for (const auto & pylon : overview.pylons) {
            YAML::Node item;
            item["id"] = pylon.id;
            iii_drone::types::point_t point;
            point.x() = static_cast<float>(pylon.x);
            point.y() = static_cast<float>(pylon.y);
            point.z() = reference.drone_world.z();
            item["position"] = gnssPointToYaml(worldToGnss(point, reference));
            pylons.push_back(item);
        }
        root["pylons"] = pylons;

        std::ofstream stream(path);
        stream << root;
        return true;
    } catch (const std::exception & error) {
        RCLCPP_WARN(logger, "Failed to persist GNSS pylon overview to %s: %s", path.string().c_str(), error.what());
        return false;
    }
}

std::optional<iii_drone_interfaces::msg::PylonOverview> loadPylonOverview(
    const std::filesystem::path & path,
    const WorldGnssReference & reference,
    const rclcpp::Logger & logger
)
{
    if (!std::filesystem::exists(path)) {
        return std::nullopt;
    }
    try {
        YAML::Node root = YAML::LoadFile(path.string());
        if (root["format"].as<std::string>() != kPylonOverviewFormat) {
            RCLCPP_WARN(logger, "Ignoring unsupported pylon overview persistence format in %s", path.string().c_str());
            return std::nullopt;
        }

        iii_drone_interfaces::msg::PylonOverview overview;
        overview.stamp = rclcpp::Clock().now();
        overview.frame_id = "world";
        for (const auto & item : root["pylons"]) {
            const auto point = gnssToWorld(gnssPointFromYaml(item["position"]), reference);
            iii_drone_interfaces::msg::Pylon pylon;
            pylon.id = item["id"].as<int32_t>();
            pylon.x = point.x();
            pylon.y = point.y();
            overview.pylons.push_back(pylon);
        }
        return overview;
    } catch (const std::exception & error) {
        RCLCPP_WARN(logger, "Failed to load GNSS pylon overview from %s: %s", path.string().c_str(), error.what());
        return std::nullopt;
    }
}

} // namespace overview_gnss
} // namespace mission
} // namespace iii_drone
