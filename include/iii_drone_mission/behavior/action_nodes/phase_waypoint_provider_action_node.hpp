#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>

/*****************************************************************************/
// III-Drone-Configuration:

#include <iii_drone_configuration/configuration.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/adapters/powerline_adapter.hpp>
#include <iii_drone_core/control/state.hpp>
#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/powerline.hpp>
#include <iii_drone_interfaces/msg/pylon_overview.hpp>

/*****************************************************************************/
// III-Drone-Mission:

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// BT.CPP:

#include <behaviortree_cpp/action_node.h>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    struct CorridorInspectionRoute {
        std::deque<iii_drone::types::point_t> waypoints;
        std::size_t loop_start_index = 0;
        std::size_t loop_route_offset = 0;
        std::string selected_route;
        bool resumed = false;
    };

    struct CorridorInspectionResume {
        std::string selected_route;
        std::size_t active_waypoint_index = 0;
        std::size_t loop_start_index = 0;
        std::size_t loop_route_offset = 0;
        iii_drone::types::point_t interrupted_position;
    };

    struct CorridorInspectionEligibility {
        bool evaluable = false;
        bool eligible = false;
        std::string side = "unknown";
        double measured_lateral_clearance_m = 0.0;
        double required_lateral_clearance_m = 0.0;
        bool between_pylons = false;
        double distance_from_start_boundary_m = 0.0;
        double distance_to_end_boundary_m = 0.0;
        double pylon_span_margin_m = 0.0;
        iii_drone::types::point_t ingress_point = iii_drone::types::point_t::Zero();
        bool ingress_point_valid = false;
        std::vector<std::string> failure_reasons;
    };

    CorridorInspectionEligibility EvaluateCorridorInspectionStart(
        const std::vector<iii_drone::types::point_t> & powerline_points,
        const iii_drone::types::vector_t & powerline_direction,
        const iii_drone::types::point_t & pylon_a,
        const iii_drone::types::point_t & pylon_b,
        const iii_drone::types::point_t & start_position,
        double inspection_clearance_m,
        double pylon_end_clearance_m,
        double pylon_structure_extent_m,
        double pylon_span_margin_m,
        double max_pylon_direction_mismatch_rad
    );

    std::optional<CorridorInspectionRoute> BuildCorridorInspectionRoute(
        const std::vector<iii_drone::types::point_t> & powerline_points,
        const iii_drone::types::vector_t & powerline_direction,
        const iii_drone::types::point_t & pylon_a,
        const iii_drone::types::point_t & pylon_b,
        const iii_drone::types::point_t & start_position,
        double inspection_clearance_m,
        double pylon_end_clearance_m,
        double pylon_structure_extent_m,
        double under_cable_clearance_m,
        double inside_corridor_threshold_m,
        double pylon_span_margin_m,
        double max_pylon_direction_mismatch_rad,
        const std::optional<CorridorInspectionResume> & resume = std::nullopt,
        double resume_position_tolerance_m = 0.75
    );

    class PhaseWaypointProviderActionNode : public BT::SyncActionNode {
    public:
        PhaseWaypointProviderActionNode(
            const std::string & name,
            const BT::NodeConfig & config,
            rclcpp::Node * node,
            iii_drone::configuration::Configuration::SharedPtr configuration
        );

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node * node_;
        iii_drone::configuration::Configuration::SharedPtr configuration_;

        double parameterOr(const std::string & name, double fallback) const;
    };

} // namespace behavior
} // namespace iii_drone
