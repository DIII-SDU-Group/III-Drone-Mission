#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <optional>
#include <vector>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {
namespace powerline_geometry {

    struct PowerlineAxes {
        iii_drone::types::vector_t direction;
        iii_drone::types::vector_t cross_corridor;
        iii_drone::types::vector_t direction_no_z;
        iii_drone::types::vector_t cross_corridor_no_z;
    };

    struct PowerlineSideSplit {
        std::vector<iii_drone::types::point_t> positive_points;
        std::vector<iii_drone::types::point_t> negative_points;
        iii_drone::types::point_t middle_point;
        double largest_gap = 0.0;
    };

    struct CorridorClassification {
        bool inside_corridor = false;
        bool on_positive_outer_side = false;
        bool on_negative_outer_side = false;
        double lateral_distance_to_middle = 0.0;
        double start_cross_dot = 0.0;
        double positive_outer_cross_dot = 0.0;
        double negative_outer_cross_dot = 0.0;
    };

    enum class InsideCorridorRouteKind {
        direct,
        same_side_exterior_clearance,
        top_clearance
    };

    struct InsideCorridorRoute {
        InsideCorridorRouteKind kind = InsideCorridorRouteKind::direct;
        std::vector<iii_drone::types::point_t> waypoints;
        std::vector<iii_drone::types::point_t> return_waypoints;
        // Index in waypoints for the low outside-corridor point, or -1 when
        // this route does not have a separately completed outside boundary.
        int outside_boundary_index = -1;
    };

    std::optional<PowerlineAxes> ComputeAxes(
        const iii_drone::types::vector_t & powerline_direction
    );

    std::optional<PowerlineAxes> ComputePylonAlignedAxes(
        const iii_drone::types::vector_t & powerline_direction,
        const iii_drone::types::point_t & pylon_a,
        const iii_drone::types::point_t & pylon_b
    );

    std::optional<double> ComputePowerlineAlignedYaw(
        const iii_drone::types::vector_t & powerline_direction,
        double current_yaw
    );

    // Approach the selected outer conductor from outside the corridor while
    // keeping forward-facing sensors pointed back toward that conductor.
    std::optional<double> ComputeCableFacingYaw(
        const iii_drone::types::vector_t & cross_corridor_no_z,
        bool positive_side_is_entry
    );

    std::optional<iii_drone::types::point_t> SelectHighestPoint(
        const std::vector<iii_drone::types::point_t> & points
    );

    std::optional<PowerlineSideSplit> SplitByLargestLateralGap(
        const std::vector<iii_drone::types::point_t> & points,
        const iii_drone::types::vector_t & cross_corridor_no_z
    );

    std::optional<iii_drone::types::point_t> LowestPointByZ(
        const std::vector<iii_drone::types::point_t> & points
    );

    std::optional<iii_drone::types::point_t> FurthestPointXY(
        const std::vector<iii_drone::types::point_t> & points,
        const iii_drone::types::point_t & reference
    );

    bool PositiveSideIsEntrySide(const PowerlineSideSplit & split);

    CorridorClassification ClassifyCorridor(
        const iii_drone::types::point_t & start_position,
        const iii_drone::types::point_t & middle_point,
        const iii_drone::types::vector_t & cross_corridor_no_z,
        const iii_drone::types::point_t & positive_outer_point,
        const iii_drone::types::point_t & negative_outer_point,
        double inside_corridor_threshold_m
    );

    // Preserve the conductor's span station and altitude while clearing both
    // its horizontal margin and the corridor including arrival tolerance.
    std::optional<iii_drone::types::point_t> OutsideCorridorClearancePoint(
        const iii_drone::types::point_t & outer_point,
        const iii_drone::types::point_t & middle_point,
        const iii_drone::types::vector_t & cross_corridor_no_z,
        bool positive_side,
        double inside_corridor_threshold_m,
        double completion_tolerance_m,
        double horizontal_clearance_m
    );

    InsideCorridorRoute BuildInsideCorridorReturnRoute(
        const iii_drone::types::point_t & start_position,
        const iii_drone::types::point_t & middle_point,
        const iii_drone::types::point_t & entry_under_point,
        const iii_drone::types::point_t & positive_outer_point,
        const iii_drone::types::point_t & negative_outer_point,
        const iii_drone::types::vector_t & cross_corridor_no_z,
        bool positive_side_is_entry,
        double inside_corridor_threshold_m,
        double completion_tolerance_m,
        double horizontal_clearance_m
    );

    bool PylonSpanMatchesPowerlineDirection(
        const iii_drone::types::point_t & pylon_a,
        const iii_drone::types::point_t & pylon_b,
        const iii_drone::types::vector_t & powerline_direction_no_z,
        double max_mismatch_rad
    );

} // namespace powerline_geometry
} // namespace behavior
} // namespace iii_drone
