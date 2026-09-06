/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/powerline_geometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace iii_drone::behavior::powerline_geometry;
using namespace iii_drone::types;

/*****************************************************************************/
// Helpers
/*****************************************************************************/

namespace {

    point_t xyOnly(point_t point) {
        point[2] = 0.0;
        return point;
    }

    std::optional<vector_t> normalized(vector_t vector) {
        const double norm = vector.norm();
        if (norm < 1e-6) {
            return std::nullopt;
        }
        return vector / norm;
    }

    double acuteAngleBetween(const vector_t & lhs, const vector_t & rhs) {
        const auto lhs_normalized = normalized(lhs);
        const auto rhs_normalized = normalized(rhs);
        if (!lhs_normalized || !rhs_normalized) {
            return std::numeric_limits<double>::infinity();
        }
        const double absolute_dot = std::abs(lhs_normalized->dot(*rhs_normalized));
        const double clamped_dot = std::clamp(absolute_dot, 0.0, 1.0);
        return std::acos(clamped_dot);
    }

} // namespace

/*****************************************************************************/
// Implementation
/*****************************************************************************/

std::optional<PowerlineAxes> iii_drone::behavior::powerline_geometry::ComputeAxes(
    const vector_t & powerline_direction
) {
    const auto direction = normalized(powerline_direction);
    if (!direction) {
        return std::nullopt;
    }

    vector_t cross_corridor = *direction;
    cross_corridor[0] = -(*direction)[1];
    cross_corridor[1] = (*direction)[0];

    vector_t direction_no_z = *direction;
    direction_no_z[2] = 0.0;
    const auto direction_no_z_normalized = normalized(direction_no_z);
    if (!direction_no_z_normalized) {
        return std::nullopt;
    }

    vector_t cross_corridor_no_z = cross_corridor;
    cross_corridor_no_z[2] = 0.0;
    const auto cross_corridor_no_z_normalized = normalized(cross_corridor_no_z);
    if (!cross_corridor_no_z_normalized) {
        return std::nullopt;
    }

    return PowerlineAxes{
        .direction = *direction,
        .cross_corridor = cross_corridor,
        .direction_no_z = *direction_no_z_normalized,
        .cross_corridor_no_z = *cross_corridor_no_z_normalized
    };
}

std::optional<PowerlineAxes> iii_drone::behavior::powerline_geometry::ComputePylonAlignedAxes(
    const vector_t & powerline_direction,
    const point_t & pylon_a,
    const point_t & pylon_b
) {
    const auto powerline_axes = ComputeAxes(powerline_direction);
    if (!powerline_axes) {
        return std::nullopt;
    }

    vector_t pylon_direction = xyOnly(pylon_b - pylon_a);
    if (pylon_direction.norm() < 1e-6) {
        return std::nullopt;
    }
    if (pylon_direction.dot(powerline_axes->direction_no_z) < 0.0) {
        pylon_direction *= -1.0;
    }
    return ComputeAxes(pylon_direction);
}

std::optional<double> iii_drone::behavior::powerline_geometry::ComputePowerlineAlignedYaw(
    const vector_t & powerline_direction,
    double current_yaw
) {
    const auto axes = ComputeAxes(powerline_direction);
    if (!axes) {
        return std::nullopt;
    }

    const double direction_yaw = std::atan2(
        axes->direction_no_z[1],
        axes->direction_no_z[0]
    );
    const auto shortest_error = [current_yaw](double target_yaw) {
        return std::atan2(
            std::sin(target_yaw - current_yaw),
            std::cos(target_yaw - current_yaw)
        );
    };

    const double forward_error = shortest_error(direction_yaw);
    const double reverse_error = shortest_error(direction_yaw + M_PI);
    return current_yaw + (
        std::abs(forward_error) <= std::abs(reverse_error)
            ? forward_error
            : reverse_error
    );
}

std::optional<double> iii_drone::behavior::powerline_geometry::ComputeCableFacingYaw(
    const vector_t & cross_corridor_no_z,
    bool positive_side_is_entry
) {
    vector_t toward_cable = cross_corridor_no_z;
    toward_cable[2] = 0.0;
    const auto normalized_toward_cable = normalized(toward_cable);
    if (!normalized_toward_cable) {
        return std::nullopt;
    }

    // The approach point is displaced outward from the selected conductor.
    // From the positive side, the conductor is therefore in the negative
    // cross-corridor direction; from the negative side it is positive.
    if (positive_side_is_entry) {
        toward_cable *= -1.0;
    }
    return std::atan2(toward_cable[1], toward_cable[0]);
}

std::optional<point_t> iii_drone::behavior::powerline_geometry::SelectHighestPoint(
    const std::vector<point_t> & points
) {
    if (points.empty()) {
        return std::nullopt;
    }

    return *std::max_element(
        points.begin(),
        points.end(),
        [](const point_t & lhs, const point_t & rhs) {
            return lhs[2] < rhs[2];
        }
    );
}

std::optional<PowerlineSideSplit> iii_drone::behavior::powerline_geometry::SplitByLargestLateralGap(
    const std::vector<point_t> & points,
    const vector_t & cross_corridor_no_z
) {
    if (points.size() < 2) {
        return std::nullopt;
    }

    const auto cross = normalized(cross_corridor_no_z);
    if (!cross) {
        return std::nullopt;
    }

    std::vector<std::pair<double, point_t>> sorted_points;
    sorted_points.reserve(points.size());
    for (const auto & point : points) {
        sorted_points.emplace_back(xyOnly(point).dot(*cross), point);
    }

    std::sort(
        sorted_points.begin(),
        sorted_points.end(),
        [](const auto & lhs, const auto & rhs) {
            return lhs.first < rhs.first;
        }
    );

    size_t split_index = sorted_points.size() / 2;
    double largest_gap = -1.0;
    for (size_t i = 0; i + 1 < sorted_points.size(); ++i) {
        const double gap = sorted_points.at(i + 1).first - sorted_points.at(i).first;
        if (gap > largest_gap) {
            largest_gap = gap;
            split_index = i + 1;
        }
    }

    if (split_index == 0 || split_index >= sorted_points.size()) {
        split_index = sorted_points.size() / 2;
    }

    PowerlineSideSplit split;
    split.largest_gap = largest_gap;
    split.negative_points.reserve(split_index);
    split.positive_points.reserve(sorted_points.size() - split_index);

    for (size_t i = 0; i < sorted_points.size(); ++i) {
        if (i < split_index) {
            split.negative_points.push_back(sorted_points.at(i).second);
        } else {
            split.positive_points.push_back(sorted_points.at(i).second);
        }
    }

    if (split.negative_points.empty() || split.positive_points.empty()) {
        return std::nullopt;
    }

    split.middle_point = (split.negative_points.back() + split.positive_points.front()) / 2.0;
    return split;
}

std::optional<point_t> iii_drone::behavior::powerline_geometry::LowestPointByZ(
    const std::vector<point_t> & points
) {
    if (points.empty()) {
        return std::nullopt;
    }

    return *std::min_element(
        points.begin(),
        points.end(),
        [](const point_t & lhs, const point_t & rhs) {
            return lhs[2] < rhs[2];
        }
    );
}

std::optional<point_t> iii_drone::behavior::powerline_geometry::FurthestPointXY(
    const std::vector<point_t> & points,
    const point_t & reference
) {
    if (points.empty()) {
        return std::nullopt;
    }

    const point_t reference_xy = xyOnly(reference);
    return *std::max_element(
        points.begin(),
        points.end(),
        [&](const point_t & lhs, const point_t & rhs) {
            return (xyOnly(lhs) - reference_xy).norm() < (xyOnly(rhs) - reference_xy).norm();
        }
    );
}

bool iii_drone::behavior::powerline_geometry::PositiveSideIsEntrySide(
    const PowerlineSideSplit & split
) {
    const auto positive_lowest = LowestPointByZ(split.positive_points);
    const auto negative_lowest = LowestPointByZ(split.negative_points);
    if (!positive_lowest || !negative_lowest) {
        return false;
    }
    return (*positive_lowest)[2] > (*negative_lowest)[2];
}

CorridorClassification iii_drone::behavior::powerline_geometry::ClassifyCorridor(
    const point_t & start_position,
    const point_t & middle_point,
    const vector_t & cross_corridor_no_z,
    const point_t & positive_outer_point,
    const point_t & negative_outer_point,
    double inside_corridor_threshold_m
) {
    CorridorClassification classification;
    const auto cross = normalized(cross_corridor_no_z);
    if (!cross) {
        return classification;
    }

    const point_t middle_xy = xyOnly(middle_point);
    const point_t start_xy = xyOnly(start_position);
    const point_t positive_outer_xy = xyOnly(positive_outer_point);
    const point_t negative_outer_xy = xyOnly(negative_outer_point);

    classification.start_cross_dot = (start_xy - middle_xy).dot(*cross);
    classification.positive_outer_cross_dot = (positive_outer_xy - middle_xy).dot(*cross);
    classification.negative_outer_cross_dot = (negative_outer_xy - middle_xy).dot(*cross);
    classification.lateral_distance_to_middle = std::abs(classification.start_cross_dot);
    classification.inside_corridor = classification.lateral_distance_to_middle < inside_corridor_threshold_m;
    classification.on_positive_outer_side =
        classification.start_cross_dot > classification.positive_outer_cross_dot;
    classification.on_negative_outer_side =
        classification.start_cross_dot < classification.negative_outer_cross_dot;

    return classification;
}

bool iii_drone::behavior::powerline_geometry::PylonSpanMatchesPowerlineDirection(
    const point_t & pylon_a,
    const point_t & pylon_b,
    const vector_t & powerline_direction_no_z,
    double max_mismatch_rad
) {
    vector_t span_direction = pylon_b - pylon_a;
    span_direction[2] = 0.0;

    return acuteAngleBetween(span_direction, powerline_direction_no_z) <= max_mismatch_rad;
}
