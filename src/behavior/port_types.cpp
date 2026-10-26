/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/port_types.hpp>

#include <charconv>
#include <cmath>
#include <stdexcept>
#include <system_error>

using namespace iii_drone::behavior;

/*****************************************************************************/
// Methods:
/*****************************************************************************/

namespace {

    std::string_view trim(std::string_view text) {

        const auto first = text.find_first_not_of(" \t\n\r");
        if (first == std::string_view::npos) {
            return {};
        }
        const auto last = text.find_last_not_of(" \t\n\r");
        return text.substr(first, last - first + 1);

    }

    float parseCoordinate(std::string_view text, std::size_t point_index) {

        std::string_view number = trim(text);
        // std::from_chars rejects an explicit plus sign; accept "+1.5".
        if (number.size() > 1 && number.front() == '+' && number[1] != '-' && number[1] != '+') {
            number.remove_prefix(1);
        }

        double value = 0.0;
        const auto * const end = number.data() + number.size();
        const auto [parsed_end, error] = std::from_chars(number.data(), end, value);
        const float coordinate = static_cast<float>(value);
        if (number.empty() || error != std::errc() || parsed_end != end || !std::isfinite(coordinate)) {
            throw std::invalid_argument(
                "point " + std::to_string(point_index) + " has an invalid coordinate '" +
                std::string(trim(text)) + "'"
            );
        }
        return coordinate;

    }

}  // namespace

std::deque<iii_drone::types::point_t> iii_drone::behavior::ParsePointListLiteral(std::string_view text) {

    if (trim(text).empty()) {
        throw std::invalid_argument("point list is empty; expected \"x,y,z;x,y,z;...\"");
    }

    std::deque<iii_drone::types::point_t> points;
    std::size_t entry_start = 0;
    while (true) {
        const auto entry_end = text.find(';', entry_start);
        const auto entry = text.substr(
            entry_start,
            entry_end == std::string_view::npos ? std::string_view::npos : entry_end - entry_start
        );
        const std::size_t point_index = points.size();
        if (trim(entry).empty()) {
            throw std::invalid_argument(
                "point " + std::to_string(point_index) + " is empty; expected \"x,y,z;x,y,z;...\""
            );
        }

        iii_drone::types::point_t point;
        std::size_t coordinate_start = 0;
        for (int axis = 0; axis < 3; ++axis) {
            const auto coordinate_end = entry.find(',', coordinate_start);
            const bool last_axis = axis == 2;
            if (last_axis != (coordinate_end == std::string_view::npos)) {
                throw std::invalid_argument(
                    "point " + std::to_string(point_index) + " '" + std::string(trim(entry)) +
                    "' must have exactly three comma-separated coordinates"
                );
            }
            point[axis] = parseCoordinate(
                entry.substr(
                    coordinate_start,
                    last_axis ? std::string_view::npos : coordinate_end - coordinate_start
                ),
                point_index
            );
            coordinate_start = coordinate_end + 1;
        }
        points.push_back(point);

        if (entry_end == std::string_view::npos) {
            break;
        }
        entry_start = entry_end + 1;
    }

    return points;

}

namespace BT {

    // template <> inline iii_drone_interfaces::msg::Target convertFromString(StringView str) {

    //     return yamlToMsg<
    //         iii_drone_interfaces::msg::Target, 
    //         iii_drone_interfaces_pkg, 
    //         iii_drone_interfaces_target_name
    //     >(str.data());

    // }

    // template <> inline iii_drone::types::point_t convertFromString(StringView str) {

    //     // Split string by comma, "x,y,z":
    //     std::vector<std::string> tokens;
    //     std::string token;
    //     std::istringstream tokenStream(str.data());
    //     while (std::getline(tokenStream, token, ',')) {
    //         tokens.push_back(token);
    //     }

    //     // Convert tokens to float values:
    //     iii_drone::types::point_t point;
    //     point[0] = std::stof(tokens[0]);
    //     point[1] = std::stof(tokens[1]);
    //     point[2] = std::stof(tokens[2]);

    //     return point;

    // }
}