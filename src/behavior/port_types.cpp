/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/port_types.hpp>

using namespace iii_drone::behavior;

/*****************************************************************************/
// Methods:
/*****************************************************************************/

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