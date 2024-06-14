/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <iii_drone_mission/behavior/port_types.hpp>

/*****************************************************************************/
// Methods:
/*****************************************************************************/

namespace BT {

    template <> inline iii_drone_interfaces::msg::Target convertFromString(StringView str) {

        return iii_drone::behavior::yamlToMsg<
            iii_drone_interfaces::msg::Target, 
            iii_drone_interfaces_pkg, 
            iii_drone_interfaces_target_name
        >(str.data());

    }
}