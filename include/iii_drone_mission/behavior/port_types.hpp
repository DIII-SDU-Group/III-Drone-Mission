#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Behaviortree.CPP:

#include "behaviortree_cpp/bt_factory.h"

/*****************************************************************************/
// Dynamic message introspection:

#include "dynmsg/message_reading.hpp"
#include "dynmsg/msg_parser.hpp"
#include "dynmsg/typesupport.hpp"
#include "dynmsg/yaml_utils.hpp"

/*****************************************************************************/
// Std:

#include <string>

/*****************************************************************************/
// III-Drone-Interfaces:

#include <iii_drone_interfaces/msg/target.hpp>

/*****************************************************************************/
// III-Drone-Core:

#include <iii_drone_core/utils/types.hpp>

/*****************************************************************************/
// Definitions
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    typedef enum {
        PL_MAPPER_COMMAND_START = 0,
        PL_MAPPER_COMMAND_STOP = 1,
        PL_MAPPER_COMMAND_PAUSE = 2,
        PL_MAPPER_COMMAND_FREEZE = 3
    } pl_mapper_command_t;

    typedef enum {
        PL_MAPPER_ACK_SUCCESS = 0,
        PL_MAPPER_ACK_INVALID_COMMAND = 1,
        PL_MAPPER_ACK_UNKNOWN_ERROR = 2
    } pl_mapper_ack_t;

    typedef enum {
        TARGET_PROVIDER_MODE_FLY_TO_CABLE = 0,
        TARGET_PROVIDER_MODE_HOVER_BY_CABLE = 1
    } target_provider_mode_t;

    typedef enum {
        GRIPPER_COMMAND_OPEN = 0,
        GRIPPER_COMMAND_CLOSE = 1
    } gripper_command_t;

    typedef enum {
        GRIPPER_COMMAND_RESPONSE_SUCCESS = 0,
        GRIPPER_COMMAND_RESPONSE_INVALID_COMMAND = 1,
        GRIPPER_COMMAND_RESPONSE_ERROR = 2,
        GRIPPER_COMMAND_RESPONSE_TIMEOUT = 3
    } gripper_command_response_t;

} // namespace behavior
} // namespace iii_drone

/*****************************************************************************/
// Methods:
/*****************************************************************************/

namespace iii_drone {
namespace behavior {

    template <typename msg_T, const char *msg_pkg, const char *msg_name>
    const std::string msgToYaml(msg_T msg) {

        RosMessage_Cpp ros_msg;

        InterfaceTypeName interface{
            std::string(msg_pkg),
            std::string(msg_name)
        };

        ros_msg.type_info = dynmsg::cpp::get_type_info(interface);
        ros_msg.data = reinterpret_cast<uint8_t *>(&msg);

        const std::string yaml_string = dynmsg::yaml_to_string(
            dynmsg::cpp::message_to_yaml(
                ros_msg
            )
        );

        return yaml_string;

    }

    template <typename msg_T, const char *msg_pkg, const char *msg_name>
    msg_T yamlToMsg(std::string yaml_str) {

        msg_T msg;

        InterfaceTypeName interface{
            std::string(msg_pkg),
            std::string(msg_name)
        };

        void * ros_message = reinterpret_cast<void *>(&msg);

        dynmsg::cpp::yaml_and_typeinfo_to_rosmsg(
            dynmsg::cpp::get_type_info(interface),
            yaml_str,
            ros_message
        );

        return msg;

    }

}
}

namespace BT {

    static constexpr char iii_drone_interfaces_pkg[] = "iii_interfaces";
    static constexpr char iii_drone_interfaces_target_name[] = "Target";

    // template <> inline iii_drone_interfaces::msg::Target convertFromString(StringView str);
    // template <> inline iii_drone::types::point_t convertFromString(StringView str);

    template <> inline iii_drone_interfaces::msg::Target convertFromString(StringView str) {

        return iii_drone::behavior::yamlToMsg<
            iii_drone_interfaces::msg::Target, 
            iii_drone_interfaces_pkg, 
            iii_drone_interfaces_target_name
        >(str.data());

    }

    template <> inline iii_drone::types::point_t convertFromString(StringView str) {

        // Split string by comma, "x,y,z":
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(str.data());
        while (std::getline(tokenStream, token, ',')) {
            tokens.push_back(token);
        }

        // Convert tokens to float values:
        iii_drone::types::point_t point;
        point[0] = std::stof(tokens[0]);
        point[1] = std::stof(tokens[1]);
        point[2] = std::stof(tokens[2]);

        return point;

    }

}