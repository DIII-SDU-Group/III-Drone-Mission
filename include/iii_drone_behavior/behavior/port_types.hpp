#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

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

} // namespace behavior
} // namespace iii_drone