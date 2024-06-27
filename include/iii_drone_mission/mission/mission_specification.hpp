#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <string>
#include <map>
#include <wordexp.h>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
// Yaml-CPP:

#include <yaml-cpp/yaml.h>

/*****************************************************************************/
// Defines:
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    typedef struct {

        std::string key;
        std::string mode_name;
        std::string behavior_tree_xml_file;
        std::string next_mode;

        bool allow_activate_when_disarmed;
        bool land_when_finished;
        bool arm_when_finished;

    } mission_specification_entry_t;

} // namespace mission
} // namespace iii_drone

/*****************************************************************************/
// Class:
/*****************************************************************************/

namespace iii_drone {
namespace mission {

    class MissionSpecificationIterator {
    public:
        typedef std::map<std::string, mission_specification_entry_t>::iterator base_iterator;

        MissionSpecificationIterator(base_iterator it);

        mission_specification_entry_t operator*() const;
        MissionSpecificationIterator& operator++();
        bool operator!=(const MissionSpecificationIterator& other) const;
    
    private:
        base_iterator it_;

    };

    class MissionSpecification {
    public:
        MissionSpecification(
            const std::string& mission_specification_file,
            rclcpp_lifecycle::LifecycleNode * node
        );

        mission_specification_entry_t GetMissionSpecificationEntry(const std::string& key) const;

        MissionSpecificationIterator begin();
        MissionSpecificationIterator end();

        std::string executor_owned_mode() const;

        typedef std::shared_ptr<MissionSpecification> SharedPtr;

    private:
        std::map<std::string, mission_specification_entry_t> mission_specification_entries_;

        std::string executor_owned_mode_;

        rclcpp_lifecycle::LifecycleNode * node_;

    };

} // namespace mission
} // namespace iii_drone