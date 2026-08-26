#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

/*****************************************************************************/
// Std:

#include <string>
#include <map>
#include <vector>

/*****************************************************************************/
// ROS2:

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

/*****************************************************************************/
#include <iii_drone_mission/mission/mission_catalog.hpp>

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

    } mission_specification_entry_t;

    typedef struct {

        std::string service_name;
        std::string flag_name;
        std::string type;
        std::vector<std::string> valid_modes;

    } mission_intent_service_t;

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
            MissionCatalog::SharedPtr catalog,
            const MissionCatalogEntry & catalog_entry,
            rclcpp_lifecycle::LifecycleNode * node
        );

        mission_specification_entry_t GetMissionSpecificationEntry(const std::string& key) const;

        MissionSpecificationIterator begin();
        MissionSpecificationIterator end();

        std::string executor_owned_mode() const;
        const std::string & catalog_id() const;
        const std::string & entry_hash() const;
        const std::string & catalog_hash() const;
        const std::string & classification() const;
        const std::vector<std::string> & compatible_profiles() const;
        const std::string & experimental_warning() const;
        std::vector<mission_specification_entry_t> entries() const;
        std::vector<std::string> mode_keys() const;
        std::vector<mission_intent_service_t> intent_services() const;

        typedef std::shared_ptr<MissionSpecification> SharedPtr;

    private:
        std::map<std::string, mission_specification_entry_t> mission_specification_entries_;
        std::vector<mission_intent_service_t> intent_services_;

        std::string executor_owned_mode_;
        MissionCatalog::SharedPtr catalog_;
        std::string catalog_id_;
        std::string entry_hash_;
        std::string catalog_hash_;
        std::string classification_;
        std::vector<std::string> compatible_profiles_;
        std::string experimental_warning_;

        rclcpp_lifecycle::LifecycleNode * node_;

    };

} // namespace mission
} // namespace iii_drone
