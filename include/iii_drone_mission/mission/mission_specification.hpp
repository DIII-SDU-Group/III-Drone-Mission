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
        MissionSpecification(const std::string& mission_specification_file);

        mission_specification_entry_t GetMissionSpecificationEntry(const std::string& key) const;

        MissionSpecificationIterator begin();
        MissionSpecificationIterator end();

        typedef std::shared_ptr<MissionSpecification> SharedPtr;

    private:
        std::map<std::string, mission_specification_entry_t> mission_specification_entries_;

    };

} // namespace mission
} // namespace iii_drone