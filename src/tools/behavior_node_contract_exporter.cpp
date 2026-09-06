#include <iii_drone_mission/behavior/behavior_node_registry.hpp>

#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/xml_parsing.h>

#include <openssl/sha.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

std::string NodeType(BT::NodeType type)
{
    switch (type) {
        case BT::NodeType::ACTION:
            return "ACTION";
        case BT::NodeType::CONDITION:
            return "CONDITION";
        case BT::NodeType::CONTROL:
            return "CONTROL";
        case BT::NodeType::DECORATOR:
            return "DECORATOR";
        case BT::NodeType::SUBTREE:
            return "SUBTREE";
        case BT::NodeType::UNDEFINED:
            return "UNDEFINED";
    }
    return "UNDEFINED";
}

std::string PortDirection(BT::PortDirection direction)
{
    switch (direction) {
        case BT::PortDirection::INPUT:
            return "INPUT";
        case BT::PortDirection::OUTPUT:
            return "OUTPUT";
        case BT::PortDirection::INOUT:
            return "INOUT";
    }
    return "UNKNOWN";
}

std::string Sha256(const std::string & value)
{
    unsigned char digest[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char *>(value.data()), value.size(), digest);
    std::ostringstream output;
    output << "sha256:" << std::hex << std::setfill('0');
    for (const auto byte : digest) {
        output << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return output.str();
}

void WriteFile(const std::filesystem::path & path, const std::string & value)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        throw std::runtime_error("cannot open output: " + path.string());
    }
    stream.write(value.data(), static_cast<std::streamsize>(value.size()));
    if (!stream) {
        throw std::runtime_error("cannot write output: " + path.string());
    }
}

}  // namespace

int main(int argc, char ** argv)
{
    if (argc != 3) {
        std::cerr << "usage: behavior_node_contract_exporter <contract.json> <models.xml>\n";
        return 2;
    }
    try {
        BT::BehaviorTreeFactory factory;
        for (const auto & manifest : iii_drone::behavior::CustomBehaviorNodeManifests()) {
            factory.registerBuilder(
                manifest,
                [](const std::string &, const BT::NodeConfig &) -> std::unique_ptr<BT::TreeNode> {
                    throw std::logic_error("model-only behavior-node builder cannot instantiate runtime nodes");
                }
            );
        }

        nlohmann::json nodes = nlohmann::json::array();
        std::map<std::string, const BT::TreeNodeManifest *> ordered;
        for (const auto & [id, manifest] : factory.manifests()) {
            ordered.emplace(id, &manifest);
        }
        for (const auto & [id, manifest] : ordered) {
            nlohmann::json ports = nlohmann::json::array();
            std::map<std::string, BT::PortInfo> ordered_ports(manifest->ports.begin(), manifest->ports.end());
            for (const auto & [name, port] : ordered_ports) {
                ports.push_back(
                    {
                        {"default", port.defaultValueString()},
                        {"description", port.description()},
                        {"direction", PortDirection(port.direction())},
                        {"name", name},
                        {"type", port.typeName()},
                    }
                );
            }
            nodes.push_back(
                {
                    {"builtin", factory.builtinNodes().count(id) != 0},
                    {"id", id},
                    {"ports", ports},
                    {"type", NodeType(manifest->type)},
                }
            );
        }
        nlohmann::json contract = {
            {"nodes", nodes},
            {"schema", "iii.behavior-node-contract/v1"},
        };
        contract["contract_hash"] = Sha256(contract.dump());
        WriteFile(argv[1], contract.dump() + "\n");
        WriteFile(argv[2], BT::writeTreeNodesModelXML(factory, false) + "\n");
    } catch (const std::exception & exception) {
        std::cerr << "behavior-node contract export failed: " << exception.what() << "\n";
        return 2;
    }
    return 0;
}
