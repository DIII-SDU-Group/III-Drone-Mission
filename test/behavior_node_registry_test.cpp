#include <iii_drone_mission/behavior/behavior_node_registry.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{

BT::NodeBuilder ModelOnlyBuilder()
{
    return [](const std::string &, const BT::NodeConfig &) -> std::unique_ptr<BT::TreeNode> {
        throw std::logic_error("model-only test builder");
    };
}

void RegisterAuthoritativeManifests(BT::BehaviorTreeFactory & factory)
{
    for (const auto & manifest : iii_drone::behavior::CustomBehaviorNodeManifests()) {
        factory.registerBuilder(manifest, ModelOnlyBuilder());
    }
}

}  // namespace

TEST(BehaviorNodeRegistryTest, CompleteAuthoritativeFactoryPasses)
{
    BT::BehaviorTreeFactory factory;
    RegisterAuthoritativeManifests(factory);
    EXPECT_NO_THROW(iii_drone::behavior::ValidateRuntimeBehaviorFactory(factory));
}

TEST(BehaviorNodeRegistryTest, MissingAndUndeclaredRuntimeNodesFail)
{
    BT::BehaviorTreeFactory missing;
    EXPECT_THROW(
        iii_drone::behavior::ValidateRuntimeBehaviorFactory(missing),
        std::runtime_error
    );

    BT::BehaviorTreeFactory extra;
    RegisterAuthoritativeManifests(extra);
    BT::TreeNodeManifest undeclared{
        BT::NodeType::ACTION,
        "UndeclaredRuntimeNode",
        {},
        {},
    };
    extra.registerBuilder(undeclared, ModelOnlyBuilder());
    EXPECT_THROW(
        iii_drone::behavior::ValidateRuntimeBehaviorFactory(extra),
        std::runtime_error
    );
}

TEST(BehaviorNodeRegistryTest, PortContractDivergenceFails)
{
    BT::BehaviorTreeFactory factory;
    auto manifests = iii_drone::behavior::CustomBehaviorNodeManifests();
    ASSERT_FALSE(manifests.empty());
    manifests.front().ports.insert(
        BT::InputPort<std::string>("undeclared_test_port", "must diverge")
    );
    for (const auto & manifest : manifests) {
        factory.registerBuilder(manifest, ModelOnlyBuilder());
    }
    EXPECT_THROW(
        iii_drone::behavior::ValidateRuntimeBehaviorFactory(factory),
        std::runtime_error
    );
}
