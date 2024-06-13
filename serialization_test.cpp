#include <iii_drone_interfaces/msg/target.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dynmsg/message_reading.hpp"
#include "dynmsg/msg_parser.hpp"
#include "dynmsg/typesupport.hpp"
#include "dynmsg/yaml_utils.hpp"

#include <iostream>
#include <string>

#include "std_msgs/msg/header.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

#include <iii_drone_mission/behavior/port_types.hpp>

static constexpr char std_msgs_pkg[] = "std_msgs";
static constexpr char std_msgs_header_name[] = "Header";

class A : BT::SyncActionNode {
public:
    A(const std::string & name, const BT::NodeConfig & config) : BT::SyncActionNode(name, config) { }

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std_msgs::msg::Header>("msg") };
    }

    BT::NodeStatus tick() override {

        std::cout << "A::tick()" << std::endl;
        std_msgs::msg::Header msg;
        getInput<std_msgs::msg::Header>("msg", msg);

        std::string msg_yaml = iii_drone::behavior::msgToYaml<
            std_msgs::msg::Header, 
            std_msgs_pkg,
            std_msgs_header_name
        >(msg);

        std::cout << msg_yaml << std::endl;

        return BT::NodeStatus::SUCCESS;

    }

};

class B : BT::SyncActionNode {
public:
    B(const std::string & name, const BT::NodeConfig & config) : BT::SyncActionNode(name, config) { }

    static BT::PortsList providedPorts() {
        return { BT::OutputPort<std_msgs::msg::Header>("msg") };
    }

    BT::NodeStatus tick() override {

        std::cout << "B::tick()" << std::endl;
        std_msgs::msg::Header msg;
        msg.frame_id = "my_frame";
        msg.stamp.sec = 4;
        msg.stamp.nanosec = 20U;

        setOutput<std_msgs::msg::Header>("msg", msg);

        return BT::NodeStatus::SUCCESS;

    }

};

static const char * tree_xml = R"(
<root BTCPP_format="4" >
    <BehaviorTree ID="MainTree">
       <Sequence name="root_sequence">
           <A/>
           <B/>
       </Sequence>
    </BehaviorTree>
</root>
)";


int main(int argc, char **argv) {
    
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<A>("A");
    factory.registerNodeType<B>("B");

    auto tree = factory.createTreeFromText(tree_xml);

    return 0;
}