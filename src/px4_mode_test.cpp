#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <iii_drone_mission/px4/modes/maneuver_mode.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

#include <iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp>

#include <iii_drone_mission/mission/mission_executor.hpp>
#include <iii_drone_mission/mission/mission_specification.hpp>

#include <iostream>
#include <thread>

using namespace iii_drone::px4;
using namespace iii_drone::configuration;
using namespace iii_drone::mission;

// class TestMode : public px4_ros2::ModeBase
// {
// public:
//     explicit TestMode(rclcpp::Node & node) : px4_ros2::ModeBase(
//         node,
//         px4_ros2::ModeBase::Settings(
//             "Test mode",
//             false
//         )
//     )
//     {
//         RCLCPP_INFO(node.get_logger(), "TestMode::TestMode(): Initializing.");

//         traj_setpoint_ = std::make_shared<iii_drone::px4::TrajectorySetpoint>(*this);
//     }

//     void onActivate() override {
//         RCLCPP_INFO(node().get_logger(), "TestMode::onActivate(): Activating mode");
//     }

//     void onDeactivate() override {
//         RCLCPP_INFO(node().get_logger(), "TestMode::onDeactivate(): Deactivating mode");
//     }

//     void updateSetpoint(float dt) override {
//         RCLCPP_INFO(node().get_logger(), "TestMode::updateSetpoint(): Updating setpoint");
//     }

// private:
//     std::shared_ptr<iii_drone::px4::TrajectorySetpoint> traj_setpoint_;

// };

int main(int argc, char * argv[])
{
    std::cout << "yalla" << std::endl;

    rclcpp::init(argc, argv);

    // std::cout << "Initializing node" << std::endl;

    // // auto mode_node_ = std::make_shared<px4_ros2::NodeWithMode<TestMode>>(
    // //     "test_mode",
    // //     true
    // // );

    // auto mode_node_ = std::make_shared<rclcpp::Node>(
    //     "mode"
    // );

    // std::cout << "Initializing mode" << std::endl;

    // ManeuverMode::SharedPtr mode = std::make_shared<ManeuverMode>(
    //     *mode_node_,
    //     "Test mode",
    //     0.2,
    //     false,
    //     false
    // );

    // std::cout << "Spinning node in separate thread" << std::endl;

    // std::thread spin_thread([mode_node_](){
    //     rclcpp::executors::MultiThreadedExecutor executor;
    //     executor.add_node(mode_node_);
    //     executor.spin();
    // });

    // spin_thread.detach();

    // std::this_thread::sleep_for(std::chrono::seconds(5));

    // std::cout << "Registering mode" << std::endl;

    // // if (!mode->doRegister()) {
    // //     std::cout << "Failed to register mode" << std::endl;
    // //     return 1;
    // // }

    // mode->Register(nullptr, nullptr);

    // std::cout << "Looping" << std::endl;

    // while (rclcpp::ok()) {
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }


    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "mission_executor",
        "/mission/mission_executor"
    );

    executor.add_node(node->get_node_base_interface());

    std::cout << "Spinning  executor in separate thread" << std::endl;

    std::thread spin_thread([&executor](){
        executor.spin();
    });

    spin_thread.detach();

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "Creating mission specification" << std::endl;

    MissionSpecification::SharedPtr mission_specification_ = std::make_shared<MissionSpecification>(
        "$MISSION_SPECIFICATION_DIR/mission_specification.yaml",
        node.get()
    );

    std::cout << "Creating tf buffer" << std::endl;

    tf2_ros::Buffer::SharedPtr tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    std::cout << "Creating tree provider" << std::endl;

    // Tree provider
    auto tree_provider_ = std::make_shared<iii_drone::behavior::TreeProvider>(
        tf_buffer_,
        mission_specification_
    );

    std::cout << "Creating mode provider" << std::endl;

    // Modes provider
    auto mode_provider_ = std::make_shared<iii_drone::px4::ModeProvider>(
        tree_provider_,
        mission_specification_,
        node.get(),
        0.2
    );

    std::cout << "Initializing configurator" << std::endl;

    auto configurator_ = std::make_shared<Configurator<rclcpp_lifecycle::LifecycleNode>>(
        node.get(),
        "mission_executor"
    );

    std::cout << "Finalizing initialization of tree provider" << std::endl;

    tree_provider_->FinalizeInitialization(executor);

    std::cout << "Finalizing initialization of mode provider" << std::endl;

    // mode_provider_->FinalizeInitialization(executor);

    std::cout << "Initializing mode node" << std::endl;

    auto mode_node = std::make_shared<rclcpp::Node>(
        "mode"
    );

    // std::cout << "Creating mode" << std::endl;

    // ManeuverMode::SharedPtr mode = std::make_shared<ManeuverMode>(
    //     *mode_node,
    //     "Test mode name",
    //     0.2,
    //     false,
    //     false
    // );

    // std::cout << "Registering mode" << std::endl;

    // if(!mode->doRegister())
    // {
    //     std::string fatal_msg = "ModeProvider::initializeModes(): Failed to register mode";

    //     RCLCPP_FATAL(node->get_logger(), fatal_msg.c_str());

    //     throw std::runtime_error(fatal_msg);
    // }

    for (mission_specification_entry_t entry : *mission_specification_) {

        std::cout << "Creating mode " << entry.key << std::endl;
    
        ManeuverMode::SharedPtr mode = std::make_shared<ManeuverMode>(
            *mode_node,
            entry.mode_name,
            0.2,
            false,
            false
        );

        std::cout << "Registering mode " << entry.key << std::endl;

        if(!mode->doRegister())
        {
            std::string fatal_msg = "ModeProvider::initializeModes(): Failed to register mode: " + entry.key;

            throw std::runtime_error(fatal_msg);
        }

        // modes_[entry.key] = mode;

    }

    // Works. Put all the functionality into a seperate node class which initializes everything in a simple way, create that class in the containing lifecycle node.


    // auto mission_executor = std::make_shared<iii_drone::mission::MissionExecutor>(
    //     node.get(),
    //     tf_buffer_,
    //     configurator_->GetParameter("mission_specification_file").as_string(),
    //     configurator_->GetParameter("dt").as_double()
    // );

    // mission_executor->FinalizeInitialization(executor);

    // mission_executor->Configure(configurator_);

    // std::cout << "Starting mission executor" << std::endl;

    // mission_executor->Start();

    std::cout << "volá" << std::endl;

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}