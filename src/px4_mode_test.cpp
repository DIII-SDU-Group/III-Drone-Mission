#include <rclcpp/rclcpp.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
// #include <px4_ros2/common/setpoint_base.hpp>

#include <iii_drone_core/control/reference.hpp>
// #include <iii_drone_core/adapters/px4/trajectory_setpoint_adapter.hpp>
#include <iii_drone_mission/px4/setpoints/trajectory_setpoint.hpp>

using namespace iii_drone::px4;

// class TrajectorySetpoint : public px4_ros2::SetpointBase {
// public:
//     explicit TrajectorySetpoint(px4_ros2::Context & context) : px4_ros2::SetpointBase(context), 

//         node_(context.node()) {

//         trajectory_setpoint_pub_ = node_.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
//             "/fmu/in/trajectory_setpoint", 
//             1
//         );

//     }

//     ~TrajectorySetpoint() override = default;

//     Configuration getConfiguration() override {
//         px4_ros2::SetpointBase::Configuration config{};

//         config.rates_enabled = true;
//         config.attitude_enabled = true;
//         config.acceleration_enabled = true;
//         config.velocity_enabled = true;
//         config.position_enabled = true;
//         config.altitude_enabled = true;
//         config.climb_rate_enabled = true;

//         return config;
//     }

//     void update(const iii_drone::control::Reference & reference) {

//         onUpdate();

//         iii_drone::adapters::px4::TrajectorySetpointAdapter adapter(reference);

//         px4_msgs::msg::TrajectorySetpoint msg = adapter.ToMsg();

//         trajectory_setpoint_pub_->publish(adapter.ToMsg());

//     }

// private:
//     rclcpp::Node & node_;
//     rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;


// };

class TestMode : public px4_ros2::ModeBase
{
public:
    explicit TestMode(rclcpp::Node & node) : px4_ros2::ModeBase(
        node,
        px4_ros2::ModeBase::Settings(
            "Test mode",
            true
        )
    )
    {
        RCLCPP_INFO(node.get_logger(), "TestMode::TestMode(): Initializing.");

        traj_setpoint_ = std::make_shared<TrajectorySetpoint>(*this);
        // rates_setpoint_ = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    }

    void onActivate() override {
        RCLCPP_INFO(node().get_logger(), "TestMode::onActivate(): Activating mode");

        start_time_ = rclcpp::Clock().now();

        completed_ = false;

    }

    void onDeactivate() override {
        RCLCPP_INFO(node().get_logger(), "TestMode::onDeactivate(): Deactivating mode");

        completed_ = true;

    }

    void updateSetpoint(float dt) override {

        if (completed_) {
            return;
        }

        RCLCPP_INFO_THROTTLE(
            node().get_logger(), 
            *node().get_clock(),
            1000,
            "TestMode::updateSetpoint(): Updating setpoint"
        );

        iii_drone::control::Reference reference(
            {NAN, NAN, NAN},
            NAN,
            {0,0,5},
            0,
            {0,0,5},
            0
        );

        traj_setpoint_->update(reference);

        // rates_setpoint_->update(
        //     Eigen::Vector3f(0,0,0),
        //     Eigen::Vector3f(0,0,-1)
        // );

        if ((rclcpp::Clock().now() - start_time_).seconds() > 10) {
            RCLCPP_INFO(node().get_logger(), "TestMode::updateSetpoint(): Deactivating mode");
            completed(px4_ros2::Result::Success);
        }

    }

private:
    std::shared_ptr<TrajectorySetpoint> traj_setpoint_;
    // std::shared_ptr<px4_ros2::RatesSetpointType> rates_setpoint_;

    rclcpp::Time start_time_;

    bool completed_;

};

class TestModeExecutor : public px4_ros2::ModeExecutorBase {
public:
    TestModeExecutor(
        px4_ros2::ModeBase & owned_mode,
        rclcpp::Node & node
    ) : px4_ros2::ModeExecutorBase(
        node,
        px4_ros2::ModeExecutorBase::Settings{.activation=px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways},
        owned_mode
    ) {
        RCLCPP_INFO(node.get_logger(), "TestModeExecutor::TestModeExecutor(): Initializing.");
    }

    void onActivate() override {
        RCLCPP_INFO(node().get_logger(), "TestModeExecutor::onActivate(): Activating mode executor");

        RCLCPP_INFO(
            node().get_logger(), 
            "ModeExecutorBase::onActivate(): Arming."
        );

        arm(
            [this](px4_ros2::Result result) {

                if (result != px4_ros2::Result::Success) {

                    RCLCPP_ERROR(node().get_logger(), "TestModeExecutor::onActivate(): Arming failed, deactivating mode executor");

                    return;

                }

                scheduleMode(
                    ownedMode().id(),
                    [this](px4_ros2::Result result) {
                        RCLCPP_INFO(node().get_logger(), "TestModeExecutor::onActivate(): Mode completed");
                        scheduleMode(
                            2,
                            [this](px4_ros2::Result result) {
                                RCLCPP_INFO(node().get_logger(), "TestModeExecutor::onActivate(): Mode completed");
                            }
                        );
                    }
                );

            }
        );
    }

    void onDeactivate(DeactivateReason reason) override {
        RCLCPP_INFO(node().get_logger(), "TestModeExecutor::onDeactivate(): Deactivating mode executor");
    }


};

int main(int argc, char * argv[])
{
    std::cout << "yalla" << std::endl;

    rclcpp::init(argc, argv);

    auto mode_node = std::make_shared<rclcpp::Node>(
        "mode_node"
    );

    auto mode = std::make_shared<TestMode>(
        *mode_node
    );

    auto mode_executor = std::make_shared<TestModeExecutor>(
        *mode,
        *mode_node
    );

    if (!mode_executor->doRegister()) {
        std::cout << "Failed to register mode executor" << std::endl;
        return 1;
    }

    // if (!mode->doRegister()) {
    //     std::cout << "Failed to register mode executor" << std::endl;
    //     return 1;
    // }

    rclcpp::spin(mode_node);

    rclcpp::shutdown();
    return 0;
}