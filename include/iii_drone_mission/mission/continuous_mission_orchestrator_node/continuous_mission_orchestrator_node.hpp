#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <stdint.h>
#include <mutex>
#include <functional>
#include <memory>
#include <thread>
#include <climits>
#include <math.h>
#include <chrono>
#include <iostream>
#include <queue>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "iii_drone_interfaces/msg/control_state.hpp"
#include "iii_drone_interfaces/msg/powerline.hpp"
#include "iii_drone_interfaces/msg/charger_operating_mode.hpp"
#include "iii_drone_interfaces/msg/charger_status.hpp"
#include "iii_drone_interfaces/msg/gripper_status.hpp"
              
#include "iii_drone_interfaces/srv/gripper_command.hpp"
#include "iii_drone_interfaces/srv/initiate_charging.hpp"
#include "iii_drone_interfaces/srv/interrupt_charging.hpp"
#include "iii_drone_interfaces/srv/set_target_cable_id.hpp"
#include "iii_drone_interfaces/srv/prolong_charging.hpp"
              
#include "iii_drone_interfaces/action/fly_under_cable.hpp"
#include "iii_drone_interfaces/action/cable_landing.hpp"
#include "iii_drone_interfaces/action/disarm_on_cable.hpp"
#include "iii_drone_interfaces/action/arm_on_cable.hpp"
#include "iii_drone_interfaces/action/cable_takeoff.hpp"

#include "iii_drone_core/utils/math.hpp"
#include "iii_drone_core/utils/types.hpp"

/*****************************************************************************/
// Class
/*****************************************************************************/

namespace iii_drone {
namespace mission {
namespace continuous_mission_orchestrator_node {

    class ContinuousMissionOrchestrator : public rclcpp::Node {
    public:
        using FlyUnderCable = iii_drone_interfaces::action::FlyUnderCable;
        using GoalHandleFlyUnderCable = rclcpp_action::ClientGoalHandle<FlyUnderCable>;

        using CableLanding = iii_drone_interfaces::action::CableLanding;
        using GoalHandleCableLanding = rclcpp_action::ClientGoalHandle<CableLanding>;

        using DisarmOnCable = iii_drone_interfaces::action::DisarmOnCable;
        using GoalHandleDisarmOnCable = rclcpp_action::ClientGoalHandle<DisarmOnCable>;

        using ArmOnCable = iii_drone_interfaces::action::ArmOnCable;
        using GoalHandleArmOnCable = rclcpp_action::ClientGoalHandle<ArmOnCable>;

        using CableTakeoff = iii_drone_interfaces::action::CableTakeoff;
        using GoalHandleCableTakeoff = rclcpp_action::ClientGoalHandle<CableTakeoff>;

        ContinuousMissionOrchestrator(const std::string & node_name="continuous_mission_orchestrator", 
                const std::string & node_namespace="/mission/continuous_mission_orchestrator", 
                const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        // Fly under cable action client:
        rclcpp_action::Client<FlyUnderCable>::SharedPtr fly_under_cable_client_;
        void flyUnderCableGoalResponseCallback(const GoalHandleFlyUnderCable::SharedPtr & goal_handle);
        void flyUnderCableFeedbackCallback(GoalHandleFlyUnderCable::SharedPtr, const std::shared_ptr<const FlyUnderCable::Feedback> feedback);
        void flyUnderCableResultCallback(const GoalHandleFlyUnderCable::WrappedResult &result);

        bool fly_under_cable_response_received_;
        bool fly_under_cable_goal_accepted_;
        bool fly_under_cable_active_;
        bool fly_under_cable_success_;
        std::mutex fly_under_cable_mutex_;

        void startFlyUnderCable();
        bool flyUnderCableTerminated();
        bool flyUnderCableSucceeded();
        void cancelFlyUnderCable();

        // Cable landing action client:
        rclcpp_action::Client<CableLanding>::SharedPtr cable_landing_client_;
        void cableLandingGoalResponseCallback(const GoalHandleCableLanding::SharedPtr & goal_handle);
        void cableLandingFeedbackCallback(GoalHandleCableLanding::SharedPtr, const std::shared_ptr<const CableLanding::Feedback> feedback);
        void cableLandingResultCallback(const GoalHandleCableLanding::WrappedResult &result);

        bool cable_landing_response_received_;
        bool cable_landing_goal_accepted_;
        bool cable_landing_active_;
        bool cable_landing_success_;
        std::mutex cable_landing_mutex_;

        void startCableLanding();
        bool cableLandingTerminated();
        bool cableLandingSucceeded();
        void cancelCableLanding();

        int cable_landing_counter_;

        // Disarm on cable action client:
        rclcpp_action::Client<DisarmOnCable>::SharedPtr disarm_on_cable_client_;
        void disarmOnCableGoalResponseCallback(const GoalHandleDisarmOnCable::SharedPtr & goal_handle);
        void disarmOnCableFeedbackCallback(GoalHandleDisarmOnCable::SharedPtr, const std::shared_ptr<const DisarmOnCable::Feedback> feedback);
        void disarmOnCableResultCallback(const GoalHandleDisarmOnCable::WrappedResult &result);

        bool disarm_on_cable_response_received_;
        bool disarm_on_cable_goal_accepted_;
        bool disarm_on_cable_active_;
        bool disarm_on_cable_success_;
        std::mutex disarm_on_cable_mutex_;

        void startDisarmOnCable();
        bool disarmOnCableTerminated();
        bool disarmOnCableSucceeded();
        void cancelDisarmOnCable();

        // Arm on cable action client:
        rclcpp_action::Client<ArmOnCable>::SharedPtr arm_on_cable_client_;
        void armOnCableGoalResponseCallback(const GoalHandleArmOnCable::SharedPtr & goal_handle);
        void armOnCableFeedbackCallback(GoalHandleArmOnCable::SharedPtr, const std::shared_ptr<const ArmOnCable::Feedback> feedback);
        void armOnCableResultCallback(const GoalHandleArmOnCable::WrappedResult &result);

        bool arm_on_cable_response_received_;
        bool arm_on_cable_goal_accepted_;
        bool arm_on_cable_active_;
        bool arm_on_cable_success_;
        std::mutex arm_on_cable_mutex_;

        void startArmOnCable();
        bool armOnCableTerminated();
        bool armOnCableSucceeded();
        void cancelArmOnCable();

        // Cable takeoff action client:
        rclcpp_action::Client<CableTakeoff>::SharedPtr cable_takeoff_client_;
        void cableTakeoffGoalResponseCallback(const GoalHandleCableTakeoff::SharedPtr & goal_handle);
        void cableTakeoffFeedbackCallback(GoalHandleCableTakeoff::SharedPtr, const std::shared_ptr<const CableTakeoff::Feedback> feedback);
        void cableTakeoffResultCallback(const GoalHandleCableTakeoff::WrappedResult &result);

        bool cable_takeoff_response_received_;
        bool cable_takeoff_goal_accepted_;
        bool cable_takeoff_active_;
        bool cable_takeoff_success_;
        std::mutex cable_takeoff_mutex_;

        void startCableTakeoff();
        bool cableTakeoffTerminated();
        bool cableTakeoffSucceeded();
        void cancelCableTakeoff();

        // Gripper command client:
        rclcpp::Client<iii_drone_interfaces::srv::GripperCommand>::SharedPtr gripper_command_client_;
        void gripperCommandResponseCallback(rclcpp::Client<iii_drone_interfaces::srv::GripperCommand>::SharedFuture future);

        bool gripper_command_response_received_;
        bool gripper_command_active_;
        bool gripper_command_success_;
        bool last_gripper_command_close_ = false;
        std::mutex gripper_command_mutex_;

        void sendGripperCommand(bool close);
        bool gripperCommandTerminated();
        bool gripperCommandSucceeded();
        bool getLastGripperCommandClose();

        // Target cable service:
        rclcpp::Service<iii_drone_interfaces::srv::SetTargetCableId>::SharedPtr set_target_cable_id_srv_;
        void setTargetCableIdSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<iii_drone_interfaces::srv::SetTargetCableId::Request> request,
                const std::shared_ptr<iii_drone_interfaces::srv::SetTargetCableId::Response> response);
        int8_t target_cable_id_ = -1;
        bool target_cable_id_changed_ = false;
        std::mutex target_cable_id_mutex_;
        int8_t getTargetCableId();
        void setTargetCableId(int8_t cable_id);
        bool targetCableIdChanged();
        void setTargetCableIdChanged(bool changed);

        // Charger/gripper status:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_voltage_sub_;
        void batteryVoltageCallback(std_msgs::msg::Float32::SharedPtr msg);
        float battery_voltage_ = false;
        bool battery_voltage_low_ = false;
        bool battery_voltage_high_ = false;
        std::mutex battery_voltage_mutex_;
        float getBatteryVoltage();
        void setBatteryVoltage(float voltage);
        bool batteryVoltageLow();
        void setBatteryVoltageLow(bool low);
        bool batteryVoltageHigh();
        void setBatteryVoltageHigh(bool high);

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr charging_power_sub_;
        void chargingPowerCallback(std_msgs::msg::Float32::SharedPtr msg);
        float charging_power_ = false;
        bool charging_power_low_ = false;
        std::mutex charging_power_mutex_;
        float getChargingPower();
        bool chargingPowerLow();

        rclcpp::Subscription<iii_drone_interfaces::msg::ChargerOperatingMode>::SharedPtr charger_operating_mode_sub_;
        void chargerOperatingModeCallback(iii_drone_interfaces::msg::ChargerOperatingMode::SharedPtr msg);
        iii_drone_interfaces::msg::ChargerOperatingMode charger_operating_mode_;
        std::mutex charger_operating_mode_mutex_;
        iii_drone_interfaces::msg::ChargerOperatingMode getChargerOperatingMode();

        rclcpp::Subscription<iii_drone_interfaces::msg::ChargerStatus>::SharedPtr charger_status_sub_;
        void chargerStatusCallback(iii_drone_interfaces::msg::ChargerStatus::SharedPtr msg);
        iii_drone_interfaces::msg::ChargerStatus charger_status_;
        bool charger_status_disabled_ = false;
        bool charger_status_charging_ = false;
        bool charger_status_fully_charged_ = false;
        std::mutex charger_status_mutex_;
        iii_drone_interfaces::msg::ChargerStatus getChargerStatus();
        bool chargerStatusDisabled();
        void setChargerStatusDisabled(bool disabled);
        bool chargerStatusCharging();
        void setChargerStatusCharging(bool charging);
        bool chargerStatusFullyCharged();
        void setChargerStatusFullyCharged(bool fully_charged);

        rclcpp::Subscription<iii_drone_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub_;
        void gripperStatusCallback(iii_drone_interfaces::msg::GripperStatus::SharedPtr msg);
        iii_drone_interfaces::msg::GripperStatus gripper_status_;
        std::mutex gripper_status_mutex_;
        iii_drone_interfaces::msg::GripperStatus getGripperStatus();

        bool gripperIsClosed();
        bool gripperIsOpen();

        // Charging:
        rclcpp::Time charging_start_time_;
        std::mutex charging_start_time_mutex_;
        rclcpp::Time getChargingStartTime();

        rclcpp::Service<iii_drone_interfaces::srv::InitiateCharging>::SharedPtr initiate_charging_srv_server_;
        void initiateChargingSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<iii_drone_interfaces::srv::InitiateCharging::Request> request,
                const std::shared_ptr<iii_drone_interfaces::srv::InitiateCharging::Response> response);
        bool initiate_charging_srv_called_ = false;
        std::mutex initiate_charging_srv_called_mutex_;
        bool initiateChargingSrvCalled();
        void clearInitiateChargingSrvCalled();
        
        rclcpp::Service<iii_drone_interfaces::srv::InterruptCharging>::SharedPtr interrupt_charging_srv_server_;
        void interruptChargingSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<iii_drone_interfaces::srv::InterruptCharging::Request> request,
                const std::shared_ptr<iii_drone_interfaces::srv::InterruptCharging::Response> response);
        bool interrupt_charging_srv_called_ = false;
        std::mutex interrupt_charging_srv_called_mutex_;
        bool interruptChargingSrvCalled();
        void clearInterruptChargingSrvCalled();

        rclcpp::Service<iii_drone_interfaces::srv::ProlongCharging>::SharedPtr prolong_charging_srv_server_;
        void prolongChargingSrvCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<iii_drone_interfaces::srv::ProlongCharging::Request> request,
                const std::shared_ptr<iii_drone_interfaces::srv::ProlongCharging::Response> response);
        bool prolong_charging_until_interrupted_ = false;
        float prolong_charging_for_seconds_ = -1;
        float prolong_charging_until_new_battery_voltage_ = -1;
        std::mutex prolong_charging_mutex_;
        bool prolongChargingUntilInterrupted();
        float getProlongChargingForSeconds();
        float getProlongChargingUntilNewBatteryVoltage();
        void clearProlongCharging();

        void evaluateChargingStatus();

        bool start_charging_flag_{false};
        std::mutex start_charging_flag_mutex_;
        bool getStartChargingFlag();
        void setStartChargingFlag(bool flag);

        bool stop_charging_flag_{false};
        std::mutex stop_charging_flag_mutex_;
        bool getStopChargingFlag();
        void setStopChargingFlag(bool flag);

        // Control state:
        rclcpp::Subscription<iii_drone_interfaces::msg::ControlState>::SharedPtr control_state_sub_;
        void controlStateCallback(iii_drone_interfaces::msg::ControlState::SharedPtr msg);
        iii_drone_interfaces::msg::ControlState control_state_;
        std::mutex control_state_mutex_;
        iii_drone_interfaces::msg::ControlState getControlState();
        void setControlState(iii_drone_interfaces::msg::ControlState control_state);

        bool isInControllableState();
        bool isInControllableState(iii_drone_interfaces::msg::ControlState control_state);

        // tf:
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // State machine:
        typedef enum {
            manual,
            wait_for_target_line,
            flying_under_cable,
            inspecting,
            landing_on_cable,
            closing_gripper,
            disarming_on_cable,
            charging,
            arming_on_cable,
            opening_gripper,
            taking_off_from_cable,
            error
        } state_t;

        state_t state_ = manual;
        std::mutex state_mutex_;
        state_t getState();
        void setState(state_t state);

        rclcpp::TimerBase::SharedPtr state_machine_timer_;
        void stateMachineCallback();

        void stateMachineManual();
        void stateMachineWaitForTargetLine();
        void stateMachineFlyingUnderCable();
        void stateMachineInspecting();
        void stateMachineLandingOnCable();
        void stateMachineClosingGripper();
        void stateMachineDisarmingOnCable();
        void stateMachineCharging();
        void stateMachineArmingOnCable();
        void stateMachineOpeningGripper();
        void stateMachineTakingOffFromCable();
        void stateMachineError();

        rclcpp::Time action_start_time_;
        void startActionTimeoutCounter();
        bool actionTimeout();

        // Publishing:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        void publishCallback();

    };

} // namespace continuous_mission_orchestrator_node
} // namespace mission
} // namespace iii_drone

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char* argv[]);