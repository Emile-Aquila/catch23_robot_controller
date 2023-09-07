//
// Created by emile on 23/06/09.
//

#ifndef ROS2_WS_MAIN_ARM_CONTROLLER_HPP
#define ROS2_WS_MAIN_ARM_CONTROLLER_HPP

#include <functional>
#include <algorithm>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "main_arm_controller/visibility.hpp"
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>
#include <main_arm_controller/robot_state.hpp>


namespace arm_controller{
    class ArmControllerNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ArmControllerNode(const rclcpp::NodeOptions & options);
        ~ArmControllerNode();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        using actuator_msg = actuator_msgs::msg::ActuatorMsg;

        actuator_msg _gen_actuator_msg(uint8_t node_type, uint8_t node_id, uint8_t device_id, float target_value, bool air_target=false);
        kondo_msg _gen_b3m_set_pos_msg(uint8_t servo_id, float target_pos, uint16_t move_time=0);
        kondo_msg _gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address);
        void _send_request_arm_state(const ArmState& req_arm_state);

        TipState tip_state_tgt;  // main armの目標位置
        ArmState request_arm_state;  // 送信するarmのstate
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros;
//        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros_r, _pub_micro_ros_theta;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_micro_ros_r, _pub_micro_ros_theta;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_kondo;
        void _b3m_init(uint8_t servo_id);
    };
}

#endif //ROS2_WS_MAIN_ARM_CONTROLLER_HPP
