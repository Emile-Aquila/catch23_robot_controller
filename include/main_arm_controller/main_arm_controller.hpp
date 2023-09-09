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
#include <kondo_drivers/srv/kondo_b3m_srv.hpp>
#include <main_arm_controller/joystick_state.hpp>
#include <main_arm_controller/trajectory/spline_utils.hpp>


namespace arm_controller{
    class ArmControllerNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ArmControllerNode(const rclcpp::NodeOptions & options);
        ~ArmControllerNode();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        using kondo_srv = kondo_drivers::srv::KondoB3mSrv;
        using actuator_msg = actuator_msgs::msg::ActuatorMsg;

        actuator_msg _gen_actuator_msg(uint8_t node_type, uint8_t node_id, uint8_t device_id, float target_value, bool air_target=false);
        kondo_msg _gen_b3m_set_pos_msg(uint8_t servo_id, float target_pos, uint16_t move_time=0);
        kondo_msg _gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address);
        void _send_request_arm_state(const ArmState& req_arm_state);
        void _b3m_init(uint8_t servo_id);
        void _request_hand_open_close(bool hand_close);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_micro_ros_r, _pub_micro_ros_theta;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_b3m;
        rclcpp::Client<kondo_srv>::SharedPtr _b3m_client;

        TipState tip_state_tgt;  // main armの目標位置
        ArmState request_arm_state;  // 送信するarmのstate
        JoyStickState joy_state;
        bool _hand_is_open;  // handの状態

        const TipState _tip_state_origin = TipState(325.0f, 0.0f, 0.0f, 0.0f);  // 初期位置
    };
}

#endif //ROS2_WS_MAIN_ARM_CONTROLLER_HPP
