//
// Created by emile on 23/06/09.
//

#include <functional>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "main_arm_controller/main_arm_controller.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>
#include <actuator_msgs/msg/device_info.hpp>
#include <rclcpp/qos.hpp>
#include <main_arm_controller/joystick_state.hpp>

using namespace std::chrono_literals;
using kondo_msg = kondo_drivers::msg::B3mServoMsg;

float clip_f(float value, float min_v, float max_v){
    return std::min(max_v, std::max(min_v, value));
}

void clip_arm_state(ArmState& arm_state){
    arm_state.r = clip_f(arm_state.r, -300.0f, 0.0f);
    arm_state.theta = clip_f(arm_state.theta, -M_PI/3.0f, M_PI/3.0f);
    arm_state.z = clip_f(arm_state.z, 0.0f, 0.0f);  // TODO: 実装
    arm_state.phi = clip_f(arm_state.phi, 0.0f, 0.0f);  // TODO: 実装
}


namespace arm_controller{
    ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
    : Node("main_arm_controller_component", options) {
        uint8_t ikko_servo_id = 1;
        uint8_t wrist_servo_id = 0;  // TODO: rosparam化

        auto joy_callback = [this, ikko_servo_id](const sensor_msgs::msg::Joy &msg) -> void {
            JoyStickState joy_state(msg);

            auto [d_theta, d_r] = joy_state.get_joystick_left_xy();
            auto [d_phi, d_z] = joy_state.get_joystick_right_xy();
            request_arm_state = request_arm_state + ArmState(d_r * 10.0f, d_theta * 0.05f, d_z, d_phi);
            clip_arm_state(request_arm_state);
            tip_state_tgt = arm_fk(request_arm_state);

            RCLCPP_INFO(this->get_logger(), "r,theta,z,phi: %lf, %lf, %lf, %lf",
                        request_arm_state.r, request_arm_state.theta, request_arm_state.z, request_arm_state.phi);
            RCLCPP_INFO(this->get_logger(), "x,y,z,theta: %lf, %lf, %lf, %lf",
                        tip_state_tgt.x, tip_state_tgt.y, tip_state_tgt.z, tip_state_tgt.theta);
            _send_request_arm_state(request_arm_state);

            if(joy_state.get_button_1_indexed(6)) {
                _pub_micro_ros->publish(_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 2, 0, true));
            }else if(joy_state.get_button_1_indexed(8)){
                _pub_micro_ros->publish(_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 2, 0, false));
            }

            // B3M
            if(joy_state.get_button_1_indexed(1)){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(ikko_servo_id, 66.5f, 500));
            }else if(joy_state.get_button_1_indexed(2)){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(ikko_servo_id, 0.0f, 500));
            }else if(joy_state.get_button_1_indexed(3)){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(ikko_servo_id, -66.5f, 500));
            }
        };

        auto joy_callback2 = [this, ikko_servo_id](const sensor_msgs::msg::Joy &msg) -> void {
            JoyStickState joy_state(msg);

            auto [d_x, d_y] = joy_state.get_joystick_left_xy();
            auto [d_theta, d_z] = joy_state.get_joystick_right_xy();
            tip_state_tgt = tip_state_tgt + TipState(d_x * 2.0f, d_y * 2.0f, d_z, d_theta);
            request_arm_state = arm_ik(tip_state_tgt);
            clip_arm_state(request_arm_state);
            tip_state_tgt = arm_fk(request_arm_state);

            RCLCPP_INFO(this->get_logger(), "r,theta,z,phi: %lf, %lf, %lf, %lf",
                        request_arm_state.r, request_arm_state.theta, request_arm_state.z, request_arm_state.phi);
            RCLCPP_INFO(this->get_logger(), "x,y,z,theta: %lf, %lf, %lf, %lf",
                        tip_state_tgt.x, tip_state_tgt.y, tip_state_tgt.z, tip_state_tgt.theta);
            _send_request_arm_state(request_arm_state);
        };

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 5, joy_callback); // joyのtopicを受け取る

        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 10);
        _pub_micro_ros_r = this->create_publisher<actuator_msg>("mros_input_r", 10);
        _pub_micro_ros_theta = this->create_publisher<actuator_msg>("mros_input_theta", 10);

        _pub_kondo = this->create_publisher<kondo_msg>("kondo_b3m_topic", 10);

        rclcpp::sleep_for(100ms);
        _b3m_init(wrist_servo_id);  // init b3m
        _b3m_init(ikko_servo_id);  // init b3m
    }

    ArmControllerNode::~ArmControllerNode(){}

    actuator_msgs::msg::ActuatorMsg ArmControllerNode::_gen_actuator_msg(uint8_t node_type, uint8_t node_id, uint8_t device_id, float target_value, bool air_target){
        actuator_msg target_data;
        target_data.device.node_type.node_type = node_type;
        target_data.device.node_id = node_id;
        target_data.device.device_num = device_id;
        target_data.target_value = target_value;
        target_data.air_target = air_target;
//        _pub_micro_ros->publish(target_data);
        return target_data;
    }


    kondo_msg ArmControllerNode::_gen_b3m_set_pos_msg(uint8_t servo_id, float target_pos, uint16_t move_time) {
        kondo_msg ans;
        ans.servo_id = servo_id;
        ans.command_type = kondo_msg::CMD_SET_POS_B3M;
        ans.cmd_set_pos.target_pos = target_pos;
        ans.cmd_set_pos.move_time = move_time;
        return ans;
    }

    kondo_msg ArmControllerNode::_gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address) {
        kondo_msg ans;
        ans.servo_id = servo_id;
        ans.command_type = kondo_msg::CMD_WRITE_B3M;
        ans.cmd_write.txdata = TxData;
        ans.cmd_write.address = address;
        return ans;
    }

    void ArmControllerNode::_b3m_init(uint8_t servo_id) {  // b3mのinit
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 動作モードをfreeに
        rclcpp::sleep_for(100ms);
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 位置制御モードに
        rclcpp::sleep_for(100ms);
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x01, 0x29)); // 軌道生成タイプ：Even (直線補間タイプの位置制御を指定)
        rclcpp::sleep_for(100ms);
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x00, 0x5C)); // PIDの設定を位置制御のプリセットに合わせる
        rclcpp::sleep_for(100ms);
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x00, 0x28)); // 動作モードをNormalに
        rclcpp::sleep_for(100ms);
    }

    void ArmControllerNode::_send_request_arm_state(const ArmState &req_arm_state) {
        uint8_t wrist_servo_id = 0;  // TODO: rosparam化
        _pub_micro_ros->publish(_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_MCMD3, 1, 0, req_arm_state.z));
        _pub_micro_ros_theta->publish(_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_C620, 0, 1, req_arm_state.theta));
        _pub_micro_ros_r->publish(_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_C620, 0, 2, req_arm_state.r));
        _pub_kondo->publish(_gen_b3m_set_pos_msg(wrist_servo_id, req_arm_state.phi, 0));
    }
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::ArmControllerNode)
