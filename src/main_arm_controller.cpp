//
// Created by emile on 23/06/09.
//

#include <functional>
#include <algorithm>
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

using namespace std::chrono_literals;
using kondo_msg = kondo_drivers::msg::B3mServoMsg;

float clip_f(float value, float min_v, float max_v){
    return std::min(max_v, std::max(min_v, value));
}

namespace arm_controller{
    ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
    : Node("main_arm_controller_component", options) {
        uint8_t wrist_servo_id = 0;
        uint8_t ikko_servo_id = 1;
        tgt_r = 0.0f;
        tgt_theta = 0.0f;
        tgt_z = 0.0f;

        auto joy_callback = [this, wrist_servo_id, ikko_servo_id](const sensor_msgs::msg::Joy &msg) -> void {
            auto list_axes = msg.axes;  // -1 ~ 1
            std::vector<int> button_inputs = msg.buttons;  // 0 or 1
            // buttonは0-indexになってる
            // axes[0]: left | lr
            // axes[1]: left | ud
            // axes[3]: right | lr
            // axes[4]: right | ud

            float delta_theta = list_axes[0];
            float delta_r = list_axes[1];
            float delta_hand = list_axes[3];  // 手首のサーボ
            float delta_z = list_axes[4];

            tgt_z = tgt_z + delta_z;
            tgt_theta = tgt_theta + delta_theta;
            tgt_r = tgt_r + delta_r;
            tgt_hand = tgt_hand + delta_hand;
            RCLCPP_INFO(this->get_logger(), "target theta: %lf", delta_theta);
            RCLCPP_INFO(this->get_logger(), "target r: %lf", delta_r);
            RCLCPP_INFO(this->get_logger(), "target z: %lf", delta_z);

            std::for_each(button_inputs.begin(), button_inputs.begin()+2,[this](auto tmp){
                RCLCPP_INFO(this->get_logger(), "button input: %d", tmp);
            });

            actuator_request_target(actuator_msgs::msg::NodeType::NODE_MCMD3, 1, 0, tgt_z);
            actuator_request_target(actuator_msgs::msg::NodeType::NODE_C620, 0, 1, tgt_theta);
            actuator_request_target(actuator_msgs::msg::NodeType::NODE_C620, 0, 2, tgt_r);
            if(button_inputs[0] == 6) {
                actuator_request_target(actuator_msgs::msg::NodeType::NODE_AIR, 2, 0, true);
            }else if(button_inputs[0] == 8){
                actuator_request_target(actuator_msgs::msg::NodeType::NODE_AIR, 2, 0, false);
            }

            // B3M
            _pub_kondo->publish(this->_gen_b3m_set_pos_msg(wrist_servo_id, tgt_hand, 0));
            if(button_inputs[0] == 1){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(ikko_servo_id, 66.5f, 500));
            }else if(button_inputs[1] == 1){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(ikko_servo_id, 0.0f, 500));
            }else if(button_inputs[2] == 1){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(ikko_servo_id, -66.5f, 500));
            }
        };

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>
                ("joy", 10, joy_callback); // joyのtopicを受け取る
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 10);
        _pub_kondo = this->create_publisher<kondo_msg>("kondo_b3m_topic", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ArmControllerNode::timer_callback, this));

        rclcpp::sleep_for(1000ms);
        _b3m_init(wrist_servo_id);  // init b3m
        _b3m_init(ikko_servo_id);  // init b3m
    }

    ArmControllerNode::~ArmControllerNode(){}

    void ArmControllerNode::timer_callback() {
        using namespace std::chrono_literals;
    }

    void ArmControllerNode::actuator_request_target(uint8_t node_type, uint8_t node_id, uint8_t device_id, float target_value){
        actuator_msg target_data;
        target_data.device.node_type.node_type = node_type;
        target_data.device.node_id = node_id;
        target_data.device.device_num = device_id;
        target_data.target_value = target_value;
        _pub_micro_ros->publish(target_data);
    }

    void ArmControllerNode::actuator_request_target(uint8_t node_type, uint8_t node_id, uint8_t device_id, bool air_target){
        actuator_msg target_data;
        target_data.device.node_type.node_type = node_type;
        target_data.device.node_id = node_id;
        target_data.device.device_num = device_id;
        target_data.air_target = air_target;
        _pub_micro_ros->publish(target_data);
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
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::ArmControllerNode)
