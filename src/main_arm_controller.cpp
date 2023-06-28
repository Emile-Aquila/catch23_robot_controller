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



namespace arm_controller{

    ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
    : Node("main_arm_controller_component", options) {
        using namespace std::chrono_literals;
        uint8_t servo_id = 1;
        auto topic_callback = [this, servo_id](const sensor_msgs::msg::Joy &msg) -> void {
            auto list_axes = msg.axes;
            std::vector<int> button_inputs = msg.buttons;  // target
            float stick_input = list_axes[0] * 0.2;  // target duty

            RCLCPP_INFO(this->get_logger(), "stick input: %lf", stick_input);
            std::for_each(button_inputs.begin(), button_inputs.begin()+2,[this](auto tmp){
                RCLCPP_INFO(this->get_logger(), "stick input: %d", tmp);
            });

            std_msgs::msg::Float32 target_duty;
            target_duty.data = stick_input;
            _pub_micro_ros->publish(target_duty);

            if(button_inputs[0] == 1){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(servo_id, 100.0f, 500));
            }else if(button_inputs[1] == 1){
                _pub_kondo->publish(this->_gen_b3m_set_pos_msg(servo_id, 0.0f, 500));
            }
        };

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>
                ("joy", 10, topic_callback); // joyのtopicを受け取る
        _pub_micro_ros = this->create_publisher<std_msgs::msg::Float32>("int32_subscriber", 10);
        _pub_kondo = this->create_publisher<kondo_msg>("kondo_b3m_topic", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&ArmControllerNode::timer_callback, this));
        rclcpp::sleep_for(1000ms);
        // init b3m
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 動作モードをfreeに
        rclcpp::sleep_for(100ms);
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 位置制御モードに
        rclcpp::sleep_for(100ms);
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x01, 0x29));
        rclcpp::sleep_for(100ms);
        // 軌道生成タイプ：Even (直線補間タイプの位置制御を指定)
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x00, 0x5C)); // PIDの設定を位置制御のプリセットに合わせる
        rclcpp::sleep_for(100ms);
        _pub_kondo->publish(_gen_b3m_write_msg(servo_id, 0x00, 0x28)); // 動作モードをNormalに
    }

    ArmControllerNode::~ArmControllerNode(){}

    void ArmControllerNode::timer_callback() {
        using namespace std::chrono_literals;
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
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::ArmControllerNode)
