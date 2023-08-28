//
// Created by emile on 23/08/22.
//

#include <functional>
#include <algorithm>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "main_arm_controller/c620_test.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>
#include <actuator_msgs/msg/device_info.hpp>

using namespace std::chrono_literals;
using kondo_msg = kondo_drivers::msg::B3mServoMsg;


namespace arm_controller{
    C620Test::C620Test(const rclcpp::NodeOptions & options)
            : Node("c620_test", options) {
        auto joy_callback = [this](const sensor_msgs::msg::Joy &msg) -> void {
            auto list_axes = msg.axes;
            std::vector<int> button_inputs = msg.buttons;  // target
            if(button_inputs[0]){
                if(!flag_pre){
                    flag = (flag+1)%2;
                }
            }
            flag_pre = button_inputs[0];
//            float stick_input = list_axes[2] * 0.2;  // target duty
//            float stick_input2 = list_axes[3] * 432.0;
//
//            RCLCPP_INFO(this->get_logger(), "stick input: %lf", stick_input);
//            RCLCPP_INFO(this->get_logger(), "stick input2: %lf", stick_input2);
//
//            std::for_each(button_inputs.begin(), button_inputs.begin()+2,[this](auto tmp){
//                RCLCPP_INFO(this->get_logger(), "button input: %d", tmp);
//            });
//
//            actuator_msg target_data;
//
//            target_data.device.node_type.node_type = actuator_msgs::msg::NodeType::NODE_C620;
//            target_data.device.node_id = 0;
//            target_data.device.device_num = 1;
//            target_data.target_value = stick_input2;
//            _pub_micro_ros->publish(target_data);

        };

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback); // joyのtopicを受け取る
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&C620Test::timer_callback, this));
    }

    void C620Test::timer_callback() {
        actuator_msg target_data;

        target_data.device.node_type.node_type = actuator_msgs::msg::NodeType::NODE_C620;
        target_data.device.node_id = 0;
        target_data.device.device_num = 1;
        if(flag){
            target_data.target_value = 3.141592f * 2.0f;
        }else{
            target_data.target_value = 0.0f;
        }
        _pub_micro_ros->publish(target_data);
    }

    C620Test::~C620Test(){}
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::C620Test)
