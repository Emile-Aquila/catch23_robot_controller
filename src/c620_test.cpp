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
                if(!flag_r_pre){
                    flag_r = (flag_r+1)%2;
                }
            }
            flag_r_pre = button_inputs[0];

            if(button_inputs[1]){
                if(!flag_theta_pre){
                    flag_theta = (flag_theta + 1)%2;
                }
            }
            flag_theta_pre = button_inputs[1];
        };


        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback); // joyのtopicを受け取る
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 10);
        _pub_micro_ros_r = this->create_publisher<actuator_msg>("mros_input_r", 10);
        _pub_micro_ros_theta = this->create_publisher<actuator_msg>("mros_input_theta", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&C620Test::timer_callback, this));
    }

    void C620Test::timer_callback() {
        actuator_msg target_data;

        target_data.device.node_type.node_type = actuator_msgs::msg::NodeType::NODE_C620;
        target_data.device.node_id = 0;
        target_data.device.device_num = 1;
        if(flag_r){
            target_data.target_value = 3.141592f * 2.0f;
        }else{
            target_data.target_value = 0.0f;
        }
        _pub_micro_ros_r->publish(target_data);

        target_data.device.device_num = 2;
        if(flag_theta){
            target_data.target_value = 100.0f;
        }else{
            target_data.target_value = 0.0f;
        }
        _pub_micro_ros_theta->publish(target_data);
    }

    C620Test::~C620Test(){}
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::C620Test)
