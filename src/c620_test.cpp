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
#include <actuator_msgs/msg/actuator_msg.hpp>
#include <actuator_msgs/msg/device_info.hpp>
#include <main_arm_controller/utils/joystick_state.hpp>

using namespace std::chrono_literals;

const float min_r = 325.0f;
const float max_r = 975.0f;
const float min_theta = -M_PI;
const float max_theta = M_PI;


namespace arm_controller{
    C620Test::C620Test(const rclcpp::NodeOptions & options)
            : Node("c620_test", options) {
        target_r = min_r;
        target_theta = 0.0f;

        auto joy_callback = [this](const sensor_msgs::msg::Joy &msg) -> void {
            this->_joy_state.set(msg);

            if(this->_joy_state.get_button_1_indexed(1, true)){
                if(target_r == min_r){
                    RCLCPP_WARN(this->get_logger(), "[INFO] r -> 100.0");
                    target_r = min_r + 100.0f;
                }else{
                    RCLCPP_INFO(this->get_logger(), "[INFO] r -> 0.0");
                    target_r = min_r;
                }
            }

            if(this->_joy_state.get_button_1_indexed(2, true)){
                if(target_theta == 0.0f){
                    RCLCPP_WARN(this->get_logger(), "[INFO] theta -> pi");
                    target_theta = 0.0f + 3.141592f;
                }else{
                    RCLCPP_INFO(this->get_logger(), "[INFO] theta -> 0.0");
                    target_theta = 0.0f;
                }
            }
        };


        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_callback); // joyのtopicを受け取る
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 5);
        _pub_micro_ros_r = this->create_publisher<std_msgs::msg::Float32>("mros_input_r", 5);
        _pub_micro_ros_theta = this->create_publisher<std_msgs::msg::Float32>("mros_input_theta", 5);

        timer_ = this->create_wall_timer(100ms, std::bind(&C620Test::timer_callback, this));
    }

    void C620Test::timer_callback() {
        std_msgs::msg::Float32 target_data;

        target_data.data = target_r;
        _pub_micro_ros_r->publish(target_data);

        target_data.data = target_theta;
        _pub_micro_ros_theta->publish(target_data);
    }

    C620Test::~C620Test(){}
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::C620Test)
