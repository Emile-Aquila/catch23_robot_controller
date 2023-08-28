//
// Created by emile on 23/08/22.
//

#ifndef ROS2_WS_C620_TEST_HPP
#define ROS2_WS_C620_TEST_HPP


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


namespace arm_controller{
    class C620Test : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit C620Test(const rclcpp::NodeOptions & options);
        ~C620Test();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        using actuator_msg = actuator_msgs::msg::ActuatorMsg;

        void timer_callback();
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros;
        bool flag = false;
        bool flag_pre = false;
    };
}


#endif //ROS2_WS_C620_TEST_HPP
