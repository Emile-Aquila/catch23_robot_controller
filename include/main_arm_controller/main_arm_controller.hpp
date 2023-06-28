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

using kondo_msg = kondo_drivers::msg::B3mServoMsg;


namespace arm_controller{
    class ArmControllerNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ArmControllerNode(const rclcpp::NodeOptions & options);
        ~ArmControllerNode();
    private:
        void timer_callback();
        kondo_msg _gen_b3m_set_pos_msg(uint8_t servo_id, float target_pos, uint16_t move_time=0);
        kondo_msg _gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address);

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_micro_ros;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_kondo;
    };
}

#endif //ROS2_WS_MAIN_ARM_CONTROLLER_HPP
