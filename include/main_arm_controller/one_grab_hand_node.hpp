//
// Created by emile on 23/09/15.
//

#ifndef ROS2_WS_ONE_GRAB_HAND_NODE_HPP
#define ROS2_WS_ONE_GRAB_HAND_NODE_HPP


#include <functional>
#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <main_arm_controller/visibility.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <main_arm_controller/utils/system_classes.hpp>
#include <main_arm_controller/utils/util_functions.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>

enum class ONE_GRAB_STATE{
    ONE_GRAB_WAIT,  // 待機
    ONE_GRAB_TOWARD,  // 取りに行く
    ONE_GRAB_CATCH,  // 掴む
    ONE_GRAB_BACK,  // 戻る
    ONE_GRAB_RELEASE,  // 話す
    ONE_GRAB_FORWARD,  // 干渉しない位置へ移動
};

namespace arm_controller{
    class OneGrabHandNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit OneGrabHandNode(const rclcpp::NodeOptions & options);
        ~OneGrabHandNode();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        using actuator_msg = actuator_msgs::msg::ActuatorMsg;

        TimeCounter _time_counter;
        int ikko_servo_id = 0;
        bool is_moving = false;  // すでに動作中ならtrue

        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_b3m;
        rclcpp::TimerBase::SharedPtr timer;
//        rclcpp::Subscription<actuator_msgs::msg::C620Feedback>::SharedPtr theta_sub;

        void _b3m_init(uint8_t servo_id);
    };
}

#endif //ROS2_WS_ONE_GRAB_HAND_NODE_HPP
