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
#include <catch23_robot_controller/msg/one_hand_request.hpp>


enum class OneGrabState{
    ONE_GRAB_WAIT,  // 待機
    ONE_GRAB_TOWARD,  // 取りに行く
    ONE_GRAB_CATCH,  // 掴む
    ONE_GRAB_BACK,  // 戻る
    ONE_GRAB_RELEASE,  // 話す
    ONE_GRAB_ESCAPE,  // 干渉しない位置へ移動
    ONE_GRAB_STOP,  // 最終位置で止まって待機
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
        using OneHandRequest = catch23_robot_controller::msg::OneHandRequest;


        void sub_callback(const catch23_robot_controller::msg::OneHandRequest& msg);
        void timer_callback();
        void change_grab_state(OneGrabState next_state);
        void request_air(bool is_close);
        void _b3m_init(uint8_t servo_id);


        TimeCounter _time_counter;
        OneGrabState _grab_state = OneGrabState::ONE_GRAB_WAIT;
        bool _is_moving = false;  // すでに動作中ならtrue
        int _ikko_servo_id = 0;

        rclcpp::Subscription<catch23_robot_controller::msg::OneHandRequest>::SharedPtr _sub_request;
        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_b3m;
        rclcpp::TimerBase::SharedPtr _timer;
    };
}

#endif //ROS2_WS_ONE_GRAB_HAND_NODE_HPP
