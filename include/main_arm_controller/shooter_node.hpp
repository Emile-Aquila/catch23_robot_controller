//
// Created by emile on 23/09/16.
//

#ifndef ROS2_WS_SHOOTER_NODE_HPP
#define ROS2_WS_SHOOTER_NODE_HPP


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
#include <actuator_msgs/msg/actuator_feedback.hpp>
#include <catch23_robot_controller/msg/shooter_state.hpp>



enum class ShooterState{
    POS_0,  // 初期位置?  // TODO: 仕様確定
    IS_MOVING, // 動作中
    POS1_UP,
    POS1_DOWN,
    POS2_UP,
    POS2_DOWN,
    POS3_UP,
    POS3_DOWN,
};

namespace arm_controller{
    class ShooterNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ShooterNode(const rclcpp::NodeOptions & options);
        ~ShooterNode();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        using actuator_msg = actuator_msgs::msg::ActuatorMsg;
        using actuator_fb_msg = actuator_msgs::msg::ActuatorFeedback;
        using shooter_msg = catch23_robot_controller::msg::ShooterState;

        const uint8_t node_id = 2;
        const uint8_t device_num = 0;
        const int _up_servo_id = 2;
        const float _angle_range = (float)rad_to_deg(120.0/22.5)/2.0f;
        const uint64_t wait_time_for_servo = 1000;


        void sub_callback(const shooter_msg& msg);
        void sub_callback_feedback(const actuator_fb_msg& msg);
        void timer_callback_movement();
        void timer_callback_send_state();
        void change_shooter_state(ShooterState next_state);
        void _b3m_init(uint8_t servo_id);
        void send_servo(bool is_up);

        ShooterState _shooter_state_now = ShooterState::POS_0;
        ShooterState _shooter_state_pre = ShooterState::POS_0;  // 一つ前の状態
        ShooterState _reqested_shooter_state = ShooterState::POS_0;  // 一つ前の状態
        TimeCounter _time_counter;
        bool _is_moving = false;  // すでに動作中ならtrue
        double _feedback_pos = 0.0;

        rclcpp::Subscription<shooter_msg>::SharedPtr _sub_request;
        rclcpp::Subscription<actuator_fb_msg>::SharedPtr _sub_feedback;
        rclcpp::Publisher<shooter_msg>::SharedPtr _pub_shooter_msg;
        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_b3m;
        rclcpp::TimerBase::SharedPtr _timer_movement, _timer_pub_state;
    };
}

#endif //ROS2_WS_SHOOTER_NODE_HPP
