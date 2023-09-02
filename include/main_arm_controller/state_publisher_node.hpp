//
// Created by emile on 23/09/02.
//

#ifndef ROS2_WS_STATE_PUBLISHER_NODE_HPP
#define ROS2_WS_STATE_PUBLISHER_NODE_HPP


#include <functional>
#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <main_arm_controller/visibility.hpp>
#include "std_msgs/msg/float32.hpp"
#include <actuator_msgs/msg/c620_feedback.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <main_arm_controller/robot_state.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>


namespace arm_controller{
    class StatePublisherNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit StatePublisherNode(const rclcpp::NodeOptions & options);
        ~StatePublisherNode();
    private:
        rclcpp::Subscription<actuator_msgs::msg::C620Feedback>::SharedPtr theta_sub, r_sub;
        rclcpp::Subscription<actuator_msgs::msg::ActuatorMsg>::SharedPtr theta_ref, r_ref;
        rclcpp::Publisher<catch23_robot_controller::msg::TipState>::SharedPtr pub_tip, pub_tip_tgt, pub_tip_ref;
        rclcpp::TimerBase::SharedPtr timer;
        ArmState arm_state_fb, arm_state_tgt, arm_state_ref;
    };
}


#endif //ROS2_WS_STATE_PUBLISHER_NODE_HPP
