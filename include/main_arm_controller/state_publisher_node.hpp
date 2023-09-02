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
#include <actuator_msgs/msg/c620_feedback.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <main_arm_controller/robot_state.hpp>


namespace state_publisher{
    class StatePublisherNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit StatePublisherNode(const rclcpp::NodeOptions & options);
        ~StatePublisherNode();
    private:
        rclcpp::Subscription<actuator_msgs::msg::C620Feedback>::SharedPtr theta_sub, r_sub;
        rclcpp::Publisher<catch23_robot_controller::msg::TipState>::SharedPtr pub_tip, pub_tip_tgt;
        rclcpp::TimerBase::SharedPtr timer;
        ArmState arm_state_fb, arm_state_tgt;
    };
}


#endif //ROS2_WS_STATE_PUBLISHER_NODE_HPP
