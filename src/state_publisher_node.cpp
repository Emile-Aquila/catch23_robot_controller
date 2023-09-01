//
// Created by emile on 23/09/02.
//

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <main_arm_controller/state_publisher_node.hpp>
#include <actuator_msgs/msg/c620_feedback.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>


using namespace std::chrono_literals;

namespace state_publisher {
    StatePublisherNode::StatePublisherNode(const rclcpp::NodeOptions &options):Node("state_publisher", options){
        auto theta_callback = [this](const actuator_msgs::msg::C620Feedback &msg) -> void {
            this->arm_state_fb.theta = msg.position;
            this->arm_state_tgt.theta = msg.target_value;
        };

        auto r_callback = [this](const actuator_msgs::msg::C620Feedback &msg) -> void {
            this->arm_state_fb.r = msg.position;
            this->arm_state_tgt.r = msg.target_value;
        };

        theta_sub = this->create_subscription<actuator_msgs::msg::C620Feedback>("c620_theta", 5, theta_callback);
        r_sub = this->create_subscription<actuator_msgs::msg::C620Feedback>("c620_r", 5, r_callback);
        pub_tip = this->create_publisher<catch23_robot_controller::msg::TipState>("tip_state", 10);
        pub_tip_tgt = this->create_publisher<catch23_robot_controller::msg::TipState>("tip_state_tgt", 10);

        auto timer_callback = [this](){
            auto tip_data = _convert_tip_state(arm_fk(this->arm_state_fb));
            auto tip_data_tgt = _convert_tip_state(arm_fk(this->arm_state_tgt));
            this->pub_tip->publish(tip_data);
            this->pub_tip_tgt->publish(tip_data_tgt);
        };
        timer = this->create_wall_timer(30ms, timer_callback);
    }

    catch23_robot_controller::msg::TipState StatePublisherNode::_convert_tip_state(const TipState &tip_state) {
        catch23_robot_controller::msg::TipState tip_data;
        tip_data.x = tip_state.x;
        tip_data.y = tip_state.y;
        tip_data.z = tip_state.z;
        tip_data.theta = tip_state.theta;
        return tip_data;
    }
};



RCLCPP_COMPONENTS_REGISTER_NODE(state_publisher::StatePublisherNode)
