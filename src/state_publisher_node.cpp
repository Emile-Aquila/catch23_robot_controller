//
// Created by emile on 23/09/02.
//

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <main_arm_controller/state_publisher_node.hpp>
#include <actuator_msgs/msg/c620_feedback.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <actuator_msgs/msg/actuator_feedback.hpp>
#include <actuator_msgs/msg/node_type.hpp>


using namespace std::chrono_literals;

namespace arm_controller {
    StatePublisherNode::StatePublisherNode(const rclcpp::NodeOptions &options)
        :Node("state_publisher_component", options){
        auto theta_callback = [this](const actuator_msgs::msg::C620Feedback &msg) -> void {
            this->arm_state_fb.theta = msg.position;
            this->arm_state_tgt.theta = msg.target_value;
        };

        auto r_callback = [this](const actuator_msgs::msg::C620Feedback &msg) -> void {
            this->arm_state_fb.r = msg.position;
            this->arm_state_tgt.r = msg.target_value;
        };

        auto mcmd_callback = [this](const actuator_msgs::msg::ActuatorFeedback& msg) -> void {
            if (msg.device.node_type.node_type == actuator_msgs::msg::NodeType::NODE_MCMD3
                    && msg.device.device_num == 0 && msg.device.node_id == 1) {  // z
                this->arm_state_fb.z = msg.fb_data;
            }
            if (msg.device.node_type.node_type == actuator_msgs::msg::NodeType::NODE_MCMD4
                    && msg.device.device_num == 0 && msg.device.node_id == 2) {  // TODO: シューター
//                this->arm_state_fb.z = msg.fb_data;
            }
        };
        auto ref_r_callback = [this](const std_msgs::msg::Float32 &msg) -> void {
            this->arm_state_ref.r = msg.data;
        };

        auto ref_theta_callback = [this](const std_msgs::msg::Float32 &msg) -> void {
            this->arm_state_ref.theta = msg.data;
        };


        theta_sub = this->create_subscription<actuator_msgs::msg::C620Feedback>("c620_theta", 5, theta_callback);
        r_sub = this->create_subscription<actuator_msgs::msg::C620Feedback>("c620_r", 5, r_callback);
        mcmd_sub = this->create_subscription<actuator_msgs::msg::ActuatorFeedback>("mros_output_mcmd", 10, mcmd_callback);

        r_ref = this->create_subscription<std_msgs::msg::Float32>("mros_input_r", 5, ref_r_callback);
        theta_ref = this->create_subscription<std_msgs::msg::Float32>("mros_input_theta", 5, ref_theta_callback);

        pub_tip = this->create_publisher<catch23_robot_controller::msg::TipState>("tip_state", 10);
        pub_tip_tgt = this->create_publisher<catch23_robot_controller::msg::TipState>("tip_state_tgt", 10);
        pub_tip_ref = this->create_publisher<catch23_robot_controller::msg::TipState>("tip_state_ref", 10);

        auto timer_callback = [this](){
            catch23_robot_controller::msg::TipState tip_data = convert_tip_state(arm_fk(this->arm_state_fb));
            catch23_robot_controller::msg::TipState tip_data_tgt = convert_tip_state(arm_fk(this->arm_state_tgt));
            catch23_robot_controller::msg::TipState tip_data_ref = convert_tip_state(arm_fk(this->arm_state_ref));

            this->pub_tip->publish(tip_data);
            this->pub_tip_tgt->publish(tip_data_tgt);
            this->pub_tip_ref->publish(tip_data_ref);
        };
        timer = this->create_wall_timer(40ms, timer_callback);
    }

    StatePublisherNode::~StatePublisherNode(){}
};



RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::StatePublisherNode)
