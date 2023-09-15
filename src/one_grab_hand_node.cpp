//
// Created by emile on 23/09/15.
//

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <main_arm_controller/one_grab_hand_node.hpp>
#include <actuator_msgs/msg/node_type.hpp>


using namespace std::chrono_literals;

namespace arm_controller {
    OneGrabHandNode::OneGrabHandNode(const rclcpp::NodeOptions &options)
        :Node("one_grab_hand_component", options){

//        auto ref_r_callback = [this](const std_msgs::msg::Float32 &msg) -> void {
//            this->arm_state_ref.r = msg.data;
//        };
//        theta_sub = this->create_subscription<actuator_msgs::msg::C620Feedback>("c620_theta", 5, theta_callback);



        _pub_b3m = this->create_publisher<kondo_msg>("b3m_topic", 10);
        auto timer_callback = [this](){};
        timer = this->create_wall_timer(40ms, timer_callback);

//        _pub_b3m->publish(gen_b3m_set_pos_msg(hand_interval_id, target_pos, 0));
        rclcpp::sleep_for(100ms);
        _b3m_init(ikko_servo_id);  // init b3m
    }

    OneGrabHandNode::~OneGrabHandNode(){}

    void OneGrabHandNode::_b3m_init(uint8_t servo_id) {  // b3mのinit
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 動作モードをfreeに
        rclcpp::sleep_for(50ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 位置制御モードに
        rclcpp::sleep_for(50ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x00, 0x29)); // 軌道生成タイプ：Normal (最速で回転)
        rclcpp::sleep_for(50ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x00, 0x5C)); // PIDの設定を位置制御のプリセットに合わせる
        rclcpp::sleep_for(50ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x00, 0x28)); // 動作モードをNormalに
        rclcpp::sleep_for(50ms);
    }
};



RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::OneGrabHandNode)
