//
// Created by emile on 23/09/15.
//

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <main_arm_controller/one_grab_hand_node.hpp>
#include <actuator_msgs/msg/node_type.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace arm_controller {
    OneGrabHandNode::OneGrabHandNode(const rclcpp::NodeOptions &options)
            :Node("one_grab_hand_component", options){
        _grab_state = ShooterState::ONE_GRAB_WAIT;

        auto bind_callback = std::bind(&OneGrabHandNode::sub_callback, this, _1);
        _sub_request = this->create_subscription<OneHandRequest>("one_hand_request", 10, bind_callback);
        _pub_b3m = this->create_publisher<kondo_msg>("b3m_topic", 10);
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 5);
        _timer = this->create_wall_timer(100ms, std::bind(&OneGrabHandNode::timer_callback, this));

        _b3m_init(_ikko_servo_id);  // init b3m
        _pub_b3m->publish(gen_b3m_set_pos_msg(_ikko_servo_id, 0.0, 0));
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

    void OneGrabHandNode::timer_callback() {
        const float angle_forward = 66.5; // (deg)
        const float angle_back = -66.5; // (deg)
        const uint64_t servo_move_time_half = 600;
        const uint64_t air_move_time = 800;

        if(this->_grab_state == ShooterState::ONE_GRAB_WAIT || this->_grab_state == ShooterState::ONE_GRAB_ESCAPE){
            this->_time_counter.disable();
            if(this->_grab_state == ShooterState::ONE_GRAB_WAIT)this->_is_moving = false;
        }else{
            if(!(this->_time_counter.is_enable()))this->_time_counter.enable();  // counter起動
            this->_time_counter.count(100);  // count
            this->_is_moving = true;
        }

        switch (this->_grab_state) {  // 動作
            case ShooterState::ONE_GRAB_WAIT:
                // no action
                break;
            case ShooterState::ONE_GRAB_TOWARD:
                _pub_b3m->publish(gen_b3m_set_pos_msg(_ikko_servo_id, angle_forward, 0));
                if(this->_time_counter.check_time(servo_move_time_half))change_grab_state(ShooterState::ONE_GRAB_CATCH);
                break;
            case ShooterState::ONE_GRAB_CATCH:
                _pub_b3m->publish(gen_b3m_set_pos_msg(_ikko_servo_id, angle_forward, 0));
                request_air(true);
                if(this->_time_counter.check_time(air_move_time))change_grab_state(ShooterState::ONE_GRAB_BACK);
                break;
            case ShooterState::ONE_GRAB_BACK:
                _pub_b3m->publish(gen_b3m_set_pos_msg(_ikko_servo_id, angle_back, 0));
                if(this->_time_counter.check_time(servo_move_time_half*2))change_grab_state(ShooterState::ONE_GRAB_RELEASE);
                break;
            case ShooterState::ONE_GRAB_RELEASE:
                _pub_b3m->publish(gen_b3m_set_pos_msg(_ikko_servo_id, angle_back, 0));
                if(this->_time_counter.check_time(air_move_time))change_grab_state(ShooterState::ONE_GRAB_ESCAPE);
                request_air(false);
                break;
            case ShooterState::ONE_GRAB_ESCAPE:
                _pub_b3m->publish(gen_b3m_set_pos_msg(_ikko_servo_id, angle_forward, 0));
                if(this->_time_counter.check_time(servo_move_time_half*2))change_grab_state(ShooterState::ONE_GRAB_STOP);
                break;
            case ShooterState::ONE_GRAB_STOP:
                // no action
                break;
        }
    }

    void OneGrabHandNode::change_grab_state(ShooterState next_state) {
        if(_grab_state != next_state){
            this->_time_counter.disable();
        }
        if(next_state == ShooterState::ONE_GRAB_ESCAPE){
            RCLCPP_INFO(this->get_logger(), "[INFO] end one grab hand motion");
        }
        _grab_state = next_state;
    }

    void OneGrabHandNode::request_air(bool is_close){
        uint8_t node_id = 1;
        uint8_t device_id = 0;
        _pub_micro_ros->publish(gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, node_id, device_id, 0.0f, is_close));
    }

    void OneGrabHandNode::sub_callback(const catch23_robot_controller::msg::OneHandRequest &msg) {
        if(msg.request_type == OneHandRequest::REQUEST_WAIT){
            change_grab_state(ShooterState::ONE_GRAB_WAIT);

        }else if(msg.request_type ==OneHandRequest::REQUEST_START){
            if(!_is_moving){
                change_grab_state(ShooterState::ONE_GRAB_TOWARD);
                RCLCPP_INFO(this->get_logger(), "[INFO] start one grab hand!");
            }
        }else if(msg.request_type == OneHandRequest::REQUEST_FORCE_START){
            change_grab_state(ShooterState::ONE_GRAB_TOWARD);
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::OneGrabHandNode)
