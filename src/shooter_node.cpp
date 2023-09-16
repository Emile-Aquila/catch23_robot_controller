//
// Created by emile on 23/09/16.
//

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <main_arm_controller/shooter_node.hpp>
#include <actuator_msgs/msg/node_type.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using shooter_msg = catch23_robot_controller::msg::ShooterState;


ShooterState convert(shooter_msg msg){
    switch (msg.state) {
        case shooter_msg::SHOOTER_MOVING:
            return ShooterState::IS_MOVING;
            break;
        case shooter_msg::SHOOTER_POS0:
            return ShooterState::POS_0;
            break;
        case shooter_msg::SHOOTER_POS1_UP:
            return ShooterState::POS1_UP;
            break;
        case shooter_msg::SHOOTER_POS1_DOWN:
            return ShooterState::POS1_DOWN;
            break;
        case shooter_msg::SHOOTER_POS2_UP:
            return ShooterState::POS2_UP;
            break;
        case shooter_msg::SHOOTER_POS2_DOWN:
            return ShooterState::POS2_DOWN;
            break;
        case shooter_msg::SHOOTER_POS3_UP:
            return ShooterState::POS3_UP;
            break;
        case shooter_msg::SHOOTER_POS3_DOWN:
            return ShooterState::POS3_DOWN;
            break;
        default:
            return ShooterState::POS_0; // TODO: 仕様決める
    }
}

shooter_msg convert(ShooterState state){
    shooter_msg msg;
    switch (state) {
        case ShooterState::IS_MOVING:
            msg.state = shooter_msg::SHOOTER_MOVING;
            break;
        case ShooterState::POS_0:
            msg.state = shooter_msg::SHOOTER_POS0;
            break;
        case ShooterState::POS1_UP:
            msg.state = shooter_msg::SHOOTER_POS1_UP;
            break;
        case ShooterState::POS1_DOWN:
            msg.state = shooter_msg::SHOOTER_POS1_DOWN;
            break;
        case ShooterState::POS2_UP:
            msg.state = shooter_msg::SHOOTER_POS2_UP;
            break;
        case ShooterState::POS2_DOWN:
            msg.state = shooter_msg::SHOOTER_POS2_DOWN;
            break;
        case ShooterState::POS3_UP:
            msg.state = shooter_msg::SHOOTER_POS3_UP;
            break;
        case ShooterState::POS3_DOWN:
            msg.state = shooter_msg::SHOOTER_POS3_DOWN;
            break;
        default:
            msg.state = shooter_msg::SHOOTER_POS0; // TODO: 仕様決める
    }
    return msg;
}

bool is_up(ShooterState state){
    return (state == ShooterState::POS1_UP || state == ShooterState::POS2_UP || state == ShooterState::POS3_UP);
}

double POS_to_pos(ShooterState state){
    switch (state) {
        case ShooterState::IS_MOVING:
        case ShooterState::POS_0:
            return 0.0;  // TODO: 値入れる
        case ShooterState::POS1_UP:
        case ShooterState::POS1_DOWN:
            return 0.0;
        case ShooterState::POS2_UP:
        case ShooterState::POS2_DOWN:
            return 0.0;
        case ShooterState::POS3_UP:
        case ShooterState::POS3_DOWN:
            return 0.0;
        default:
            return 0.0;
    }
}

namespace arm_controller {
    ShooterNode::ShooterNode(const rclcpp::NodeOptions &options)
            :Node("shooter_node_component", options){

        _pub_shooter_msg = this->create_publisher<shooter_msg>("shooter_state", 10);
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 5);
        _pub_b3m = this->create_publisher<kondo_msg>("b3m_topic", 10);
        _timer_movement = this->create_wall_timer(100ms, std::bind(&ShooterNode::timer_callback_movement, this));
        _timer_pub_state = this->create_wall_timer(50ms, std::bind(&ShooterNode::timer_callback_send_state, this));

        auto bind_callback = std::bind(&ShooterNode::sub_callback, this, _1);
        _sub_request = this->create_subscription<shooter_msg>("request_shooter_state", 10, bind_callback);
        auto bind_callback2 = std::bind(&ShooterNode::sub_callback_feedback, this, _1);
        _sub_feedback = this->create_subscription<actuator_fb_msg>("mros_output_mcmd", 10, bind_callback2);

        _b3m_init(_up_servo_id);  // init b3m
    }

    ShooterNode::~ShooterNode(){}

    void ShooterNode::_b3m_init(uint8_t servo_id) {  // b3mのinit
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

    void ShooterNode::timer_callback_movement() {
        if(_reqested_shooter_state == _shooter_state_now)return;  // 要求されたstateと現在のstateが同じなら動作停止
        if(_shooter_state_now != ShooterState::IS_MOVING)change_shooter_state(ShooterState::IS_MOVING);

        if(!_time_counter.is_enable())_time_counter.enable();
        _time_counter.count(100);
        bool is_moved_servo = (is_up(_shooter_state_pre) == is_up(_reqested_shooter_state));  // servoの動作が完了したか否か
        double target_pos = POS_to_pos(_reqested_shooter_state);

        _pub_micro_ros->publish(gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_MCMD3, 1, 0, target_pos));
        send_servo(is_up(_reqested_shooter_state));
        const double move_threshold = 5.0f;
        if((is_moved_servo || _time_counter.check_time(wait_time_for_servo)) && (abs(target_pos - _feedback_pos) < move_threshold)){
            _time_counter.disable();
            change_shooter_state(_reqested_shooter_state);
        }
    }

    void ShooterNode::change_shooter_state(ShooterState next_state) {
        _shooter_state_pre = _shooter_state_now;
        _shooter_state_now = next_state;
    }

    void ShooterNode::timer_callback_send_state() {
        _pub_shooter_msg->publish(convert(this->_shooter_state_now));
    }


    void ShooterNode::sub_callback(const shooter_msg &msg) {
        if(msg.state == shooter_msg::SHOOTER_MOVING){
            RCLCPP_ERROR(this->get_logger(), "[ERROR] shooter node, invalid request state");
        }else {
            _reqested_shooter_state = convert(msg);
        }
    }

    void ShooterNode::send_servo(bool is_up) {
        if(is_up){
            _pub_b3m->publish(gen_b3m_set_pos_msg(_up_servo_id, _angle_range, 0));
        }else{
            _pub_b3m->publish(gen_b3m_set_pos_msg(_up_servo_id, -_angle_range, 0));
        }
    }

    void ShooterNode::sub_callback_feedback(const actuator_fb_msg &msg) {
        if(msg.device.node_type.node_type == actuator_msgs::msg::NodeType::NODE_MCMD4
            && msg.device.device_num == device_num && msg.device.node_id == node_id){
            _feedback_pos = msg.fb_data;
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::ShooterNode)
