//
// Created by emile on 23/06/09.
//

#include <functional>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "main_arm_controller/main_arm_controller.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>
#include <actuator_msgs/msg/device_info.hpp>
#include <rclcpp/qos.hpp>


using namespace std::chrono_literals;
using kondo_msg = kondo_drivers::msg::B3mServoMsg;

float clip_f(float value, float min_v, float max_v){
    return std::min(max_v, std::max(min_v, value));
}

const float min_r = 325.0f;
const float max_r = 975.0f;
const float min_theta = -M_PI / 2.0f;
const float max_theta = M_PI;


float rad_to_deg(const float& rad){
    return (float)(rad / M_PI * 180.0f);
}


ArmState clip_arm_state(const ArmState& arm_state) {
    ArmState ans;
    ans.r = clip_f(arm_state.r, min_r, max_r);
    ans.theta = clip_f(arm_state.theta, min_theta, max_theta);
    ans.z = clip_f(arm_state.z, 0.0f, 225.0f);
    ans.phi = clip_f(arm_state.phi, -M_PI*32.0f/18.0f, M_PI*32.0f/18.0f);  // TODO: 実装
    return ans;
}



namespace arm_controller{
    ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
    : Node("main_arm_controller_component", options) {
        uint8_t ikko_servo_id = 0;
        uint8_t wrist_servo_id = 1;  // TODO: rosparam化
        _controller_state = ControllerState::CTRL_HUMAN;
        _requested_tip_state = _tip_state_origin;  // 初期位置
        _hand_is_open = false;  // ハンドが開いてるか

        // xy座標で動かす
        auto joy_callback_xy = [this, ikko_servo_id](const sensor_msgs::msg::Joy &msg) -> void {
            this->joy_state.set(msg);

            auto [d_x, d_y] = this->joy_state.get_joystick_left_xy();
            auto [d_theta, d_z] = this->joy_state.get_joystick_right_xy();
            if(d_z > 0.0f)d_z = 1.0f;
            if(d_z < 0.0f)d_z = -1.0f;

            if(this->joy_state.get_button_1_indexed(7, true)) {
                switch (this->_controller_state) {
                    case ControllerState::CTRL_HUMAN:
                        this->_controller_state = ControllerState::CTRL_AUTO;
                        RCLCPP_WARN(this->get_logger(), "[INFO] -> CTRL_AUTO");
                        break;
                    case ControllerState::CTRL_AUTO:
                        this->_controller_state = ControllerState::CTRL_HUMAN;
                        RCLCPP_WARN(this->get_logger(), "[INFO] -> CTRL_HUMAN");
                        break;
                }
            }

            TipState next_tip_state = _requested_tip_state + TipState(d_x * 10.0f, d_y * 10.0f, d_z * 5.0f, d_theta * 2.0f * M_PI / 180.0f);
            ArmState next_arm_state = arm_ik(next_tip_state);
            if(next_arm_state == clip_arm_state(next_arm_state)){  // TODO: verify
                if(_requested_arm_state != next_arm_state) {
                    _requested_arm_state = next_arm_state;
                    _requested_tip_state = arm_fk(_requested_arm_state);

                    RCLCPP_INFO(this->get_logger(), "x,y,z,theta: %.2lf, %.2lf, %.2lf, %.3lf",
                                _requested_tip_state.x, _requested_tip_state.y, _requested_tip_state.z, _requested_tip_state.theta);
                    RCLCPP_INFO(this->get_logger(), " --> r,theta,z,phi: %.2lf, %.3lf, %.2lf, %.3lf",
                                _requested_arm_state.r, _requested_arm_state.theta, _requested_arm_state.z, _requested_arm_state.phi);
                }
            }else{
                RCLCPP_WARN(this->get_logger(), "Invalid input!");
            }
            _send_request_arm_state(_requested_arm_state);

            // handの開閉
            if(this->joy_state.get_button_1_indexed(6, true)){
                if(this->_hand_is_open){
                    this->_hand_is_open = false;
                    this->_request_hand_open_close(true);  // handを閉じる
                    RCLCPP_INFO(this->get_logger(), "[INFO] close hand!");
                }else{
                    this->_hand_is_open = true;
                    this->_request_hand_open_close(false);  // handを開ける
                    RCLCPP_INFO(this->get_logger(), "[INFO] open hand!");
                }
            }

            // 一個取り
            if(this->joy_state.get_button_1_indexed(11, true)) {
                RCLCPP_INFO(this->get_logger(), "[INFO] ikkodori hand!");
            }

            // 妨害機構
            if(this->joy_state.get_button_1_indexed(12, true)){

            }

            // 原点に戻る
            if(this->joy_state.get_button_1_indexed(9, true)) {
                RCLCPP_WARN(this->get_logger(), "[INFO] return to origin!");
                _send_request_arm_state(clip_arm_state(arm_ik(this->_tip_state_origin)));
                this->_requested_tip_state = this->_tip_state_origin;
            }
        };

        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 5, joy_callback_xy);
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 10);
        _pub_micro_ros_r = this->create_publisher<std_msgs::msg::Float32>("mros_input_r", 10);
        _pub_micro_ros_theta = this->create_publisher<std_msgs::msg::Float32>("mros_input_theta", 10);

        _pub_b3m = this->create_publisher<kondo_msg>("b3m_topic", 10);
        _b3m_client = this->create_client<kondo_srv>("b3m_service");
        _traj_client = this->create_client<traj_srv>("arm_trajectory_service");

        while(!_b3m_client->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "waiting for b3m service...");
        }
        while(!_traj_client->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "waiting for arm_trajectory_service...");
        }

        rclcpp::sleep_for(100ms);
        _b3m_init(wrist_servo_id);  // init b3m
        _b3m_init(ikko_servo_id);  // init b3m

//        _timer_planner = this->create_wall_timer(30ms, timer_callback);
        RCLCPP_WARN(this->get_logger(), "[START] main_arm_controller");
    }

    ArmControllerNode::~ArmControllerNode(){}

    actuator_msgs::msg::ActuatorMsg ArmControllerNode::_gen_actuator_msg(uint8_t node_type, uint8_t node_id, uint8_t device_id, float target_value, bool air_target){
        actuator_msg target_data;
        target_data.device.node_type.node_type = node_type;
        target_data.device.node_id = node_id;
        target_data.device.device_num = device_id;
        target_data.target_value = target_value;
        target_data.air_target = air_target;
        return target_data;
    }


    kondo_msg ArmControllerNode::_gen_b3m_set_pos_msg(uint8_t servo_id, float target_pos, uint16_t move_time) {
        kondo_msg ans;
        ans.servo_id = servo_id;
        ans.command_type = kondo_msg::CMD_SET_POS_B3M;
        ans.cmd_set_pos.target_pos = target_pos;
        ans.cmd_set_pos.move_time = move_time;
        return ans;
    }

    kondo_msg ArmControllerNode::_gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address) {
        kondo_msg ans;
        ans.servo_id = servo_id;
        ans.command_type = kondo_msg::CMD_WRITE_B3M;
        ans.cmd_write.txdata = TxData;
        ans.cmd_write.address = address;
        return ans;
    }

    void ArmControllerNode::_b3m_init(uint8_t servo_id) {  // b3mのinit
        _pub_b3m->publish(_gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 動作モードをfreeに
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(_gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 位置制御モードに
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(_gen_b3m_write_msg(servo_id, 0x00, 0x29)); // 軌道生成タイプ：Normal (最速で回転)
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(_gen_b3m_write_msg(servo_id, 0x00, 0x5C)); // PIDの設定を位置制御のプリセットに合わせる
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(_gen_b3m_write_msg(servo_id, 0x00, 0x28)); // 動作モードをNormalに
        rclcpp::sleep_for(100ms);
    }

    void ArmControllerNode::_send_request_arm_state(const ArmState &req_arm_state) {
        uint8_t wrist_servo_id = 1;  // TODO: rosparam化
        std_msgs::msg::Float32 tgt_data;
        tgt_data.data = req_arm_state.r;
        _pub_micro_ros_r->publish(tgt_data);
        tgt_data.data = req_arm_state.theta;
        _pub_micro_ros_theta->publish(tgt_data);

        _pub_micro_ros->publish(_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_MCMD3, 1, 0, req_arm_state.z));
        _pub_b3m->publish(_gen_b3m_set_pos_msg(wrist_servo_id, -rad_to_deg(req_arm_state.phi), 0));
    }

    void ArmControllerNode::_request_hand_open_close(bool hand_close) {
        _pub_micro_ros->publish(this->_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 0, 1, 0.0f, hand_close));
        _pub_micro_ros->publish(this->_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 0, 0, 0.0f, hand_close));
        _pub_micro_ros->publish(this->_gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 0, 2, 0.0f, hand_close));
    }
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::ArmControllerNode)
