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
#include <main_arm_controller/utils/util_functions.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>
#include <actuator_msgs/msg/device_info.hpp>
#include <rclcpp/qos.hpp>


using namespace std::chrono_literals;


const float min_r = 325.0f;
const float max_r = 975.0f;
const float min_theta = -M_PI;
const float max_theta = M_PI;



ArmState clip_arm_state(const ArmState& arm_state) {
    ArmState ans;
    ans.r = clip_f(arm_state.r, min_r, max_r);
    ans.theta = clip_f(arm_state.theta, min_theta, max_theta);
    ans.z = clip_f(arm_state.z, 0.0f, 225.0f);
    ans.phi = clip_f(arm_state.phi, deg_to_rad(-97.0f), deg_to_rad(110.0f));  // TODO: 実装
    return ans;
}



namespace arm_controller{
    ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
    : Node("main_arm_controller_component", options) {
        uint8_t ikko_servo_id = 0;
        uint8_t wrist_servo_id = 1;  // TODO: rosparam化
        _requested_state = MainArmState(_tip_state_origin, false);  // 初期位置

        // xy座標で動かす
        auto joy_callback_xy = [this, ikko_servo_id](const sensor_msgs::msg::Joy &msg) -> void {
            this->joy_state.set(msg);

            auto [d_x, d_y] = this->joy_state.get_joystick_left_xy();
            auto [d_theta, d_z] = this->joy_state.get_joystick_right_xy();
            if(d_z > 0.0f)d_z = 1.0f;
            if(d_z < 0.0f)d_z = -1.0f;


            if(this->joy_state.get_button_1_indexed(7, true)){  // HUMAN <--> AUTOの切り替え
                switch (this->_controller_state) {
                    case ControllerState::CTRL_HUMAN:
                        RCLCPP_WARN(this->get_logger(), "[INFO] CTRL_HUMAN -> CTRL_AUTO");
                        this->_change_controller_state(ControllerState::CTRL_AUTO);
                        break;
                    case ControllerState::CTRL_AUTO:
                        RCLCPP_WARN(this->get_logger(), "[INFO] CTRL_AUTO -> CTRL_HUMAN");
                        this->_change_controller_state(ControllerState::CTRL_HUMAN);
                        break;
                }
            }

            if(this->_controller_state == ControllerState::CTRL_HUMAN) { // CTRL_HUMANの動作
                TipState next_tip_state = this->_requested_state.tip_state() + TipState(d_x * 15.0f, d_y * 15.0f, d_z * 4.5f, d_theta * 2.0f * M_PI / 180.0f);
                ArmState next_arm_state = arm_ik(next_tip_state);
                if (next_arm_state == clip_arm_state(next_arm_state) && (abs(next_arm_state.theta- this->_requested_state.arm_state().theta) <= M_PI)) {
                    if (d_x != 0.0 || d_y != 0.0 || d_z != 0.0 || d_theta != 0.0) {
                        this->_requested_state.set_state(next_arm_state);

                        RCLCPP_INFO(this->get_logger(), "x,y,z,theta: %.2lf, %.2lf, %.2lf, %.3lf",
                                    this->_requested_state.tip_state().x, this->_requested_state.tip_state().y, this->_requested_state.tip_state().z, this->_requested_state.tip_state().theta);
                        RCLCPP_INFO(this->get_logger(), " --> r,theta,z,phi: %.2lf, %.3lf, %.2lf, %.3lf",
                                    this->_requested_state.arm_state().r, this->_requested_state.arm_state().theta, this->_requested_state.arm_state().z, this->_requested_state.arm_state().phi);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid input!");
                }
                this->_send_request_arm_state(this->_requested_state.arm_state());

            }else if(this->_controller_state == ControllerState::CTRL_AUTO) {  // CTRL_AUTOの動作
                switch (this->_auto_state) {
                    case AutoState::AUTO_WAITING:
                        // TODO: button押されたときの処理
                        if(this->joy_state.get_button_1_indexed(5 , true)) {  // ワークの位置へ
                            this->_traj_target_points = {
                                    this->_requested_state.tip_state(),
                                    TipState(-505.0, 415.0, 0.0, M_PI/4.0f + M_PI_2)
                            };
                            this->_change_auto_mode_state(AutoState::AUTO_BEFORE_GENERATING);
                        }else if(this->joy_state.get_button_1_indexed(6, true)){  // シューティングボックスへ
                            this->_traj_target_points = {
                                    this->_requested_state.tip_state(),
                                    TipState(410.0 + 100.0, 5.0 - 200.0, 0.0, M_PI_2),
                                    TipState(410.0 + 100.0 + 100.0, 5.0 - 200.0, 0.0, M_PI_2),
                            };
                            this->_change_auto_mode_state(AutoState::AUTO_BEFORE_GENERATING);
                        }

                        if(this->joy_state.get_button_1_indexed(9, true)) {  // 原点に戻る
                            RCLCPP_WARN(this->get_logger(), "[AUTO] return to origin!");
                            if(this->_requested_state.tip_state() != this->_tip_state_origin) {
                                _traj_target_points = {this->_requested_state.tip_state(), this->_tip_state_origin};
                                this->_change_auto_mode_state(AutoState::AUTO_BEFORE_GENERATING);
                            }
                        }
                        break;
                    case AutoState::AUTO_BEFORE_GENERATING:  // 経路生成前&生成中
                        // no action
                        break;
                    case AutoState::AUTO_GENERATING:
                        // no action
                        break;
                    case AutoState::AUTO_FOLLOWING:  // 経路追従中の処理
                        RCLCPP_INFO(this->get_logger(), "[INFO] path following...");
                        if(!(this->_trajectory_data.complete())) {  // 経路追従中
                            this->_requested_state.set_state(this->_trajectory_data.get_front());
                        }else{  // 経路追従が終わった状態
                            this->_change_auto_mode_state(AutoState::AUTO_WAITING);
                        }
                        this->_send_request_arm_state(this->_requested_state.arm_state());
                        break;
                }
            }

            // handの開閉
            if(this->joy_state.get_button_1_indexed(6, true)){
                if(this->_requested_state.is_hand_open()){
                    this->_requested_state.set_state(false);
                    this->_request_hand_open_close(true);  // handを閉じる
                    RCLCPP_INFO(this->get_logger(), "[INFO] close hand!");
                }else{
                    this->_requested_state.set_state(true);
                    this->_request_hand_open_close(false);  // handを開ける
                    RCLCPP_INFO(this->get_logger(), "[INFO] open hand!");
                }
            }

            // 一個取り
            if(this->joy_state.get_button_1_indexed(11, true)) {
                RCLCPP_INFO(this->get_logger(), "[INFO] ikkodori hand!");
                // 66.5f -> 0.0f -> -66.5f

            }

            // 妨害機構
            if(this->joy_state.get_button_1_indexed(12, true)){
            }
        };


        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy> ("joy", 10, joy_callback_xy);
        _pub_micro_ros = this->create_publisher<actuator_msg>("mros_input", 5);
        _pub_micro_ros_r = this->create_publisher<std_msgs::msg::Float32>("mros_input_r", 5);
        _pub_micro_ros_theta = this->create_publisher<std_msgs::msg::Float32>("mros_input_theta", 5);

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

        _timer_planner = this->create_wall_timer(30ms, std::bind(&ArmControllerNode::_trajectory_timer_callback, this));
        RCLCPP_WARN(this->get_logger(), "[START] main_arm_controller");
    }

    ArmControllerNode::~ArmControllerNode(){}

    void ArmControllerNode::_trajectory_timer_callback(){
        if(this->_controller_state == ControllerState::CTRL_HUMAN)return;
        if(this->_auto_state == AutoState::AUTO_BEFORE_GENERATING){
            auto request = std::make_shared<traj_srv::Request>();
            if(this->_traj_target_points.size() < 2)RCLCPP_ERROR(this->get_logger(), "[ERROR] _traj_target_points.size() < 2");
            for(const auto& tmp: this->_traj_target_points){
                request->waypoints.emplace_back(convert_tip_state(tmp));
            }
//            request->waypoints = {
//                    convert_tip_state(this->_traj_target_points.front()),
//                    convert_tip_state(this->_traj_target_points.back())
//            };

            request->step_min = 10.0f;
            request->step_max = 70.0f;
            request->d_step_max = 10.0f;
            request->is_common = false; // TODO: 条件に合わせて変更する.

            auto future_res = _traj_client->async_send_request(
                    request, std::bind(&ArmControllerNode::_traj_service_future_callback, this, std::placeholders::_1));
            // serviceのthreadを分ける必要がある
            this->_change_auto_mode_state(AutoState::AUTO_GENERATING);
        }
    }

    void ArmControllerNode::_traj_service_future_callback(rclcpp::Client<traj_srv>::SharedFuture future){
        if(future.get()->is_feasible){
            auto ans_path = future.get() -> trajectory;
            std::vector<ArmState> tmp_traj;
            std::transform(ans_path.begin(), ans_path.end(), std::back_inserter(tmp_traj), [this](const auto& tmp){
                ArmState arm_state = convert_arm_state(tmp);
                if(clip_arm_state(arm_state) != arm_state){
                    RCLCPP_WARN(this->get_logger(), "[WARN] trajectory clipped! : (%lf, %lf, %lf)", arm_state.r, arm_state.theta, arm_state.phi);
                }
                return clip_arm_state(arm_state);
            });
            _trajectory_data.set(tmp_traj);
            RCLCPP_INFO(this->get_logger(), "[INFO] trajectory generated!!");
            this->_change_auto_mode_state(AutoState::AUTO_FOLLOWING);
        }else{
            RCLCPP_ERROR(this->get_logger(), "[ERROR] generated trajectory is invalid!");
            this->_requested_state.set_state(this->_traj_target_points.back());  // 経路が生成できなった場合には位置制御を行う.
            _send_request_arm_state(this->_requested_state.arm_state());
            this->_change_auto_mode_state(AutoState::AUTO_WAITING);
        }
    }

    bool ArmControllerNode::_change_controller_state(ControllerState next_state){
        if(_controller_state != next_state){
            _change_auto_mode_state(AutoState::AUTO_WAITING);
            _trajectory_data.clear();
            _traj_target_points.clear();
        }
        _controller_state = next_state;
        return true;
    }

    bool ArmControllerNode::_change_auto_mode_state(AutoState next_state) {
        if(_auto_state == AutoState::AUTO_FOLLOWING && next_state == AutoState::AUTO_WAITING){
            _trajectory_data.clear();
        }
        _auto_state = next_state;
        return false;
    }

    void ArmControllerNode::_b3m_init(uint8_t servo_id) {  // b3mのinit
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 動作モードをfreeに
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x02, 0x28)); // 位置制御モードに
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x00, 0x29)); // 軌道生成タイプ：Normal (最速で回転)
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x00, 0x5C)); // PIDの設定を位置制御のプリセットに合わせる
        rclcpp::sleep_for(100ms);
        _pub_b3m->publish(gen_b3m_write_msg(servo_id, 0x00, 0x28)); // 動作モードをNormalに
        rclcpp::sleep_for(100ms);
    }

    void ArmControllerNode::_send_request_arm_state(const ArmState &req_arm_state) {
        uint8_t wrist_servo_id = 1;  // TODO: rosparam化
        std_msgs::msg::Float32 tgt_data;
        tgt_data.data = req_arm_state.r;
        _pub_micro_ros_r->publish(tgt_data);
        tgt_data.data = req_arm_state.theta;
        _pub_micro_ros_theta->publish(tgt_data);

        _pub_micro_ros->publish(gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_MCMD3, 1, 0, req_arm_state.z));
        _pub_b3m->publish(gen_b3m_set_pos_msg(wrist_servo_id, -rad_to_deg(req_arm_state.phi), 0));
    }

    void ArmControllerNode::_request_hand_open_close(bool hand_close) {
        _pub_micro_ros->publish(gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 0, 1, 0.0f, hand_close));
        _pub_micro_ros->publish(gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 0, 0, 0.0f, hand_close));
        _pub_micro_ros->publish(gen_actuator_msg(actuator_msgs::msg::NodeType::NODE_AIR, 0, 2, 0.0f, hand_close));
    }
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::ArmControllerNode)
