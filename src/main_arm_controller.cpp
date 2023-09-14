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
    ans.phi = clip_f(arm_state.phi, deg_to_rad(-97.0f), deg_to_rad(110.0f));
    return ans;
}



namespace arm_controller{
    ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions & options)
    : Node("main_arm_controller_component", options) {
        uint8_t ikko_servo_id = 0;
        uint8_t wrist_servo_id = 1;  // TODO: rosparam化
        _requested_state = MainArmState(_tip_state_origin, false);  // 初期位置

        _field_tip_pos = PositionSelector(get_position_selector_targets(true));  // TODO: 実装
        _common_tip_pos = PositionSelector(get_position_selector_targets(true));
        _shooter_tip_pos = PositionSelector(get_position_selector_shooter(true));


        // xy座標で動かす
        auto joy_callback_xy = [this, ikko_servo_id](const sensor_msgs::msg::Joy &msg) -> void {
            this->joy_state.set(msg);
            // コントローラー入力の処理
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


            if(this->_controller_state == ControllerState::CTRL_HUMAN && this->_planner_state == PlannerState::PLANNER_WAITING) {
                // CTRL_HUMANの動作
                TipState next_tip_state = this->_requested_state.tip_state() + TipState(d_x * 15.0f, d_y * 15.0f, d_z * 4.5f, d_theta * 2.0f * M_PI / 180.0f);
                ArmState next_arm_state = arm_ik(next_tip_state);
                if (next_arm_state == clip_arm_state(next_arm_state) && (abs(next_arm_state.theta- this->_requested_state.arm_state().theta) <= M_PI)) {
                    if (d_x != 0.0 || d_y != 0.0 || d_z != 0.0 || d_theta != 0.0) {
                        this->_change_planner_state(PlannerState::PLANNER_WAITING);
                        RCLCPP_INFO(this->get_logger(), "x,y,z,theta: %.2lf, %.2lf, %.2lf, %.3lf",
                                    this->_requested_state.tip_state().x, this->_requested_state.tip_state().y, this->_requested_state.tip_state().z, this->_requested_state.tip_state().theta);
                        RCLCPP_INFO(this->get_logger(), " --> r,theta,z,phi: %.2lf, %.3lf, %.2lf, %.3lf",
                                    this->_requested_state.arm_state().r, this->_requested_state.arm_state().theta, this->_requested_state.arm_state().z, this->_requested_state.arm_state().phi);
                        this->_send_request_arm_state(next_arm_state);
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid input!");
                }
//                this->_send_request_arm_state(this->_requested_state.arm_state());  // 人力でロボットを動かす場合の処理

            }else if(this->_controller_state == ControllerState::CTRL_AUTO) {
                // CTRL_AUTOの動作
                if(this->joy_state.get_button_1_indexed(5 , true)) {
                    // ワークの位置へ
                    TipStates target_points = {
                            this->_requested_state.tip_state(),
                            TipState(-505.0, 415.0, 0.0, M_PI/4.0f + M_PI_2)
                    };
                    bool is_completed = false;
                    if((this->_common_area_state == CommonAreaState::COMMON_AREA_ENABLE) && (!this->_common_tip_pos.complete())){
                        target_points = this->_common_tip_pos.next();
                    }else{
                        if(this->_field_tip_pos.complete()){
                            is_completed = true;
                        }else {
                            target_points = this->_field_tip_pos.next();
                        }
                    }
                    if(!is_completed) {
                        this->_request_trajectory_following(
                                target_points, this->_common_area_state == CommonAreaState::COMMON_AREA_ENABLE);
                    }else{// _field_tip_posが空
                        RCLCPP_WARN(this->get_logger(), "****[WARN]**** field_tip_pos is completed.");
                    }

                }else if(this->joy_state.get_button_1_indexed(6, true)){
                    // シューティングボックスへ
                    TipStates target_points = {
                            this->_requested_state.tip_state(),
                            TipState(410.0 + 100.0, 5.0 - 200.0, 0.0, M_PI_2),
                            TipState(410.0 + 100.0 + 100.0, 5.0 - 200.0, 0.0, M_PI_2),
                    };
                    bool is_completed = false;
                    if(this->_shooter_tip_pos.complete()){
                        is_completed = true;
                    }else {
                        target_points = this->_shooter_tip_pos.next();
                    }
                    if(!is_completed){
                        this->_request_trajectory_following(target_points, false);  // こっちはfalseの方が良いかも?
                    }else{ // _shooter_tip_posが空
                        RCLCPP_WARN(this->get_logger(), "****[WARN]**** shooter_tip_pos is completed.");
                    }
                }

                if(this->joy_state.get_button_1_indexed(9, true)) {  // 原点に戻る
                    RCLCPP_WARN(this->get_logger(), "[AUTO] return to origin!");
                    if(this->_requested_state.tip_state() != this->_tip_state_origin) {
                        TipStates target_points = {this->_requested_state.tip_state(), this->_tip_state_origin};
                        this->_request_trajectory_following(target_points, false);
                    }
                }
            }


            /* その他アクチュエーターの動作 */
            if(this->_controller_state == ControllerState::CTRL_HUMAN) {
                // handの開閉
                if (this->joy_state.get_button_1_indexed(6, true)) {
                    if (this->_requested_state.is_hand_open()) {
                        this->_requested_state.set_state(false);
                        this->_request_hand_open_close(true);  // handを閉じる
                        RCLCPP_INFO(this->get_logger(), "[INFO] close hand!");
                    } else {
                        this->_requested_state.set_state(true);
                        this->_request_hand_open_close(false);  // handを開ける
                        RCLCPP_INFO(this->get_logger(), "[INFO] open hand!");
                    }
                }

                // 一個取り
                if (this->joy_state.get_button_1_indexed(11, true)) {
                    RCLCPP_INFO(this->get_logger(), "[INFO] ikkodori hand!");
                    // 66.5f -> 0.0f -> -66.5f

                }

                // 妨害機構
                if (this->joy_state.get_button_1_indexed(12, true)) {
                    // 妨害の状態と共通エリアの状態は共通にしてある
                    switch(this->_common_area_state){
                        case CommonAreaState::COMMON_AREA_DISABLE:
                            RCLCPP_WARN(this->get_logger(), "[INFO] Common Area Enable");
                            this->_change_common_area_state(CommonAreaState::COMMON_AREA_ENABLE);
                            break;
                        case CommonAreaState::COMMON_AREA_ENABLE:
                            RCLCPP_WARN(this->get_logger(), "[INFO] Common Area Disable");
                            this->_change_common_area_state(CommonAreaState::COMMON_AREA_DISABLE);
                            break;
                    }
                }
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

        _timer_planner = this->create_wall_timer(100ms, std::bind(&ArmControllerNode::_trajectory_timer_callback, this));
        _timer_hand_unit = this->create_wall_timer(100ms, std::bind(&ArmControllerNode::_trajectory_timer_callback, this));
        RCLCPP_WARN(this->get_logger(), "[START] main_arm_controller");
    }

    ArmControllerNode::~ArmControllerNode(){}

    void ArmControllerNode::_trajectory_timer_callback(){  // 経路生成 & 追従
//        if(this->_controller_state == ControllerState::CTRL_HUMAN)return;
        if(this->_planner_state == PlannerState::PLANNER_BEFORE_GENERATING){  // 経路生成
            auto request = std::make_shared<traj_srv::Request>();
            if(this->_traj_target_points.size() < 2)RCLCPP_ERROR(this->get_logger(), "[ERROR] _traj_target_points.size() < 2");
            for(size_t i=0; i<this->_traj_target_points.size(); i++){
                if(i==0){
                    request->waypoints.emplace_back(convert_tip_state(this->_traj_target_points[i]));
                }else{
                    if(this->_traj_target_points[i-1] == this->_traj_target_points[i]){ // 同じ点が入っている場合は除去
                        RCLCPP_WARN(this->get_logger(), "****[WARN]**** Duplicated Points");
                        continue;
                    }
                    request->waypoints.emplace_back(convert_tip_state(this->_traj_target_points[i]));
                }
            }

            request->step_min = 10.0f;
            request->step_max = 100.0f;
            request->d_step_max = 15.0f;
            request->is_common = this->_traj_enter_common_area_is_enable;

            auto future_res = _traj_client->async_send_request(
                    request, std::bind(&ArmControllerNode::_traj_service_future_callback, this, std::placeholders::_1));
            // serviceのthreadを分ける必要がある
            this->_change_planner_state(PlannerState::PLANNER_GENERATING);

        }else if(this->_planner_state == PlannerState::PLANNER_FOLLOWING){  // 経路追従
            if(!(this->_trajectory_data.complete())) {  // 経路追従中
                this->_send_request_arm_state(this->_trajectory_data.get_front());
            }else{  // 経路追従が終わった状態
                RCLCPP_INFO(this->get_logger(), "[INFO] Path following completed!");
                this->_change_planner_state(PlannerState::PLANNER_WAITING);
            }
        }
    }

    void ArmControllerNode::_traj_service_future_callback(rclcpp::Client<traj_srv>::SharedFuture future){
        if(future.get()->is_feasible && (future.get()->trajectory.size() < 200)){
            auto ans_path = future.get() -> trajectory;
            std::vector<ArmState> tmp_traj;
            std::transform(ans_path.begin(), ans_path.end(), std::back_inserter(tmp_traj), [this](const auto& tmp){
                ArmState arm_state = convert_arm_state(tmp);
                if(clip_arm_state(arm_state) != arm_state){
                    RCLCPP_WARN(this->get_logger(), "****[WARN]**** trajectory clipped! : (%lf, %lf, %lf)", arm_state.r, arm_state.theta, arm_state.phi);
                }
                return clip_arm_state(arm_state);
            });
            _trajectory_data.set(tmp_traj);
            RCLCPP_INFO(this->get_logger(), "[INFO] Trajectory generated!!");
            RCLCPP_INFO(this->get_logger(), "[INFO] Start path following!");
            this->_change_planner_state(PlannerState::PLANNER_FOLLOWING);
        }else{
            RCLCPP_ERROR(this->get_logger(), "[ERROR] generated trajectory is invalid!");
            // 経路が生成できなった場合には位置制御を行う.
            this->_send_request_arm_state(arm_ik(this->_traj_target_points.back()));
            this->_change_planner_state(PlannerState::PLANNER_WAITING);
        }
    }

    bool ArmControllerNode::_change_controller_state(ControllerState next_state){
        _controller_state = next_state;
        return true;
    }

    bool ArmControllerNode::_change_planner_state(PlannerState next_state) {
        if(next_state == PlannerState::PLANNER_WAITING){
            _trajectory_data.clear();
        }
        _planner_state = next_state;
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

        _requested_state.set_state(req_arm_state);
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

    bool ArmControllerNode::_request_trajectory_following(std::vector<TipState> &traj_target_points, bool is_common) {
        if(this->_planner_state == PlannerState::PLANNER_WAITING){
            this->_traj_target_points.clear();
            std::copy(traj_target_points.begin(), traj_target_points.end(), std::back_inserter(this->_traj_target_points));
            this->_traj_enter_common_area_is_enable = is_common;
            this->_change_planner_state(PlannerState::PLANNER_BEFORE_GENERATING);
            return true;
        }else{
            return false;
        }
    }

    bool ArmControllerNode::_change_common_area_state(CommonAreaState next_state) {
        if(this->_common_area_state == next_state)return false;
        if(next_state == CommonAreaState::COMMON_AREA_ENABLE){
            // TODO: アクチュエーターの動作実装
        }else{
            // TODO: アクチュエーターの動作実装
        }
        this->_common_area_state = next_state;
        return true;
    }

    bool ArmControllerNode::_change_hand_unit_state(HandUnitState next_state) {
        this->_hand_unit_state = next_state;
    }

    void ArmControllerNode::_hand_unit_timer_callback() {
        TipState tip_state_now = this->_requested_state.tip_state();
        switch (this->_hand_unit_state) {
            case HandUnitState::HAND_WAIT:
                // no action
                break;
            case HandUnitState::HAND_BEFORE:  // ハンドを下ろす
                if(this->_hand_unit_motion_type == HandMotionType::MOTION_GRAB_OUR_AREA){
                    _request_hand_open_close(false);  // open
                    tip_state_now.z = 217.7f;
                }else if(this->_hand_unit_motion_type == HandMotionType::MOTION_GRAB_COMMON){
                    _request_hand_open_close(false);  // open
                    tip_state_now.z = 187.7f;
                }else if(this->_hand_unit_motion_type == HandMotionType::MOTION_RELEASE_NORMAL){
                    _request_hand_open_close(true);  // close
                    tip_state_now.z = 187.7f;  // TODO: 未定
                }else if(this->_hand_unit_motion_type == HandMotionType::MOTION_RELEASE_SHOOTER){
                    _request_hand_open_close(true);  // close
                    tip_state_now.z = 57.0f;
                }
                this->_send_request_arm_state(arm_ik(tip_state_now));
                // TODO: feedbackで遷移処理
                break;
            case HandUnitState::HAND_MOTION:  // ハンドの開閉を行う
                if(this->_hand_unit_motion_type == HandMotionType::MOTION_RELEASE_SHOOTER ||
                        this->_hand_unit_motion_type == HandMotionType::MOTION_RELEASE_NORMAL){
                    _request_hand_open_close(false);  // open
                }else if(this->_hand_unit_motion_type == HandMotionType::MOTION_GRAB_OUR_AREA ||
                        this->_hand_unit_motion_type == HandMotionType::MOTION_GRAB_COMMON){
                    _request_hand_open_close(true);  // close
                }  // TODO: n秒待つ
                break;
            case HandUnitState::HAND_AFTER:  // ハンドを上げる
                tip_state_now.z = 0.0f;
                this->_send_request_arm_state(arm_ik(tip_state_now));
                // TODO: feedbackで遷移処理
                break;
            case HandUnitState::HAND_CARRY:  // ハンドがworkを保持してz=0にした状態
                // no action   <- TODO: ?
                break;
        }
    }

    void ArmControllerNode::_set_hand_motion(HandMotionType hand_motion) {
        _hand_unit_motion_type = hand_motion;
    }
}




RCLCPP_COMPONENTS_REGISTER_NODE(arm_controller::ArmControllerNode)
