//
// Created by emile on 23/09/10.
//

#include <main_arm_controller/utils/util_functions.hpp>
#include <cmath>
#include <numeric>
#include <algorithm>


float clip_f(float value, float min_v, float max_v){
    return std::min(max_v, std::max(min_v, value));
}

float rad_to_deg(float rad){
    return (float)(rad / M_PI * 180.0);
}

float deg_to_rad(float deg) {
    return (float)(deg / 180.0 * M_PI);
}

float convert_angle_in_pi(float angle_rad) {
    while(angle_rad >= M_PI)angle_rad -= 2.0*M_PI;
    while(angle_rad < -M_PI)angle_rad += 2.0*M_PI;
    return angle_rad;
}


actuator_msgs::msg::ActuatorMsg gen_actuator_msg(uint8_t node_type, uint8_t node_id, uint8_t device_id, float target_value, bool air_target){
    actuator_msgs::msg::ActuatorMsg target_data;
    target_data.device.node_type.node_type = node_type;
    target_data.device.node_id = node_id;
    target_data.device.device_num = device_id;
    target_data.target_value = target_value;
    target_data.air_target = air_target;
    return target_data;
}


kondo_drivers::msg::B3mServoMsg gen_b3m_set_pos_msg(uint8_t servo_id, float target_pos, uint16_t move_time) {
    kondo_drivers::msg::B3mServoMsg ans;
    ans.servo_id = servo_id;
    ans.command_type = kondo_drivers::msg::B3mServoMsg::CMD_SET_POS;
    ans.cmd_set_pos.target_pos = target_pos;
    ans.cmd_set_pos.move_time = move_time;
    return ans;
}

kondo_drivers::msg::B3mServoMsg gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address) {
    kondo_drivers::msg::B3mServoMsg ans;
    ans.servo_id = servo_id;
    ans.command_type = kondo_drivers::msg::B3mServoMsg::CMD_WRITE;
    ans.cmd_write.txdata = TxData;
    ans.cmd_write.address = address;
    return ans;
}

int binary_search(int n_min, int n_max, std::function<bool(int)> f) {  // fはleftでfalse, rightでtrue
    int left = n_min-1;  // 常に f(left) = false
    int right = n_max; // 常に f(right) = true

    while (right - left > 1) {  // meguru
        int mid = left + (right - left) / 2;

        if (f(mid)){
            right = mid;
        } else {
            left = mid;
        }
    }
    return right; // left は条件を満たさない最大の値、right は条件を満たす最小の値になる
}

