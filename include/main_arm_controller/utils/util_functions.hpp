//
// Created by emile on 23/09/10.
//

#ifndef ROS2_WS_UTIL_FUNCTIONS_HPP
#define ROS2_WS_UTIL_FUNCTIONS_HPP

#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>

template<class T>bool chmax(T &former, const T &b) { if (former<b) { former=b; return true; } return false; }
template<class T>bool chmin(T &former, const T &b) { if (b<former) { former=b; return true; } return false; }

float clip_f(float value, float min_v, float max_v);
float rad_to_deg(float rad);
float deg_to_rad(float deg);
float convert_angle_in_pi(float angle_rad);

int binary_search(int n_min, int n_max, std::function<bool(int)> f);  // fはleftでfalse, rightでtrue

actuator_msgs::msg::ActuatorMsg gen_actuator_msg(uint8_t node_type, uint8_t node_id, uint8_t device_id, float target_value, bool air_target=false);
kondo_drivers::msg::B3mServoMsg gen_b3m_set_pos_msg(uint8_t servo_id, float target_pos, uint16_t move_time=0);
kondo_drivers::msg::B3mServoMsg gen_b3m_write_msg(uint8_t servo_id, uint8_t TxData, uint8_t address);


#endif //ROS2_WS_UTIL_FUNCTIONS_HPP
