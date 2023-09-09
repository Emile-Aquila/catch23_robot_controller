//
// Created by emile on 23/08/31.
//

#ifndef ROS2_WS_JOYSTICK_STATE_HPP
#define ROS2_WS_JOYSTICK_STATE_HPP

#include "sensor_msgs/msg/joy.hpp"
#include <vector>
#include <iterator>
#include <utility>


class JoyStickState{
private:
    std::vector<float> _axes;
    std::vector<int> _button_inputs;
    std::vector<int> _button_inputs_pre;

public:
    JoyStickState();
    explicit JoyStickState(const sensor_msgs::msg::Joy &joy_msg);
    void set(const sensor_msgs::msg::Joy &joy_msg);
    bool get_button_1_indexed(int id, bool strictly_changed=false);
    std::pair<float, float> get_joystick_left_xy();
    std::pair<float, float> get_joystick_right_xy();
};


#endif //ROS2_WS_JOYSTICK_STATE_HPP
