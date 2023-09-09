//
// Created by emile on 23/08/31.
//

#include <main_arm_controller/utils/joystick_state.hpp>
#include <iostream>

JoyStickState::JoyStickState() = default;

JoyStickState::JoyStickState(const sensor_msgs::msg::Joy &joy_msg){
    _axes = joy_msg.axes;
    _button_inputs = joy_msg.buttons;
}

bool JoyStickState::get_button_1_indexed(int id, bool strictly_changed) {
    // buttonは0-indexになってる
    if(id < 1 || id >= (int)_button_inputs.size())return false;
    if(strictly_changed){  // buttonが押されてない状態から押した状態に変化した時にtrueを返す
        return ((_button_inputs_pre[id-1] == 0) && (_button_inputs[id-1] == 1));
    }else {
        return _button_inputs[id - 1];
    }
}

std::pair<float, float> JoyStickState::get_joystick_left_xy() {
    // axes[0]: left | lr
    // axes[1]: left | ud
    // axes[3]: right | lr
    // axes[4]: right | ud
    float x = -_axes[0], y = _axes[1];
    if(_axes[4] != 0.0)x = -_axes[4];
    if(_axes[5] != 0.0)y = _axes[5];
    return std::make_pair(x, y);  // elecom
//    return std::make_pair(-_axes[0], _axes[1]);  // logicool
}

std::pair<float, float> JoyStickState::get_joystick_right_xy() {
    return std::make_pair(-_axes[3], _axes[2]);  // elecom
//    return std::make_pair(_axes[2], _axes[3]);  // logicool
}

void JoyStickState::set(const sensor_msgs::msg::Joy &joy_msg) {
    _axes = joy_msg.axes;
    if(_button_inputs_pre.empty()){
        _button_inputs_pre = joy_msg.buttons;
    }else{
        _button_inputs_pre = _button_inputs;
    }
    _button_inputs = joy_msg.buttons;
}



