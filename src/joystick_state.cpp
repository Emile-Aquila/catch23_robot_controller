//
// Created by emile on 23/08/31.
//

#include <main_arm_controller/joystick_state.hpp>


JoyStickState::JoyStickState(const sensor_msgs::msg::Joy &joy_msg){
    _axes = joy_msg.axes;
    _button_inputs = joy_msg.buttons;
}

bool JoyStickState::get_button_1_indexed(int id) {
    // buttonは0-indexになってる
    if(id < 1 || id >= (int)_button_inputs.size()){
        return false;
    }
    return _button_inputs[id-1];
}

std::pair<float, float> JoyStickState::get_joystick_left_xy() {
    // axes[0]: left | lr
    // axes[1]: left | ud
    // axes[3]: right | lr
    // axes[4]: right | ud
    return std::make_pair(_axes[0], _axes[1]);
}

std::pair<float, float> JoyStickState::get_joystick_right_xy() {
    return std::make_pair(_axes[3], _axes[4]);
};

