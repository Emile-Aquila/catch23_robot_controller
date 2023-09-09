//
// Created by emile on 23/08/31.
//

#include <main_arm_controller/utils/robot_state.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <cmath>


ArmState arm_ik(const TipState& tip_state){
    ArmState ans;
    ans.r = fsqrt(pow(tip_state.x, 2.0) + pow(tip_state.y, 2.0));
    ans.theta = atan2f(tip_state.y, tip_state.x);
    ans.z = tip_state.z;
    ans.phi = tip_state.theta + ans.theta;  // TODO: 合ってる?
    return ans;
}

TipState arm_fk(const ArmState& arm_state){
    TipState tip;
    tip.x = arm_state.r * cosf(arm_state.theta);
    tip.y = arm_state.r * sinf(arm_state.theta);
    tip.z = arm_state.z;
    tip.theta = arm_state.phi - arm_state.theta;  // TODO: 合ってる?
    return tip;
}

TipState convert_tip_state(const catch23_robot_controller::msg::TipState &tip_state_msg) {
    return {tip_state_msg.x, tip_state_msg.y, tip_state_msg.z, tip_state_msg.theta};
}

catch23_robot_controller::msg::TipState convert_tip_state(const TipState &tip_state) {
    catch23_robot_controller::msg::TipState ans;
    ans.x = tip_state.x;
    ans.y = tip_state.y;
    ans.z = tip_state.z;
    ans.theta = tip_state.theta;
    return ans;
}

ArmState convert_arm_state(const catch23_robot_controller::msg::ArmState &arm_state_msg) {
    return {arm_state_msg.r, arm_state_msg.theta, arm_state_msg.z, arm_state_msg.phi};
}

catch23_robot_controller::msg::ArmState convert_arm_state(const ArmState &arm_state) {
    catch23_robot_controller::msg::ArmState ans;
    ans.r = arm_state.r;
    ans.theta = arm_state.theta;
    ans.phi = arm_state.phi;
    ans.z = arm_state.z;
}
