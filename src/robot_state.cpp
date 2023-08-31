//
// Created by emile on 23/08/31.
//

#include <main_arm_controller/robot_state.hpp>
#include <cmath>


ArmState arm_ik(const TipState& tip_state){
    ArmState ans;
    ans.r = sqrtf(pow(tip_state.x, 2.0) + pow(tip_state.y, 2.0));
    ans.theta = atan2(tip_state.y, tip_state.x);
    ans.z = tip_state.z;
    ans.phi = tip_state.theta - ans.theta;  // TODO: 合ってる?
    return ans;
}

TipState arm_fk(const ArmState& arm_state){
    TipState tip;
    tip.x = arm_state.r * cos(arm_state.theta);
    tip.y = arm_state.r * sin(arm_state.theta);
    tip.z = arm_state.z;
    tip.theta = arm_state.phi+arm_state.theta;  // TODO: 合ってる?
    return tip;
}