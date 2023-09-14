//
// Created by emile on 23/08/31.
//

#ifndef ROS2_WS_ROBOT_STATE_HPP
#define ROS2_WS_ROBOT_STATE_HPP

#include <catch23_robot_controller/msg/tip_state.hpp>
#include <catch23_robot_controller/msg/arm_state.hpp>


struct ArmState{  // Main Arm自体のstate
    float r, theta, z;  // r-thetaのアームのstate
    float phi; // handの角度, 原点はarmの機構の原点

    ArmState(): r(0.0f), theta(0.0f), z(0.0f), phi(0.0f) {}
    ArmState(float r, float arm_theta, float z, float hand_phi);
    void simplify();

    ArmState operator +(const ArmState& as2) const{ return {r + as2.r, theta + as2.theta, z + as2.z, phi + as2.phi}; }
    ArmState operator *(const float value) const{ return {r*value, theta*value, z*value, phi * value}; }
    ArmState operator -(const ArmState& as2) const{ return {r - as2.r, theta - as2.theta, z - as2.z, phi - as2.phi}; }
    bool operator ==(const ArmState& as2) const { return (r==as2.r && theta == as2.theta && z == as2.z && phi == as2.phi); }
    bool operator !=(const ArmState& as2) const { return !(r==as2.r && theta == as2.theta && z == as2.z && phi == as2.phi); }
};


struct TipState{  // Main Armの先端ハンドの姿勢と状態
public:
    float x, y, z, theta;  // thetaはx軸正の方向が0.0rad

    TipState(): x(0.0f), y(0.0f), z(0.0f), theta(0.0f) {}
    TipState(float x, float y, float z, float theta);
//    void simplify();

    TipState operator +(const TipState& tip2) const{ return {x+tip2.x, y+tip2.y, z+tip2.z, theta+tip2.theta}; }
    TipState operator *(const float value) const{ return {x*value, y*value, z*value, theta*value}; }
    TipState operator -(const TipState& tip2) const{ return {x-tip2.x, y-tip2.y, z-tip2.z, theta-tip2.theta}; }
    bool operator ==(const TipState& tip2) const { return (x==tip2.x && y==tip2.y && z == tip2.z && theta == tip2.theta); }
    bool operator !=(const TipState& tip2) const { return !(x==tip2.x && y==tip2.y && z == tip2.z && theta == tip2.theta); }
};


using ArmStates = std::vector<ArmState>;
using TipStates = std::vector<TipState>;


ArmState arm_ik(const TipState& tip_state);
TipState arm_fk(const ArmState& arm_state);

TipState convert_tip_state(const catch23_robot_controller::msg::TipState& tip_state_msg);
ArmState convert_arm_state(const catch23_robot_controller::msg::ArmState& arm_state_msg);
catch23_robot_controller::msg::TipState convert_tip_state(const TipState& tip_state);
catch23_robot_controller::msg::ArmState convert_arm_state(const ArmState& arm_state);

#endif //ROS2_WS_ROBOT_STATE_HPP
