//
// Created by emile on 23/08/31.
//

#ifndef ROS2_WS_ROBOT_STATE_HPP
#define ROS2_WS_ROBOT_STATE_HPP

struct TipState{  // Main Armの先端ハンドの姿勢と状態
public:
    float x, y, z, theta;

    TipState(): x(0.0f), y(0.0f), z(0.0f), theta(0.0f) {}
    TipState(float x, float y, float z, float theta): x(x), y(y), z(z), theta(theta){}

    TipState operator +(const TipState& tip2) const{ return {x+tip2.x, y+tip2.y, z+tip2.z, theta+tip2.theta}; }
    TipState operator *(const float value) const{ return {x*value, y*value, z*value, theta*value}; }
    TipState operator -(const TipState& tip2) const{ return {x-tip2.x, y-tip2.y, z-tip2.z, theta-tip2.theta}; }
};

struct ArmState{  // Main Arm自体のstate
    float r, theta, z;  // r-thetaのアームのstate
    float phi; // handの角度

    ArmState(): r(0.0f), theta(0.0f), z(0.0f), phi(0.0f) {}
    ArmState(float r, float arm_theta, float z, float hand_phi)
        : r(r), theta(arm_theta), z(z), phi(hand_phi){}

    ArmState operator +(const ArmState& as2) const{ return {r + as2.r, theta + as2.theta, z + as2.z, phi + as2.phi}; }
    ArmState operator *(const float value) const{ return {r*value, theta*value, z*value, phi * value}; }
    ArmState operator -(const ArmState& as2) const{ return {r - as2.r, theta - as2.theta, z - as2.z, phi - as2.phi}; }
};

#endif //ROS2_WS_ROBOT_STATE_HPP