//
// Created by emile on 23/08/31.
//

#ifndef ROS2_WS_ROBOT_STATE_HPP
#define ROS2_WS_ROBOT_STATE_HPP

struct TipState{  // Main Armの先端ハンドの姿勢と状態
    float x, y, z, theta;

    TipState(float x, float y, float z, float theta): x(x), y(y), z(z), theta(theta){}
};

struct ArmState{  // Main Arm自体のstate
    float arm_r, arm_theta, arm_z;  // r-thetaのアームのstate
    float hand_theta; // handの角度

    ArmState(float arm_r, float arm_theta, float arm_z, float hand_theta)
        :arm_r(arm_r), arm_theta(arm_theta), arm_z(arm_z), hand_theta(hand_theta){}
};

#endif //ROS2_WS_ROBOT_STATE_HPP
