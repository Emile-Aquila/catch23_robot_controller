//
// Created by emile on 23/09/10.
//

#ifndef ROS2_WS_SYSTEM_CLASSES_HPP
#define ROS2_WS_SYSTEM_CLASSES_HPP

#include <functional>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <queue>
#include <memory>
#include <vector>
#include <main_arm_controller/utils/joystick_state.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include <main_arm_controller/utils/robot_state.hpp>


enum class ControllerState{
    CTRL_HUMAN,
    CTRL_AUTO,  // 自動モード
};

enum class AutoState{
    AUTO_WAITING,
    AUTO_BEFORE_GENERATING,
    AUTO_GENERATING,
    AUTO_FOLLOWING,
};

class MainArmState {
private:
    ArmState _arm_state;
    TipState _tip_state;
    bool _is_hand_open;

public:
    MainArmState();
    MainArmState(const TipState& tip_state, bool is_hand_open);
    void set_state(const ArmState& arm_state);
    void set_state(const TipState& tip_state);
    void set_state(bool is_hand_open);

    ArmState arm_state();
    TipState tip_state();
    bool is_hand_open();
};

class TrajectoryData {
private:
    std::vector<ArmState> _trajectory;
    size_t _front_id;
public:
    TrajectoryData();
    void clear();
    void set(const std::vector<ArmState>& trajectory);
    ArmState get_front();
    bool complete();
};


#endif //ROS2_WS_SYSTEM_CLASSES_HPP
