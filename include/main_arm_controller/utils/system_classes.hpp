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
#include <sensor_msgs/msg/joy.hpp>
#include <main_arm_controller/utils/robot_state.hpp>


enum class ControllerState{
    CTRL_HUMAN,
    CTRL_AUTO,  // 自動モード
};


enum class PlannerState{
    PLANNER_WAITING,
    PLANNER_BEFORE_GENERATING,
    PLANNER_GENERATING,
    PLANNER_FOLLOWING,
};

enum class CommonAreaState{
    COMMON_AREA_ENABLE,  // 共通エリア入って良いよ
    COMMON_AREA_DISABLE,
};

enum class HandUnitState {  // handの手先の状態 (上下)
    HAND_WAIT,  // workを持っていない状態でz=0
    HAND_BEFORE, // handをおろし始めた状態
    HAND_MOTION,   // 把持動作 or 開放動作
    HAND_AFTER,  // handを持ち上げる動作
    HAND_CARRY,  // workを持っている状態でz=0
};

enum class HandMotionType{
    MOTION_NULL,
    MOTION_GRAB_OUR_AREA,
    MOTION_GRAB_COMMON,
    MOTION_RELEASE_SHOOTER,
    MOTION_RELEASE_NORMAL,
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



class PositionSelector{
private:
    std::vector<TipStates> _tip_states_list;
    int id_now = -1;
public:
    PositionSelector();
    PositionSelector(std::vector<TipStates>& tip_states_list);
    int size();
    TipStates next();
    TipStates prev();
    TipStates get();
    void clear();
    bool complete();
};

PositionSelector get_position_selector_shooter(bool is_red);
PositionSelector get_position_selector_targets(bool is_red);



class TimeCounter{  // 擬似的に秒数を数える
private:
    uint64_t _counter = 0;
    bool _is_enable = false;
public:
    void count(uint64_t time_interval);
    bool check_time(uint64_t ms);
    bool is_enable();
    void disable();
    void enable();

};

#endif //ROS2_WS_SYSTEM_CLASSES_HPP
