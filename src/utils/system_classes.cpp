//
// Created by emile on 23/09/10.
//

#include <iostream>
#include <main_arm_controller/utils/system_classes.hpp>
#include <main_arm_controller/utils/util_functions.hpp>


MainArmState::MainArmState(): _is_hand_open(true){}

void MainArmState::set_state(const ArmState &arm_state) {
    this->_arm_state = arm_state;
    this->_tip_state = arm_fk(arm_state);
}

void MainArmState::set_state(const TipState &tip_state) {
    this->_arm_state = arm_ik(tip_state);
    this->_tip_state = tip_state;
}

void MainArmState::set_state(bool is_hand_open) {
    _is_hand_open = is_hand_open;
}

ArmState MainArmState::arm_state() { return _arm_state; }
TipState MainArmState::tip_state() { return _tip_state; }
bool MainArmState::is_hand_open() { return _is_hand_open; }

MainArmState::MainArmState(const TipState &tip_state, bool is_hand_open) {
    _arm_state = arm_ik(tip_state);
    _tip_state = tip_state;
    _is_hand_open = is_hand_open;
}



TrajectoryData::TrajectoryData(){
    clear();
}

void TrajectoryData::clear(){
    _front_id = 0;
    _trajectory.clear();
}

void TrajectoryData::set(const std::vector<ArmState>& trajectory){
    clear();
    std::copy(trajectory.begin(), trajectory.end(), std::back_inserter(_trajectory));
}

ArmState TrajectoryData::get_front(){
    if(_front_id < _trajectory.size()){
        _front_id += 1;
        return _trajectory[_front_id-1];
    }else{
        return ArmState();
    }
}

bool TrajectoryData::complete(){
    return _front_id == _trajectory.size();
}


// Position Selector
PositionSelector::PositionSelector(std::vector<TipStates> tip_states_list) {
    std::copy(tip_states_list.begin(), tip_states_list.end(), std::back_inserter(this->_tip_states_list));
}

PositionSelector::PositionSelector(){
    this->clear();
}


int PositionSelector::size(){
    return (int)(_tip_states_list.size());
}

TipStates PositionSelector::next(){
    id_now = std::min((int)_tip_states_list.size()-1, id_now+1);
    if(id_now < 0)std::cerr<< "[ERROR] id_now < 0" << std::endl;
    return _tip_states_list[id_now];
}

TipStates PositionSelector::prev() {
    id_now = std::max(0, (int)id_now-1);
    return _tip_states_list[id_now];
}

TipStates PositionSelector::get() {
    if(id_now == -1)return _tip_states_list[0];
    return _tip_states_list[id_now];
}

void PositionSelector::clear() {
    id_now = -1;
    _tip_states_list.clear();
}

bool PositionSelector::complete() {
    return (id_now == ((int)_tip_states_list.size()-1) || _tip_states_list.empty());
}




// TimeCounter
void TimeCounter::count(uint64_t time_interval) {
    if(!_is_enable){
        _counter = 0;
        return;
    }
    if(_counter == UINT64_MAX){
        std::cerr << "[ERROR] TimeCounter is max." << std::endl;
    }else{
        _counter += time_interval;
    }
}

bool TimeCounter::check_time(uint64_t ms) {
    return (ms <= _counter) && _is_enable;
}

void TimeCounter::disable() {
    _is_enable = false;
    _counter = 0;
}

void TimeCounter::enable() {
    _is_enable = true;
}

bool TimeCounter::is_enable() {
    return _is_enable;
}



PositionSelector get_position_selector_shooter(bool is_red){
    double x_prepare = 508.0, x_release = 650.0;
    std::vector<double> ys = {-47.5, -267.5, -487.5};  // もう1箇所の投下位置はy-30した位置にある。
    std::vector<double> thetas = {0.0, 0.0, 0.0};

    std::vector<TipStates> vectors;
    for(int i=0; i<ys.size(); i++){
        double y = ys[i], theta = thetas[i];
        TipStates vec = {
                TipState(x_prepare, 0.0, 0.0, deg_to_rad(theta)),
                TipState(x_prepare, y, 0.0, deg_to_rad(theta)),
                TipState(x_release, y, 0.0, deg_to_rad(theta)),
        }, vec2 = {
                TipState(x_prepare, 0.0, 0.0, deg_to_rad(theta)),
                TipState(x_prepare, y - 30.0, 0.0, deg_to_rad(theta)),
                TipState(x_release, y - 30.0, 0.0, deg_to_rad(theta)),
        };
        vectors.emplace_back(vec);
        vectors.emplace_back(vec2);
    }

    if(is_red){
        for(auto& vec: vectors){
            for(auto& tmp: vec) {
                tmp.x *= -1.0;
                tmp.theta *= -1.0;
            }
        }
    }
    return PositionSelector(vectors);
}


PositionSelector get_position_selector_targets(bool is_red){
    double y = 392.5;

    TipState p4(500.0, y, 0.0, deg_to_rad(45.0));
    TipState p5(300.0, y, 0.0, deg_to_rad(45.0));
    TipState p6(100.0, y, 0.0, deg_to_rad(225.0));
    TipState p7(-100.0, y, 0.0, deg_to_rad(225.0));
    TipState p8(-300.0, y, 0.0, deg_to_rad(225.0));
    TipState p9(-500.0, y, 0.0, deg_to_rad(225.0));

//    TipState p4(500.0, y, 0.0, deg_to_rad(45.0));
//    TipState p5(300.0, y, 0.0, deg_to_rad(45.0));
//    TipState p6(100.0, y, 0.0, deg_to_rad(225.0));
//    TipState p7(-100.0, y, 0.0, deg_to_rad(225.0));
//    TipState p8(-300.0, y, 0.0, deg_to_rad(225.0));
//    TipState p9(-500.0, y, 0.0, deg_to_rad(225.0));

    std::vector<TipStates> vectors = {
            {p4},
            {p5},
            {p6},
            {p7},
            {p8},
            {p9},
    };
    if(is_red){
        for(auto& vec: vectors){
            for(auto& tmp: vec) {
                tmp.x *= -1.0;
                tmp.theta *= -1.0;
            }
        }
    }
    return PositionSelector(vectors);
}

PositionSelector get_position_selector_common(bool is_red) {
    double y = 842.5;
    TipState common1(420.0, y, 0.0, deg_to_rad(90.0));
    TipState common2(0, y, 0.0, deg_to_rad(90.0));
    TipState common3(-420.0, y, 0.0, deg_to_rad(90.0));

    std::vector<TipStates> vectors = {
            {common1},
            {common2},
            {common3},
    };
    if(is_red) {
        for (auto &vec: vectors) {
            for (auto &tmp: vec) {
                tmp.x *= -1.0;
                tmp.theta *= -1.0;
            }
        }
    }
    return PositionSelector(vectors);
}
