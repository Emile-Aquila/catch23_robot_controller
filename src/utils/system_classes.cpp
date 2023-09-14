//
// Created by emile on 23/09/10.
//

#include <main_arm_controller/utils/system_classes.hpp>
#include <iostream>



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
PositionSelector::PositionSelector(std::vector<TipStates> &tip_states_list) {
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


PositionSelector get_position_selector_shooter(bool is_red){
    // TODO: 実装
    return PositionSelector();
}


PositionSelector get_position_selector_targets(bool is_red){
    // TODO: 実装
    return PositionSelector();
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
