//
// Created by emile on 23/09/10.
//

#include <main_arm_controller/utils/system_classes.hpp>



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