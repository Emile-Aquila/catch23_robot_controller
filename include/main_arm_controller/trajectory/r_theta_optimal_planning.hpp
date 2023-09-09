//
// Created by emile on 23/09/09.
//

#ifndef ROS2_WS_R_THETA_OPTIMAL_PLANNING_HPP
#define ROS2_WS_R_THETA_OPTIMAL_PLANNING_HPP

#include <main_arm_controller/utils/robot_state.hpp>
#include <utility>
#include <vector>

std::pair<std::vector<ArmState>, bool> plan(const TipState& start_tip, const TipState& goal_tip, double length);
// is_feasible (trueなら問題ない)

#endif //ROS2_WS_R_THETA_OPTIMAL_PLANNING_HPP
