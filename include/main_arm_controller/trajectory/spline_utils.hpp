//
// Created by emile on 23/09/08.
//

#ifndef OMPL_TEST_SPLINE_UTILS_HPP
#define OMPL_TEST_SPLINE_UTILS_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>
#include <main_arm_controller/robot_state.hpp>
#include <main_arm_controller/trajectory/spline.h>


void create_time_grid(std::vector<double>& T, double& tmin, double& tmax,
                      std::vector<double>& X, std::vector<double>& Y, bool is_closed_curve);
std::vector<ArmState> path_func(const std::vector<ArmState>& waypoints, double length);


#endif //OMPL_TEST_SPLINE_UTILS_HPP
