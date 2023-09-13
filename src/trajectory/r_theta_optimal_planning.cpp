//
// Created by emile on 23/08/15.
//

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>

// For boost program options
#include <boost/program_options.hpp>
#include <memory>
#include <fstream>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>


#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <utility>
#include <fstream>
#include <limits>

#include <main_arm_controller/trajectory/r_theta_optimal_planning.hpp>
#include <main_arm_controller/trajectory/spline.h>
#include <main_arm_controller/trajectory/spline_utils.hpp>
#include <main_arm_controller/utils/robot_state.hpp>
#include <main_arm_controller/utils/util_functions.hpp>


template<class T> using matrix= std::vector<std::vector<T>>;


ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr space_info, PlannerType plannerType){
    switch (plannerType){
        case PLANNER_AITSTAR:
            return std::make_shared<og::AITstar>(space_info);
            break;
        case PLANNER_BFMTSTAR:
            return std::make_shared<og::BFMT>(space_info);
            break;
        case PLANNER_BITSTAR:
            return std::make_shared<og::BITstar>(space_info);
            break;
        case PLANNER_CFOREST:
            return std::make_shared<og::CForest>(space_info);
            break;

        case PLANNER_FMTSTAR:
            return std::make_shared<og::FMT>(space_info);
            break;

        case PLANNER_INF_RRTSTAR:
            return std::make_shared<og::InformedRRTstar>(space_info);
            break;
        case PLANNER_PRMSTAR:
            return std::make_shared<og::PRMstar>(space_info);
            break;
        case PLANNER_RRTSTAR:
            return std::make_shared<og::RRTstar>(space_info);
            break;
        case PLANNER_SORRTSTAR:
            return std::make_shared<og::SORRTstar>(space_info);
            break;
        default:
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
    }
}


std::tuple<double, double, double> FK(const ob::State* state){  // r, theta, phi
    const auto *r_vec = state->as<ob::RealVectorStateSpace::StateType>();
    const double *tmp = r_vec->values;
    auto [theta, r, phi] = std::make_tuple(tmp[0], tmp[1], tmp[2]);
    return std::make_tuple(r*cos(theta), r*sin(theta), theta+phi);
}


XY rot2D(XY xy, double theta){
    auto [x, y] = xy;
    return std::make_tuple(x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta));
}
XY operator+ (const XY& a, const XY& b){
    return {std::get<0>(a) + std::get<0>(b), std::get<1>(a) + std::get<1>(b)};
}



std::vector<XY> rectangle_vertexes(double w, double h, double x, double y, double theta){
    std::vector<XY> tmp = {{w/2.0, h/2.0}, {-w/2.0, h/2.0}, {-w/2.0, -h/2.0}, {w/2.0, -h/2.0}};
    std::for_each(tmp.begin(), tmp.end(), [theta, x, y](auto& point){ point = rot2D(point, theta)+XY(x, y);});
    return tmp;
}


ValidityCheckerRobotArea::ValidityCheckerRobotArea(const ob::SpaceInformationPtr& space_info_): ob::StateValidityChecker(space_info_){
    space_info = space_info_;
}


double ValidityCheckerRobotArea::_clearance_field_area(const XY& vertex) const {
    auto [x,y] = vertex;
    double ans = std::min({max_field_x - abs(x), max_field_y_up - y, y - min_field_y_lw});
    if(y <= 0.0)chmin(ans, max_shooter_x - x);
    return ans;
}

double ValidityCheckerRobotArea::clearance(const ob::State* state) const {
    // 与えられた状態の位置から円形の障害物の境界までの距離を返す。
    auto [x, y, yaw] = FK(state);
    std::vector<XY> vertexes = rectangle_vertexes(hand_w, hand_h, x, y, yaw);
    double min_clearance = std::numeric_limits<double>::max();
    for (const auto &tmp: vertexes) {
        chmin(min_clearance, _clearance_field_area(tmp));
    }
    return min_clearance;
}

    // 与えられた状態の位置が円形の障害物に重なっているかどうかを返す。
bool ValidityCheckerRobotArea::isValid(const ob::State* state) const {
    return this->clearance(state) > 0.0 && this->space_info->satisfiesBounds(state);
}



// 共通エリア有りの方
ValidityCheckerRobotAreaCommon::ValidityCheckerRobotAreaCommon(const ob::SpaceInformationPtr& space_info_): ob::StateValidityChecker(space_info_){
    space_info = space_info_;
}

double ValidityCheckerRobotAreaCommon::_clearance_field_area(const XY& vertex) const {
    auto [x,y] = vertex;
    double ans = std::min({max_field_x - abs(x), max_field_y_up - y, y - min_field_y_lw});
    if(y <= 0.0)chmin(ans, max_shooter_x - x);
    return ans;
}

double ValidityCheckerRobotAreaCommon::clearance(const ob::State* state) const {
    // 与えられた状態の位置から円形の障害物の境界までの距離を返す。
    auto [x, y, yaw] = FK(state);
    std::vector<XY> vertexes = rectangle_vertexes(hand_w, hand_h, x, y, yaw);
    double min_clearance = std::numeric_limits<double>::max();
    for (const auto &tmp: vertexes) {
        chmin(min_clearance, _clearance_field_area(tmp));
    }
    return min_clearance;
}

// 与えられた状態の位置が円形の障害物に重なっているかどうかを返す。
bool ValidityCheckerRobotAreaCommon::isValid(const ob::State* state) const {
    return this->clearance(state) > 0.0 && this->space_info->satisfiesBounds(state);
}






ClearanceObjective::ClearanceObjective(const ob::SpaceInformationPtr& space_info)
    :ob::StateCostIntegralObjective(space_info, true){
}

ob::Cost ClearanceObjective::stateCost(const ob::State* s) const {
    // 障害物との距離を最大化する
    return ob::Cost(log(1 / (si_->getStateValidityChecker()->clearance(s) + std::numeric_limits<double>::min())));
}





TipPathObjective::TipPathObjective(const ob::SpaceInformationPtr& space_info)
    :ob::StateCostIntegralObjective(space_info, true){};

ob::Cost TipPathObjective::motionCost(const ob::State* s1, const ob::State* s2) const {
    auto [x1, y1, yaw1] = FK(s1);
    auto [x2, y2, yaw2] = FK(s2);
    return ob::Cost(sqrt(pow(x2 - x1 / 1000, 2.0) + pow(y2 - y1 / 1000, 2.0)));
}




ParamLengthObjective::ParamLengthObjective(const ob::SpaceInformationPtr& space_info) :
    ob::StateCostIntegralObjective(space_info, true){}

ob::Cost ParamLengthObjective::motionCost(const ob::State* s1, const ob::State* s2) const {
    const auto *tmp = s1->as<ob::RealVectorStateSpace::StateType>()->values;
    auto [theta, r, phi] = std::make_tuple(tmp[0], tmp[1], tmp[2]);
    const auto *tmp2 = s2->as<ob::RealVectorStateSpace::StateType>()->values;
    auto [theta2, r2, phi2] = std::make_tuple(tmp2[0], tmp2[1], tmp2[2]);
    return ob::Cost(abs(theta2 - theta) / d_theta + abs(r2 - r) / d_r + abs(phi2 - phi) / d_phi);
}



ob::OptimizationObjectivePtr getBalancedObjective1(const ob::SpaceInformationPtr& si){
    auto trajObj(std::make_shared<TipPathObjective>(si));
    auto paramlengthObj(std::make_shared<ParamLengthObjective>(si));
    auto clearObj(std::make_shared<ClearanceObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));  // 目的関数の合成

    std::function<ob::Cost(const ob::State*, const ob::Goal*)> h_func = [](const ob::State *s, const ob::Goal *g){
        auto goal = g->getSpaceInformation()->allocState();
        auto[x1, y1, yaw1] = FK(s);
        auto[x2, y2, yaw2] = FK(goal);
        return ob::Cost((pow(x2-x1, 2.0) + pow(y2-y1, 2.0)));
    };
    opt->setCostToGoHeuristic(h_func);
    opt->addObjective(paramlengthObj, 1.0);
//    opt->addObjective(trajObj, 5.0);
//    opt->addObjective(clearObj, 0.05);

    return ob::OptimizationObjectivePtr(opt);
}

class MotionValidatorChecker : public ob::MotionValidator{  // state spaceのvalidity checker
    ob::SpaceInformationPtr space_info;
public:
    explicit MotionValidatorChecker(const ompl::base::SpaceInformationPtr &space_info_) : MotionValidator(space_info_) {
        space_info = space_info_;
    }

    bool checkMotion(const ob::State *s1, const ob::State* s2) const override{
        const auto *r_vec = s1->as<ob::RealVectorStateSpace::StateType>();
        const double *tmp = r_vec->values;
        auto [theta1, r1, phi1] = std::make_tuple(tmp[0], tmp[1], tmp[2]);

        const auto *r_vec2 = s2->as<ob::RealVectorStateSpace::StateType>();
        const double *tmp2 = r_vec2->values;
        auto [theta2, r2, phi2] = std::make_tuple(tmp2[0], tmp2[1], tmp2[2]);
        if(abs(theta2-theta1) < 1.5 && abs(phi2 - phi1) < 1.5)return true;
        else return false;
    }

    bool checkMotion(const ob::State *s1, const ob::State* s2, std::pair<ob::State*, double> &lastValid) const override{
        return checkMotion(s1, s2);
    }
};

OMPL_PlannerClass::OMPL_PlannerClass() {
    auto state_space(std::make_shared<ob::RealVectorStateSpace>(3));
    auto state_space_common(std::make_shared<ob::RealVectorStateSpace>(3));
    matrix<double> bounds_pre{
            std::vector<double>{-M_PI_2, M_PI*2.0 - deg_to_rad(10.0)}, // theta
            std::vector<double>{325.0, 975.0},  // r
            std::vector<double>{deg_to_rad(-97.0f), deg_to_rad(110.0f)},
    };

    ob::RealVectorBounds bounds(3), bounds_common(3);
    for(int i=0; i<bounds_pre.size(); i++){
        bounds.setLow(i, bounds_pre[i][0]);
        bounds.setHigh(i, bounds_pre[i][1]);
        bounds_common.setLow(i, bounds_pre[i][0]);
        bounds_common.setHigh(i, bounds_pre[i][1]);
    }
    state_space->setBounds(bounds);  // bounds for param
    state_space_common->setBounds(bounds);  // bounds for param

    _space_info_our_area = std::make_shared<ob::SpaceInformation>(state_space);
    _space_info_our_area->setStateValidityChecker(std::make_shared<ValidityCheckerRobotArea>(_space_info_our_area));
//    _space_info_our_area->setMotionValidator(std::make_shared<MotionValidatorChecker>(_space_info_our_area));
    _space_info_our_area->setup();
    _space_info_our_area->printSettings(std::cout);

    _space_info_common = std::make_shared<ob::SpaceInformation>(state_space_common);
    _space_info_common->setStateValidityChecker(std::make_shared<ValidityCheckerRobotAreaCommon>(_space_info_common));
    _space_info_common->setup();
    _space_info_common->printSettings(std::cout);


}

std::pair<std::vector<ArmState>, bool> OMPL_PlannerClass::plan(const TipState &start_tip, const TipState &goal_tip, double l_min, double l_max, double d_max, bool is_common) {
    auto state_space(std::make_shared<ob::RealVectorStateSpace>(3));
    auto space_info = _space_info_our_area;
    if(is_common)space_info = _space_info_common;

    // start point
    ob::ScopedState<> start(state_space);  // (theta, r, phi)
    ArmState start_tmp = arm_ik(start_tip);
    std::cout<<"start: x,y,theta -> " << start_tip.x <<", "<< start_tip.y <<", "<< start_tip.theta <<std::endl;
    std::cout<<"start: r,theta,phi -> " << start_tmp.r <<", "<< start_tmp.theta <<", "<< start_tmp.phi <<std::endl;
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_tmp.theta;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_tmp.r;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = start_tmp.phi;

    // goal point
    ob::ScopedState<> goal(state_space);  // (1, 1, 1)
    ArmState goal_tmp = arm_ik(goal_tip);
    std::cout<<"goal: x,y,theta -> " << goal_tip.x <<", "<< goal_tip.y <<", "<< goal_tip.theta <<std::endl;
    std::cout<<"goal: r,theta,phi -> " << goal_tmp.r <<", "<< goal_tmp.theta <<", "<< goal_tmp.phi <<std::endl;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_tmp.theta;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_tmp.r;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_tmp.phi;

    // problem definition
    auto prob_def(std::make_shared<ob::ProblemDefinition>(space_info));  // problem instance
    prob_def->setStartAndGoalStates(start, goal);
    prob_def->setOptimizationObjective(getBalancedObjective1(space_info));  // 目的関数の設定
//    prob_def->print(std::cout);  // 問題設定を表示

    auto planner = allocatePlanner(space_info, PLANNER_FMTSTAR);
    planner->setProblemDefinition(prob_def);  // problem instanceを代入
    planner->setup();  // plannerのsetup
    planner->checkValidity();

    ob::PlannerStatus solved = planner->ob::Planner::solve(0.5);
    if (!solved) {
        std::cout << "No solution found" << std::endl;
        return std::make_pair(std::vector<ArmState>{}, false);
    }

    auto path = std::static_pointer_cast<og::PathGeometric>(prob_def->getSolutionPath());
    if(path->getStates().size() <= 2){  // スプライン補間は3点以上必要
        path->interpolate(3);
    }
    path->interpolate(4);
//    path->checkAndRepair(10);

    std::vector<ArmState> traj;
    for(auto& tmp: path->getStates()){
        auto *tmp2 = (*tmp).as<ob::RealVectorStateSpace::StateType>()->values;
        auto [theta, r, phi] = std::make_tuple(tmp2[0], tmp2[1], tmp2[2]);
        traj.emplace_back(r, theta, 0.0, phi);
    }

    std::vector<ArmState> traj_pre = path_func(traj, 0.3);  // スプライン補間 (パラメータ空間)
    std::vector<ArmState> r_theta_trajectory = path_func_xy(traj_pre, l_min, l_max, d_max);  // スプライン補間 (xy)
    return std::make_pair(r_theta_trajectory, true);
//    return std::make_pair(traj, true);
}
