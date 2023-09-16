//
// Created by emile on 23/09/09.
//

#ifndef ROS2_WS_R_THETA_OPTIMAL_PLANNING_HPP
#define ROS2_WS_R_THETA_OPTIMAL_PLANNING_HPP

#include <main_arm_controller/utils/robot_state.hpp>
#include <utility>
#include <vector>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Cost.h>
#include "ompl/util/Console.h"
#include <boost/program_options.hpp>
#include <memory>
#include <fstream>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;

using XY = std::tuple<double, double>;
XY rot2D(XY xy, double theta);
XY operator+ (const XY& a, const XY& b);



enum PlannerType{
    PLANNER_AITSTAR,
    PLANNER_BFMTSTAR,
    PLANNER_BITSTAR,
    PLANNER_CFOREST,
    PLANNER_FMTSTAR,
    PLANNER_INF_RRTSTAR,
    PLANNER_PRMSTAR,
    PLANNER_RRTSTAR,
    PLANNER_SORRTSTAR,
};


class ValidityCheckerRobotArea : public ob::StateValidityChecker{  // state spaceのvalidity checker
    ob::SpaceInformationPtr space_info;
    double hand_h = 390;
    double hand_w = 58.0 * 2.0;

//    double max_field_x = 1005.0 / 2.0 ;
    double max_field_x = 1800.0 / 2.0 ;
    double max_field_y_up = 610.0;
    double min_field_y_lw = -1045.0 ;
    double max_shooter_x = (1560 + 200.0) / 2.0;

    double _clearance_field_area(const XY& vertex) const;
public:
    explicit ValidityCheckerRobotArea(const ob::SpaceInformationPtr& space_info_);

    double clearance(const ob::State* state) const override;

    // 与えられた状態の位置が円形の障害物に重なっているかどうかを返す。
    bool isValid(const ob::State* state) const override;
};


// 共通エリア有り
class ValidityCheckerRobotAreaCommon : public ob::StateValidityChecker{  // state spaceのvalidity checker
    ob::SpaceInformationPtr space_info;
    double hand_h = 390;
    double hand_w = 58.0 * 2.0;

//    double max_field_x = 1005.0 / 2.0 ;
    double max_field_x = 1800.0 / 2.0 ;
    double max_field_y_up = 830.0 + 150.0 - 50.0;
    double min_field_y_lw = -1045.0 ;
    double max_shooter_x = (1560 + 200.0) / 2.0;

    double _clearance_field_area(const XY& vertex) const;
public:
    explicit ValidityCheckerRobotAreaCommon(const ob::SpaceInformationPtr& space_info_);

    double clearance(const ob::State* state) const override;

    // 与えられた状態の位置が円形の障害物に重なっているかどうかを返す。
    bool isValid(const ob::State* state) const override;
};



class ClearanceObjective : public ob::StateCostIntegralObjective{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& space_info);

    ob::Cost stateCost(const ob::State* s) const override;
};

class TipPathObjective : public ob::StateCostIntegralObjective{
public:
    TipPathObjective(const ob::SpaceInformationPtr& space_info);

    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;
};

class ParamLengthObjective : public ob::StateCostIntegralObjective{
private:
//    double d_theta = 0.08;
    double d_theta = 0.2;
    double d_r = 1000.0;
    double d_phi = 0.2;

public:
    ParamLengthObjective(const ob::SpaceInformationPtr& space_info);

    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override;
};



class OMPL_PlannerClass{
private:
    std::shared_ptr<ompl::base::SpaceInformation> _space_info_our_area;  // 共通エリアなし
    std::shared_ptr<ompl::base::SpaceInformation> _space_info_common;  // 共通エリア有り
public:
    OMPL_PlannerClass();
    std::pair<std::vector<ArmState>, bool> plan(const TipState& start_tip, const TipState& goal_tip, double l_min, double l_max, double d_max, bool is_common=false);
    // is_feasible (trueなら問題ない)
};


#endif //ROS2_WS_R_THETA_OPTIMAL_PLANNING_HPP
