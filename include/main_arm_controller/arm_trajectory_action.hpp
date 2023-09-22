//
// Created by emile on 23/09/23.
//

#ifndef ROS2_WS_ARM_TRAJECTORY_ACTION_HPP
#define ROS2_WS_ARM_TRAJECTORY_ACTION_HPP

#include <functional>
#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <main_arm_controller/visibility.hpp>
#include <catch23_robot_controller/srv/arm_trajectory_srv.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <main_arm_controller/utils/robot_state.hpp>
#include <main_arm_controller/trajectory/r_theta_optimal_planning.hpp>


namespace arm_trajectory_action{
    class ArmTrajectoryAction : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ArmTrajectoryAction(const rclcpp::NodeOptions & options);
        ~ArmTrajectoryAction();
    private:
        using arm_traj_srv = catch23_robot_controller::srv::ArmTrajectorySrv;
        rclcpp::Service<arm_traj_srv>::SharedPtr arm_trajectory_service;
        void srv_callback(const std::shared_ptr<arm_traj_srv::Request> request, const std::shared_ptr<arm_traj_srv::Response> response);

        OMPL_PlannerClass planner;
    };
}


#endif //ROS2_WS_ARM_TRAJECTORY_ACTION_HPP
