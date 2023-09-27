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
#include <rclcpp_action/rclcpp_action.hpp>
#include <main_arm_controller/visibility.hpp>
#include <kondo_drivers/srv/kondo_b3m_srv.hpp>
#include "std_msgs/msg/float32.hpp"

#include <catch23_robot_controller/action/arm_trajectory_action.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <main_arm_controller/utils/robot_state.hpp>
#include <main_arm_controller/trajectory/r_theta_optimal_planning.hpp>
#include <main_arm_controller/utils/system_classes.hpp>


namespace arm_trajectory_action{
    class ArmTrajectoryAction : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ArmTrajectoryAction(const rclcpp::NodeOptions & options);
        ~ArmTrajectoryAction();
    private:
        using traj_action = catch23_robot_controller::action::ArmTrajectoryAction;
        using goal_handle_traj = rclcpp_action::ServerGoalHandle<traj_action>;

        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_micro_ros_r, _pub_micro_ros_theta;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_b3m;
        TrajectoryData _trajectory_data;

        rclcpp_action::Server<traj_action>::SharedPtr arm_traj_action_server;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const traj_action::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<goal_handle_traj> goal_handle);
        void handle_accepted(const std::shared_ptr<goal_handle_traj> goal_handle);
        void execute(const std::shared_ptr<goal_handle_traj> goal_handle);

        void _convert_trajectory_data(const std::vector<catch23_robot_controller::msg::ArmState>& traj);
        void _send_request_arm_state(const ArmState& req_arm_state);

        OMPL_PlannerClass planner;
    };
}


#endif //ROS2_WS_ARM_TRAJECTORY_ACTION_HPP
