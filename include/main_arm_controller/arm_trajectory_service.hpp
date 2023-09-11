//
// Created by emile on 23/09/02.
//

#ifndef ROS2_WS_ARM_TRAJRCTORY_SERVICE_HPP
#define ROS2_WS_ARM_TRAJRCTORY_SERVICE_HPP


#include <functional>
#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <main_arm_controller/visibility.hpp>
#include <catch23_robot_controller/srv/arm_trajectory_srv.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <main_arm_controller/utils/robot_state.hpp>


namespace arm_trajectory{
    class ArmTrajectoryService : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ArmTrajectoryService(const rclcpp::NodeOptions & options);
        ~ArmTrajectoryService();
    private:
        using arm_traj_srv = catch23_robot_controller::srv::ArmTrajectorySrv;
        rclcpp::Service<arm_traj_srv>::SharedPtr arm_trajectory_service;
        void srv_callback(const std::shared_ptr<arm_traj_srv::Request> request, const std::shared_ptr<arm_traj_srv::Response> response);
    };
}


#endif //ROS2_WS_ARM_TRAJRCTORY_SERVICE_HPP
