//
// Created by emile on 23/09/02.
//

#include <main_arm_controller/arm_trajectory_service.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <catch23_robot_controller/msg/arm_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <main_arm_controller/trajectory/r_theta_optimal_planning.hpp>
#include <main_arm_controller/utils/robot_state.hpp>


namespace arm_trajectory{

    ArmTrajectoryService::ArmTrajectoryService(const rclcpp::NodeOptions &options):Node("traj_service", options) {
        using namespace std::placeholders;
        auto bind_callback = std::bind(&ArmTrajectoryService::srv_callback, this, _1, _2);
        arm_trajectory_service = this->create_service<arm_traj_srv>("arm_trajectory_service", bind_callback);

    }

    void ArmTrajectoryService::srv_callback(std::shared_ptr<arm_traj_srv::Request> request,
                                            std::shared_ptr<arm_traj_srv::Response> response) {
        if(request->waypoints.size() != 2){
            response->is_feasible = false;
            response->trajectory.clear();
            return;
        }
        auto [traj, is_feasible] = plan(convert_tip_state(request->waypoints[0]), convert_tip_state(request->waypoints[1]), 15.0f);
        if(!is_feasible){
            response->is_feasible = is_feasible;
            response->trajectory.clear();
            return;
        }
        std::transform(traj.begin(), traj.end(), std::back_inserter(response->trajectory),
                       [](const auto& tmp){ return convert_arm_state(tmp); });
        response->is_feasible = true;
    }

    ArmTrajectoryService::~ArmTrajectoryService(){}
};


RCLCPP_COMPONENTS_REGISTER_NODE(arm_trajectory::ArmTrajectoryService)