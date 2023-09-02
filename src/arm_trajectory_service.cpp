//
// Created by emile on 23/09/02.
//

#include <main_arm_controller/arm_trajectory_service.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>


namespace arm_trajectory{

    ArmTrajectoryService::ArmTrajectoryService(const rclcpp::NodeOptions &options):Node("traj_service_component", options) {
        using namespace std::placeholders;
        auto bind_callback = std::bind(&ArmTrajectoryService::srv_callback, this, _1, _2);
        arm_trajectory_service = this->create_service<arm_traj_srv>("arm_trajectory_service", bind_callback);

    }

    void ArmTrajectoryService::srv_callback(std::shared_ptr<arm_traj_srv::Request> request,
                                            std::shared_ptr<arm_traj_srv::Response> response) {
        if(request->waypoints.size() < 2){
            response->is_feasible = false;
            response->trajectory.clear();
            return;
        }  // 解なし

    }
};


RCLCPP_COMPONENTS_REGISTER_NODE(arm_trajectory::ArmTrajectoryService)