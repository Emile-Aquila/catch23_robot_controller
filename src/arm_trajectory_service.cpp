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

    void ArmTrajectoryService::srv_callback(const std::shared_ptr<arm_traj_srv::Request> request,
                                            const std::shared_ptr<arm_traj_srv::Response> response) {
        if(request->waypoints.size() < 2){
            RCLCPP_WARN(this->get_logger(), "[TrajService] invalid points num (points.size() < 2)");
            response->is_feasible = false;
            response->trajectory.clear();
            return;
        }

        std::vector<ArmState> ans_traj;
        bool is_feasible = true;
        for(size_t i=0; i<request->waypoints.size()-1; i++){
            TipState start = convert_tip_state(request->waypoints[i]), goal = convert_tip_state(request->waypoints[i+1]);
            auto [traj_tmp, is_feasible_tmp] = this->planner.plan(
                    start, goal,request->step_min, request->step_max, request->d_step_max, request->is_common);
            if(!is_feasible_tmp)is_feasible = false;
            ans_traj.insert(ans_traj.end(), traj_tmp.begin(), traj_tmp.end());
        }
        if(!is_feasible){
            response->is_feasible = is_feasible;
            response->trajectory.clear();
            return;
        }

        std::ofstream writing_file;
        try{
            writing_file.open("path.txt");
        }catch(const std::exception& e){
            std::cerr << "can't open path.txt" << std::endl;
        }
        for(auto& tmp: ans_traj){
            writing_file << tmp.theta << " " << tmp.r << " " << tmp.phi << std::endl;
        }
        writing_file.close();

        std::transform(ans_traj.begin(), ans_traj.end(), std::back_inserter(response->trajectory),
                       [](const auto& tmp){ return convert_arm_state(tmp); });
        response->is_feasible = true;
    }

    ArmTrajectoryService::~ArmTrajectoryService(){}
};


RCLCPP_COMPONENTS_REGISTER_NODE(arm_trajectory::ArmTrajectoryService)