//
// Created by emile on 23/09/23.
//

#include <main_arm_controller/arm_trajectory_action.hpp>
#include <catch23_robot_controller/msg/tip_state.hpp>
#include <catch23_robot_controller/msg/arm_state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>
#include <main_arm_controller/trajectory/r_theta_optimal_planning.hpp>
#include <main_arm_controller/utils/robot_state.hpp>
#include <main_arm_controller/utils/util_functions.hpp>



namespace arm_trajectory_action{
    const float min_r = 325.0f;
    const float max_r = 975.0f;
    const float min_theta = -M_PI;
    const float max_theta = M_PI;

    ArmState clip_arm_state(const ArmState& arm_state) {
        ArmState ans;
        ans.r = clip_f(arm_state.r, min_r, max_r);
        ans.theta = clip_f(arm_state.theta, min_theta, max_theta);
        ans.z = clip_f(arm_state.z, 0.0f, 225.0f);
        ans.phi = clip_f(arm_state.phi, deg_to_rad(-97.0f), deg_to_rad(110.0f));
        return ans;
    }



    ArmTrajectoryAction::ArmTrajectoryAction(const rclcpp::NodeOptions &options):Node("traj_action", options) {
        using namespace std::placeholders;
        this->arm_traj_action_server = rclcpp_action::create_server<traj_action>(
                this, "trajectory_action",
                std::bind(&ArmTrajectoryAction::handle_goal, this, _1, _2),
                std::bind(&ArmTrajectoryAction::handle_cancel, this, _1),
                std::bind(&ArmTrajectoryAction::handle_accepted, this, _1));

        _pub_b3m = this->create_publisher<kondo_msg>("b3m_topic", 10);
        _pub_micro_ros_r = this->create_publisher<std_msgs::msg::Float32>("mros_input_r", 5);
        _pub_micro_ros_theta = this->create_publisher<std_msgs::msg::Float32>("mros_input_theta", 5);
    }
    ArmTrajectoryAction::~ArmTrajectoryAction(){}

    rclcpp_action::GoalResponse ArmTrajectoryAction::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const traj_action::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "[TrajAction] Receive Trajectory Following Request");
        (void) uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ArmTrajectoryAction::handle_cancel(const std::shared_ptr<goal_handle_traj> goal_handle) {
        // serverにcancel要求が送信された時に呼び出される
        RCLCPP_INFO(this->get_logger(), "[TrajAction] Received Cancel Trajectory Following Request");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ArmTrajectoryAction::handle_accepted(const std::shared_ptr<goal_handle_traj> goal_handle) {
        // serverがゴールを受け入れて実行を開始したときに呼び出される
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&ArmTrajectoryAction::execute, this, _1), goal_handle}.detach();
    }

    void ArmTrajectoryAction::execute(const std::shared_ptr<goal_handle_traj> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "[TrajAction] Start Path Following");
        rclcpp::Rate loop_rate(10);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<traj_action::Feedback>();
        auto result = std::make_shared<traj_action::Result>();

        this->_convert_trajectory_data(goal->trajectory);  // set trajectory
        while(true) {
            if ((!this->_trajectory_data.complete()) && rclcpp::ok()) {  // 経路追従中
                if (goal_handle->is_canceling()) {  // cancel requestが来ているか確認
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "[TrajAction] Path Following canceled");
                    return;
                }
                this->_send_request_arm_state(this->_trajectory_data.get_front());
                feedback->dummy = true;
                goal_handle->publish_feedback(feedback);  // feedbackの送信
                loop_rate.sleep();
            }else{  // 経路追従完了
                break;
            }
        }

        if (rclcpp::ok()) {  // resultの処理
            result->is_success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "[TrajAction] Path Following succeeded");
        }
    }

    void ArmTrajectoryAction::_convert_trajectory_data(const std::vector<catch23_robot_controller::msg::ArmState>& traj){
        _trajectory_data.clear();
        std::vector<ArmState> tmp_traj;
        std::transform(traj.begin(), traj.end(), std::back_inserter(tmp_traj), [this](const auto& tmp){
            ArmState arm_state = convert_arm_state(tmp);
            if(clip_arm_state(arm_state) != arm_state){
                RCLCPP_WARN(this->get_logger(), "****[WARN]**** [TrajAction] trajectory clipped! : (%lf, %lf, %lf)", arm_state.r, arm_state.theta, arm_state.phi);
            }
            return clip_arm_state(arm_state);
        });
        _trajectory_data.set(tmp_traj);
    }

    void ArmTrajectoryAction::_send_request_arm_state(const ArmState& req_arm_state){
        uint8_t wrist_servo_id = 1;  // TODO: rosparam化
        std_msgs::msg::Float32 tgt_data;
        tgt_data.data = req_arm_state.r;
        _pub_micro_ros_r->publish(tgt_data);
        tgt_data.data = req_arm_state.theta;
        _pub_micro_ros_theta->publish(tgt_data);
        _pub_b3m->publish(gen_b3m_set_pos_msg(wrist_servo_id, -rad_to_deg(req_arm_state.phi), 0));
    }
};


RCLCPP_COMPONENTS_REGISTER_NODE(arm_trajectory_action::ArmTrajectoryAction)


