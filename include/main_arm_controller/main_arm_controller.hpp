//
// Created by emile on 23/06/09.
//

#ifndef ROS2_WS_MAIN_ARM_CONTROLLER_HPP
#define ROS2_WS_MAIN_ARM_CONTROLLER_HPP

#include <functional>
#include <algorithm>
#include <chrono>
#include <iterator>
#include <queue>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "main_arm_controller/visibility.hpp"
#include <kondo_drivers/msg/b3m_servo_msg.hpp>
#include <actuator_msgs/msg/actuator_msg.hpp>
#include <main_arm_controller/utils/robot_state.hpp>
#include <kondo_drivers/srv/kondo_b3m_srv.hpp>
#include <catch23_robot_controller/srv/arm_trajectory_srv.hpp>
#include <main_arm_controller/utils/joystick_state.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/logger.hpp>
#include <main_arm_controller/utils/system_classes.hpp>
#include <catch23_robot_controller/msg/one_hand_request.hpp>
#include <catch23_robot_controller/msg/shooter_state.hpp>


namespace arm_controller{
    class ArmControllerNode : public rclcpp::Node {
    public:
        COMPOSITION_PUBLIC
        explicit ArmControllerNode(const rclcpp::NodeOptions & options);
        ~ArmControllerNode();
    private:
        using kondo_msg = kondo_drivers::msg::B3mServoMsg;
        using kondo_srv = kondo_drivers::srv::KondoB3mSrv;
        using actuator_msg = actuator_msgs::msg::ActuatorMsg;
        using traj_srv = catch23_robot_controller::srv::ArmTrajectorySrv;

        using shooter_msg = catch23_robot_controller::msg::ShooterState;
        using one_grab_msg = catch23_robot_controller::msg::OneHandRequest;

        // trajectory
        bool _request_trajectory_following(std::vector<TipState>& traj_target_points, bool is_common);  // 問題なく移行出来た場合にはtrueを返す
        void _trajectory_timer_callback();
        void _traj_service_future_callback(rclcpp::Client<traj_srv>::SharedFuture future);  // callback for async_send_request

        // send data
        void _b3m_init(uint8_t servo_id);
        void _send_request_arm_state(const ArmState& req_arm_state);
        void _request_hand_open_close(bool hand_close);
        void _hand_unit_timer_callback();
        void _hand_interval_open_close(bool hand_close, bool is_common);
        void _request_one_grab_start(bool force);
//        void _request_shooter_state(uint8_t shooter_state);
        shooter_msg _shooter_next_state();

        // change state
        bool _change_controller_state(ControllerState next_state);
        bool _change_planner_state(PlannerState next_state);
        bool _change_common_area_state(CommonAreaState next_state);
        bool _change_hand_unit_state(HandUnitState next_state);
        void _set_hand_motion(HandMotionType hand_motion);

        // ROS
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::Subscription<catch23_robot_controller::msg::TipState>::SharedPtr sub_tip_fb;
        rclcpp::Publisher<actuator_msg>::SharedPtr _pub_micro_ros;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pub_micro_ros_r, _pub_micro_ros_theta;
        rclcpp::Publisher<kondo_msg>::SharedPtr _pub_b3m;
        rclcpp::Publisher<one_grab_msg>::SharedPtr _pub_grab;
        rclcpp::Client<traj_srv>::SharedPtr _traj_client;
        rclcpp::TimerBase::SharedPtr _timer_planner, _timer_hand_unit;

        // states
        ControllerState _controller_state = ControllerState::CTRL_HUMAN;
        PlannerState _planner_state = PlannerState::PLANNER_WAITING;
        CommonAreaState _common_area_state = CommonAreaState::COMMON_AREA_DISABLE;  // 共通エリアの状態と妨害の状態を共通化してる
        HandUnitState _hand_unit_state = HandUnitState::HAND_WAIT;
        HandMotionType _hand_unit_motion_type = HandMotionType::MOTION_NULL;
        shooter_msg _shooter_state;
        JoyStickState joy_state;
        bool _is_color_red = false;

        // data
        MainArmState _requested_state, _feedback_state;
        TrajectoryData _trajectory_data;
        std::vector<TipState> _traj_target_points;
        bool _traj_enter_common_area_is_enable = false;
        PositionSelector _field_tip_pos, _common_tip_pos, _shooter_tip_pos;
        TimeCounter _time_counter_hand_motion;  // hand motion用

        const TipState _tip_state_origin = TipState(325.0f, 0.0f, 0.0f, 0.0f);  // 初期位置
    };
}

#endif //ROS2_WS_MAIN_ARM_CONTROLLER_HPP
