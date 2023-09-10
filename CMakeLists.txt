cmake_minimum_required(VERSION 3.8)
project(catch23_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()  # packageから自動でfind_packageしてくれる

## required for ompl
find_package(Boost REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(ompl REQUIRED)
include_directories(
        ${EIGEN_INCLUDE_DIR}
        ${OMPL_INCLUDE_DIRS}
        ${Boost_LIBRARIES}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


## msg/srv generation
file(GLOB ${PROJECT_NAME}_msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_msg_files}
        ${${PROJECT_NAME}_srv_files}
        DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)


## Component Settings
ament_auto_add_library(main_arm_controller_component SHARED
        src/main_arm_controller.cpp src/utils/joystick_state.cpp src/utils/system_classes.cpp
        src/utils/robot_state.cpp src/utils/util_functions.cpp)
rclcpp_components_register_nodes(main_arm_controller_component "arm_controller::ArmControllerNode")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(main_arm_controller_component "${cpp_typesupport_target}")


ament_auto_add_library(arm_trajectory_service_component SHARED src/arm_trajectory_service.cpp
        src/utils/robot_state.cpp  src/trajectory/spline_utils.cpp src/trajectory/r_theta_optimal_planning.cpp)
rclcpp_components_register_nodes(arm_trajectory_service_component "arm_trajectory::ArmTrajectoryService")
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(arm_trajectory_service_component "${cpp_typesupport_target}" ${OMPL_LIBRARIES} ${Boost_LIBRARIES})


ament_auto_add_library(c620_test SHARED src/c620_test.cpp)
rclcpp_components_register_nodes(c620_test "arm_controller::C620Test")
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(c620_test "${cpp_typesupport_target}")


ament_auto_add_library(state_publisher_component SHARED src/state_publisher_node.cpp src/utils/robot_state.cpp)
rclcpp_components_register_nodes(state_publisher_component "arm_controller::StatePublisherNode")
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(state_publisher_component "${cpp_typesupport_target}")


# package setting
ament_auto_package(
        INSTALL_TO_SHARE
        launch
#        params
)