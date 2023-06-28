cmake_minimum_required(VERSION 3.8)
project(catch23_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()  # packageから自動でfind_packageしてくれる


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
#file(GLOB ${PROJECT_NAME}_msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
#file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
#rosidl_generate_interfaces(
#        ${PROJECT_NAME}
#        ${${PROJECT_NAME}_msg_files}
#        ${${PROJECT_NAME}_srv_files}
#        DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
#)


## Component Settings
ament_auto_add_library(main_arm_controller_component SHARED src/main_arm_controller.cpp)
rclcpp_components_register_nodes(main_arm_controller_component "arm_controller::ArmControllerNode")
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(main_arm_controller_component "${cpp_typesupport_target}")



# package setting
ament_auto_package(
        INSTALL_TO_SHARE
        launch
#        params
)