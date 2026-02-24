cmake_minimum_required(VERSION 3.8)
project(arm_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(six_dof_arm_interfaces REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface)


add_executable(send_joint_pose_goal src/send_joint_pose_goal.cpp)
ament_target_dependencies(send_joint_pose_goal rclcpp moveit_ros_planning_interface)

add_executable(cartesian_path src/cartesian_path.cpp)
ament_target_dependencies(cartesian_path rclcpp moveit_ros_planning_interface)

add_executable(moveit_oop src/moveit_oop.cpp)
ament_target_dependencies(moveit_oop rclcpp moveit_ros_planning_interface example_interfaces six_dof_arm_interfaces)

install(TARGETS
  test_moveit send_joint_pose_goal cartesian_path moveit_oop
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
