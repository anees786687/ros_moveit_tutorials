#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
using Pose = geometry_msgs::msg::Pose;

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("joint_pose_goals", options);

    // This node runs standalone via `ros2 run`, so provide IK defaults locally.
    if (!node->has_parameter("robot_description_kinematics.arm.kinematics_solver")) {
    node->declare_parameter("robot_description_kinematics.arm.kinematics_solver",
                            "kdl_kinematics_plugin/KDLKinematicsPlugin");
    node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.005);
    node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_attempts", 10);
    node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_timeout", 0.2);
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxAccelerationScalingFactor(0.7);
    arm.setMaxVelocityScalingFactor(0.7);
    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(10);
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.1);
    arm.allowReplanning(true);

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    q = q.normalize();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = arm.getPlanningFrame();
    pose.pose.position.x = 0.587695;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.238066;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.963630;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.267238;

    RCLCPP_INFO(node->get_logger(), "Planning path!");

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success){
        RCLCPP_INFO(node->get_logger(), "Planning success, moving the arm!");
        arm.execute(plan);
    }

    // get cartesian 
    RCLCPP_INFO(node->get_logger(),"Doing cartesian paths now!");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose pose_1 = arm.getCurrentPose().pose;
    pose_1.position.z += -0.05;
    waypoints.push_back(pose_1);

    Pose pose_2 = pose_1;
    pose_2.position.y += 0.05;
    waypoints.push_back(pose_2);

    Pose pose_3 = pose_2;
    pose_3.position.y += -0.05;
    pose_3.position.z += 0.05;
    waypoints.push_back(pose_3);

    moveit_msgs::msg::RobotTrajectory trajectory;
    arm.setStartStateToCurrentState();
    double fraction = arm.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);


    if(fraction > 0){
        RCLCPP_INFO(node->get_logger(),"Found trajectory, executing");
        arm.execute(trajectory);
    }
    else{
        RCLCPP_ERROR(node->get_logger(),"Couldnt find trajectory!");
    }


    rclcpp::shutdown();
    spinner.join();
    return 0;




}