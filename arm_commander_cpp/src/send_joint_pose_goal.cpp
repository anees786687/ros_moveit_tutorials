#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

constexpr const char *GREEN = "\033[1;32m";
constexpr const char *RESET = "\033[0m";

int main(int argc, char **argv) {
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
    auto spinner = std::thread([&executor]() { executor.spin(); });


    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);
    arm.setPlanningTime(10.0);
    arm.setNumPlanningAttempts(10);
    arm.setGoalPositionTolerance(0.01);
    arm.setGoalOrientationTolerance(0.1);
    arm.allowReplanning(true);

    // Sending join goals
    RCLCPP_INFO(node->get_logger(), "%sSending pose goal using setJointValueTarget%s", GREEN, RESET);

    std::vector<double> joints = {1.5, 0.5, 0.0, 1.5, 0.0, -0.7};

    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    bool joint_plan_success = (arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(joint_plan_success) arm.execute(joint_plan);


    RCLCPP_INFO(node->get_logger(), "%sSending pose goal using setApproximateJointValeTarget%s", GREEN, RESET);

    const std::string ee_link = arm.getEndEffectorLink();
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", arm.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "EE link: %s", ee_link.c_str());

    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0); // neutral orientation first
    q = q.normalize();
    const auto current_pose = arm.getCurrentPose(ee_link);
    // geometry_msgs::msg::PoseStamped target_pose = current_pose;
    // target_pose.header.frame_id = arm.getPlanningFrame();
    // target_pose.pose.position.x += 0.0;   // forward, reachable
    // target_pose.pose.position.y += -0.07;   // centered
    // target_pose.pose.position.z += 0.04;   // moderate height
    // target_pose.pose.orientation.x = q.getX();
    // target_pose.pose.orientation.y = q.getY();
    // target_pose.pose.orientation.z = q.getZ();
    // target_pose.pose.orientation.w = q.getW();
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = arm.getPlanningFrame();
    target_pose.pose.position.x = 0.1;   // forward, reachable
    target_pose.pose.position.y = 0.0;   // centered
    target_pose.pose.position.z = 0.3;   // moderate height
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();


    arm.setStartStateToCurrentState();
    // Why use setApproximateJointValueTarget() here instead of setPoseTarget()?
    // - setPoseTarget() keeps the goal as an EE pose constraint (position + orientation).
    //   During planning, MoveIt must sample goal states and solve IK that matches the pose
    //   (within tolerances). If exact IK solutions are hard to find (unreachable pose,
    //   orientation too strict, near joint limits/singularity, or collision), planning fails.
    // - setApproximateJointValueTarget() runs IK first and converts the pose into a joint goal,
    //   allowing an approximate IK match (closest reachable pose). The planner then plans in
    //   joint space to that concrete joint target, which is often more robust.
    // Tradeoff: planning may succeed even if the final EE pose is only approximate.
    bool ik_ok = arm.setApproximateJointValueTarget(target_pose, ee_link);
    if (!ik_ok) {
    RCLCPP_WARN(node->get_logger(),
                "Requested pose is not reachable with current constraints. Falling back to current pose.");
    target_pose = current_pose;
    ik_ok = arm.setApproximateJointValueTarget(target_pose, ee_link);
    }

    if (!ik_ok) {
    RCLCPP_ERROR(node->get_logger(),
                    "Could not find an IK solution. Verify kinematics config and target pose.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
    }

    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    bool pose_success = (arm.plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (pose_success) {
    RCLCPP_INFO(node->get_logger(), "Pose planning succeeded. Executing...");
    arm.execute(pose_plan);
    } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed. Check collisions and controller status.");
    }

    RCLCPP_INFO(node->get_logger(), "%sSending pose goal using setPoseTarget%s", GREEN, RESET);
    target_pose.header.frame_id = arm.getPlanningFrame();  // likely "base_link"

    // target_pose.pose.position.x = 0.0227;
    // target_pose.pose.position.y = 0.3204;
    // target_pose.pose.position.z = 0.8130;

    // target_pose.pose.orientation.x = -0.0848;
    // target_pose.pose.orientation.y = 0.2324;
    // target_pose.pose.orientation.z = 0.8844;
    // target_pose.pose.orientation.w = 0.3958;


    // target_pose.pose.position.x = 0.044733;
    // target_pose.pose.position.y = 0.557740;
    // target_pose.pose.position.z = 0.289499;

    // target_pose.pose.orientation.x = -0.618571;
    // target_pose.pose.orientation.y = 0.751279;
    // target_pose.pose.orientation.z = 0.205149;
    // target_pose.pose.orientation.w = 0.104230;
    target_pose.pose.position.x = 0.023400;
    target_pose.pose.position.y = 0.329975;
    target_pose.pose.position.z = 0.855532;

    target_pose.pose.orientation.x = -0.084834;
    target_pose.pose.orientation.y = 0.232405;
    target_pose.pose.orientation.z = 0.884388;
    target_pose.pose.orientation.w = 0.395789;

    arm.setStartStateToCurrentState();
    arm.clearPoseTargets();
    arm.setPoseTarget(target_pose, ee_link);  // ee_link should be tool_link (or your active EE link)

    pose_success = (arm.plan(pose_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (pose_success) {
    RCLCPP_INFO(node->get_logger(), "Pose planning succeeded. Executing...");
    arm.execute(pose_plan);
    } else {
    RCLCPP_ERROR(node->get_logger(), "Planning failed. Check collisions and controller status.");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
