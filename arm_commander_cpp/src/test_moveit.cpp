#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    
    /* Why spin in a separate thread?
     * 
     * executor.spin() is a BLOCKING call - it runs indefinitely until shutdown.
     * If we called it in the main thread, the program would be stuck there and
     * never reach the MoveIt commands below. spin() does not create its own thread
     * thats why we assign a separate thread to spin() in order to not block the main
     * thread which will use the moveit API to move the arm, the worker thread will spin
     * the node to gather necessary information recieved from moveit
     * 
     * The node MUST spin to:
     * 1. Process callbacks from move_group action server (planning/execution feedback)
     * 2. Handle service responses (current state queries, TF lookups, etc.)
     * 3. Receive action results when planning/execution completes
     * 4. Keep ROS communication alive
     * 
     * By spinning in a separate thread:
     * - Background thread: Continuously processes ROS messages and callbacks
     * - Main thread: Remains free to call MoveIt commands (plan, execute, etc.)
     *   and wait for their synchronous responses
     * 
     * Without this separate thread, MoveGroupInterface would timeout waiting
     * for responses that could never be received.
     */
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();}); // spin in background thread
    
    // to move the arm we are goin to get the planing group for this we will use MoveGroupInterface
    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // named goal
    arm.setStartStateToCurrentState(); // important to set the current state; moveit will start planning from the current state
    arm.setNamedTarget("pose_1"); // this is just going to set the goal pose

    // now we will do planning and then do execute
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    bool success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS); // check if planning is successful

    if (success) {
        RCLCPP_INFO(node->get_logger(), "Planning successful! Executing plan...");
        arm.execute(plan1); // this will excute the successful plan
        RCLCPP_INFO(node->get_logger(), "Execution complete!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }

    // now we try to move the gripper
    auto gripper = moveit::planning_interface::MoveGroupInterface(node,"gripper");
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("close");

    moveit::planning_interface::MoveGroupInterface::Plan gripper_close;
    bool gripper_success = (gripper.plan(gripper_close) == moveit::core::MoveItErrorCode::SUCCESS);
    if (gripper_success){
        gripper.execute(gripper_close);
        RCLCPP_INFO(node->get_logger(), "Gripper is closed");
    }
    
    
    // now we want to go back to home
    rclcpp::Rate rate(1/5.0);
    RCLCPP_INFO(node->get_logger(), "Waiting before returning to home position...");
    rate.sleep();

    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    success = (arm.plan(plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success) arm.execute(plan2);

    // now we open the gripper
    gripper.setStartStateToCurrentState();
    gripper.setNamedTarget("open");

    moveit::planning_interface::MoveGroupInterface::Plan gripper_open;
    gripper_success = (gripper.plan(gripper_open) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(gripper_success){
        gripper.execute(gripper_open);
        RCLCPP_INFO(node->get_logger(),"Gripper is open");
    }

    
    rclcpp::shutdown();
    spinner.join();
    return 0;
}