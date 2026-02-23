#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>


using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
using MoveItErrorCode = moveit::core::MoveItErrorCode;
class MoveitCommander{
    public:
        MoveitCommander(std::shared_ptr<rclcpp::Node> node) : rate(1.0/3.0) {
            this->node = node; // we do this we have to pass a node to the MoveGroupInterface and when
            // the class inherits the Node class we need to use this, somehow this gives errors. So instead of inheritance we will use composition
            if (!node->has_parameter("robot_description_kinematics.arm.kinematics_solver")) {
                node->declare_parameter("robot_description_kinematics.arm.kinematics_solver",
                                        "kdl_kinematics_plugin/KDLKinematicsPlugin");
                node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_search_resolution", 0.005);
                node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_attempts", 10);
                node->declare_parameter("robot_description_kinematics.arm.kinematics_solver_timeout", 0.2);
            }


            this->arm = std::make_shared<MoveGroupInterface>(this->node, "arm");
            this->arm->setMaxAccelerationScalingFactor(0.7);
            this->arm->setMaxVelocityScalingFactor(0.7);
            this->arm->setPlanningTime(10.0);
            this->arm->setNumPlanningAttempts(10);
            this->arm->setGoalPositionTolerance(0.01);
            this->arm->setGoalOrientationTolerance(0.1);
            this->arm->allowReplanning(true);

            this->gripper = std::make_shared<MoveGroupInterface>(this->node, "gripper");
        }   

        bool namedGoal(const std::string &goal){
            this->log_info("REcevied named goal");
            this->arm->setStartStateToCurrentState();
            this->arm->setNamedTarget(goal);
            if(this->planAndExecute(this->arm)) return true;
            else return false;
        }

        bool jointGoal(const std::vector<double> &joints){
            this->log_info("REcevied joint goal");

            this->arm->setStartStateToCurrentState();
            this->arm->setJointValueTarget(joints);
            if(this->planAndExecute(this->arm)) return true;
            else return false;
        }

        bool poseGoal(const geometry_msgs::msg::PoseStamped &pose, bool cartesian_path = false){
            this->arm->setStartStateToCurrentState();
            
            if(!cartesian_path){
                this->log_info("REcevied pose goal");

                this->arm->setPoseTarget(pose);
                if(this->planAndExecute(this->arm)) return true;
                else return false;   
            }
           else{
                this->log_info("Got cartesian path pose!");
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(pose.pose);
                moveit_msgs::msg::RobotTrajectory trajectory;

                double fraction = this->arm->computeCartesianPath(waypoints,0.01,0.0,trajectory);
                RCLCPP_INFO(
                    this->node->get_logger(),
                    "Cartesian fraction=%.3f, points=%zu",
                    fraction, trajectory.joint_trajectory.points.size());

                if(fraction > 0.99 && trajectory.joint_trajectory.points.size() > 1){
                    this->log_info("Found valid trajectory executing cartesian path");
                    MoveItErrorCode exec_rc = this->arm->execute(trajectory);
                    if (exec_rc == MoveItErrorCode::SUCCESS) {
                        return true;
                    }
                    this->log_error("Cartesian execution failed!");
                    return false;
                }
                else{
                    this->log_error("No valid trajectory!");
                    return false;
                }
           }
        }

        bool openGripper(){
            this->gripper->setStartStateToCurrentState();
            this->gripper->setNamedTarget("open");
            return this->planAndExecute(this->gripper);
        }

        bool closeGripper(){
            this->gripper->setStartStateToCurrentState();
            this->gripper->setNamedTarget("close");
            return this->planAndExecute(this->gripper);
        }
    private:
        std::shared_ptr<rclcpp::Node> node;
        std::shared_ptr<MoveGroupInterface> arm;
        std::shared_ptr<MoveGroupInterface> gripper;
        std::function<void(const char*)> log_info = [this](const char *msg){
            RCLCPP_INFO(this->node->get_logger(), "%s", msg);
        };

        std::function<void(const char *)> log_error = [this](const char *msg){
            RCLCPP_ERROR(this->node->get_logger(), "%s", msg);
        };
        rclcpp::Rate rate;

        bool planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface){
            this->log_info("Received plan!");
            Plan plan;
            bool success = (interface->plan(plan) == MoveItErrorCode::SUCCESS);

            if (success){
                this->log_info("Planning successful, executing goal!");
                MoveItErrorCode exec_rc = interface->execute(plan);
                if (exec_rc == MoveItErrorCode::SUCCESS) {
                    return true;
                }
                this->log_error("Execution failed!");
                return false;
            }
            else{
                this->log_error("Planning has failed!");
                return false;
            }

        }

        
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = MoveitCommander(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){executor.spin();});

    rclcpp::Rate rate (1/3.0);
    commander.namedGoal("pose_1");
    rate.sleep();
    
    // std::vector<double> joints = {1.5, 0.5, 0.0, 1.5, 0.0, -0.7};
    // commander.jointGoal(joints);
    // rate.sleep();

    // tf2::Quaternion q;
    // q.setRPY(0, 0, 0);
    // q = q.normalize();
    // geometry_msgs::msg::PoseStamped pose;
    // pose.header.frame_id = "base_link";
    // pose.pose.position.x = 0.587695;
    // pose.pose.position.y = 0.0;
    // pose.pose.position.z = 0.238066;

    // pose.pose.orientation.x = 0.0;
    // pose.pose.orientation.y = 0.963630;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 0.267238;
    // commander.poseGoal(pose);
    // rate.sleep();

    // geometry_msgs::msg::PoseStamped pose_1 = pose;
    // pose_1.pose.position.z += -0.05;
    // commander.poseGoal(pose_1, true);
    // geometry_msgs::msg::PoseStamped  pose_2 = pose_1;
    // pose_2.pose.position.y += 0.05;
    // commander.poseGoal(pose_2, true);
    // geometry_msgs::msg::PoseStamped  pose_3 = pose_2;
    // pose_3.pose.position.y += -0.05;
    // pose_3.pose.position.z += 0.05;
    // commander.poseGoal(pose_3, true);
    // // rate.sleep();

    commander.openGripper();
    rate.sleep();
    bool ok = commander.closeGripper();
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "Final command failed (plan or execute).");
    }

    // Give the executor a short moment to process final result/status callbacks before shutdown.
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    rclcpp::shutdown();
    spinner.join();
    return ok ? 0 : 1;
}
