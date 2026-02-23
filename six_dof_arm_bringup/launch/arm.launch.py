import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ---- Launch args (paths) ----
    urdf_xacro_arg = DeclareLaunchArgument(
        "urdf_xacro",
        default_value=PathJoinSubstitution(
            [FindPackageShare("six_dof_arm"), "urdf", "arm.urdf.xacro"]
        ),
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("six_dof_arm_bringup"), "config", "moveit.rviz"]
        ),
    )

    controllers_yaml_arg = DeclareLaunchArgument(
        "controllers_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("six_dof_arm_bringup"), "config", "ros2_controllers.yaml"]
        ),
    )

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    launch_rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true")

    urdf_xacro = LaunchConfiguration("urdf_xacro")
    rviz_config = LaunchConfiguration("rviz_config")
    controllers_yaml = LaunchConfiguration("controllers_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # ---- robot_description from *your* xacro (shared by ALL nodes) ----
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", urdf_xacro]),
            value_type=str,
        )
    }

    # ---- Build MoveIt config (loads kinematics.yaml from your moveit_config pkg) ----
    moveit_config = (
        MoveItConfigsBuilder("six_dof_arm", package_name="six_dof_arm_moveit_config")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # ---- robot_state_publisher ----
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # ---- ros2_control controller manager ----
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---- Spawners ----
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    # ---- move_group ----
    # Pass full moveit_config, then OVERRIDE robot_description to ensure it's identical to RSP/ros2_control
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,  # override robot_description from our xacro
            {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True},
        ],
    )

    # ---- RViz ----
    # You asked: "send all the moveit_config to it" -> pass moveit_config.to_dict()
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        condition=None,  # keep simple; if you want conditional, I can add IfCondition(launch_rviz)
        parameters=[
            moveit_config.to_dict(),
            robot_description,  # ensure RViz sees the same robot_description too
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            urdf_xacro_arg,
            rviz_config_arg,
            controllers_yaml_arg,
            use_sim_time_arg,
            launch_rviz_arg,
            rsp_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            move_group_node,
            rviz_node,
        ]
    )