from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.actions import TimerAction

def generate_launch_description():
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    moveit_config = MoveItConfigsBuilder("jetcobot", package_name="moveit_jetcobot_pkg").to_moveit_configs()
    launch_package_path = moveit_config.package_path
    

    ld = LaunchDescription()
    
    joint_control_node = Node(
        package="hardware_jetcobot_pkg",
        executable="joint_control",
        name="joint_control_node",
        output="screen"
    )

    joint_state_switcher_node = Node(
        package="hardware_jetcobot_pkg",
        executable="joint_state_switcher",
        name="joint_state_switcher_node",
        output="screen"
    )
    
    ld.add_action(joint_control_node)
    ld.add_action(joint_state_switcher_node)
    
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    rsp_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/rsp.launch.py")
        ),
    )
    
    move_group_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/move_group.launch.py")
        ),
    )
    
    # Run Rviz and load the default config to see the state of the move_group node
    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/moveit_rviz.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    
    delayed_rsp_group_rviz_node = TimerAction(
        period=3.0,
        actions=[rsp_node, move_group_node, rviz_node],
    )
    
    ld.add_action(delayed_rsp_group_rviz_node)

    # Fake joint driver
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
    )
    
    delayed_ros2_control_node = TimerAction(
        period=5.0,
        actions=[ros2_control_node],
    )
    
    ld.add_action(delayed_ros2_control_node)
    
    spawner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/spawn_controllers.launch.py")
        ),
    )
    
    delayed_spawner_node = TimerAction(
        period=5.0,
        actions=[spawner_node],
    )
    
    ld.add_action(delayed_spawner_node)

    return ld
