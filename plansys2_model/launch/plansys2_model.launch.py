import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    directory = get_package_share_directory('plansys2_model')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
            'model_file': directory + '/pddl/domain.pddl',
            'problem_file': directory + '/pddl/problem.pddl',
            'namespace': namespace
        }.items())

    gantry_move_cmd = Node(
        package='plansys2_model',
        executable='gantry_move_action_node',
        name='gantry_move_action_node',
        namespace=namespace,
        output='screen',
    )
    gantry_calibrate_cmd = Node(
        package='plansys2_model',
        executable='gantry_calibrate_action_node',
        name='gantry_calibrate_action_node',
        namespace=namespace,
        output='screen',
    )
    gantry_lock_cmd = Node(
        package='plansys2_model',
        executable='gantry_lock_action_node',
        name='gantry_lock_action_node',
        namespace=namespace,
        output='screen',
    )
    gantry_unlock_cmd = Node(
        package='plansys2_model',
        executable='gantry_unlock_action_node',
        name='gantry_unlock_action_node',
        namespace=namespace,
        output='screen',
    )

    robot_move_cmd = Node(
        package='plansys2_model',
        executable='robot_move_action_node',
        name='robot_move_action_node',
        namespace=namespace,
        output='screen',
    )
    robot_mount_cmd = Node(
        package='plansys2_model',
        executable='robot_mount_action_node',
        name='robot_mount_action_node',
        namespace=namespace,
        output='screen',
    )
    robot_unmount_cmd = Node(
        package='plansys2_model',
        executable='robot_unmount_action_node',
        name='robot_unmount_action_node',
        namespace=namespace,
        output='screen',
    )

    goal_cmd = Node(
        package='plansys2_terminal',
        executable='plansys2_terminal',
        name='send_goal',
        arguments=[
            '--domain', directory + '/pddl/domain.pddl',
            '--problem', directory + '/pddl/problem.pddl',
            '--goal', '(and (at_robot robot1 B1) (mounted robot1 tool1) (calibrated gantry1) (locked gantry1))',
            '--plan'
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)
    ld.add_action(gantry_move_cmd)
    ld.add_action(gantry_calibrate_cmd)
    ld.add_action(gantry_lock_cmd)
    ld.add_action(gantry_unlock_cmd)
    ld.add_action(robot_move_cmd)
    ld.add_action(robot_mount_cmd)
    ld.add_action(robot_unmount_cmd)
    # ld.add_action(TimerAction(period=5.0, actions=[goal_cmd]))
    return ld
