#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
#This file is the launch file on REAL ROBOT, other one is for simulation!!




def generate_launch_description():

    # ── SAME AS BEFORE ──────────────────────────
    use_sim_time = LaunchConfiguration(
        'use_sim_time', default='false')  # ← FALSE on real robot!

    # ── ROBOT STATE PUBLISHER ───────────────────
    # still needed — publishes robot's TF transforms
    # so ROS2 knows robot's coordinate frames
    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir,
                'robot_state_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # ── YOUR NODES ──────────────────────────────
    # launch all your nodes automatically
    # so you dont need 6 separate terminals!

    grid_node = Node(
        package='my_tb3_world',
        executable='grid_node.py',
        name='grid_node',
        output='screen'
    )

    mux_node = Node(
        package='my_tb3_world',
        executable='mux.py',
        name='mux',
        output='screen'
    )

    twin_core_node = Node(
        package='my_tb3_world',
        executable='twin_core.py',
        name='twin_core',
        output='screen'
    )

    sensor_node = Node(
        package='my_tb3_world',
        executable='sensor_node.py',
        name='sensor_node',
        output='screen'
    )

    plant_monitor_node = Node(
        package='my_tb3_world',
        executable='plant_monitor.py',
        name='plant_monitor',
        output='screen'
    )

    # navigation launches last — small delay so others are ready
    navigation_node = Node(
        package='my_tb3_world',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen',
        # wait 5 seconds before starting navigation
        # so all other nodes are ready
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld = LaunchDescription()

    # add everything
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(grid_node)
    ld.add_action(mux_node)
    ld.add_action(twin_core_node)
    ld.add_action(sensor_node)
    ld.add_action(plant_monitor_node)
    ld.add_action(navigation_node)  # last!

    return ld
