# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Robot ground-truth demo: Gazebo as the mocap room, robot_gt, RViz."""

# system:=ros        the plugin is a MOCAP4ROS2 system
# system:=optitrack  the plugin plays OptiTrack Motive and the real OptiTrack driver
#                    connects to it (as in the lab)

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def world_for(context):
    system = LaunchConfiguration('system').perform(context)
    if system not in ('ros', 'optitrack'):
        raise RuntimeError("system must be 'ros' or 'optitrack'")
    pkg = get_package_share_directory('mocap4r2_gz_demos')
    with open(os.path.join(pkg, 'worlds', 'robot_groundtruth.sdf')) as f:
        sdf = f.read().replace('OUTPUT', 'natnet' if system == 'optitrack' else 'ros')
    world = os.path.join(tempfile.gettempdir(), f'mocap4r2_robot_groundtruth_{system}.sdf')
    with open(world, 'w') as f:
        f.write(sdf)
    gz = get_package_share_directory('ros_gz_sim')
    actions = [IncludeLaunchDescription(
        os.path.join(gz, 'launch', 'gz_sim.launch.py'),
        launch_arguments={'gz_args': f'-r {world}'}.items())]
    if system == 'optitrack':
        actions.append(Node(
            package='mocap4r2_optitrack_driver', executable='optitrack_driver',
            name='optitrack_driver', output='screen',
            parameters=[{'server_address': '127.0.0.1', 'local_address': '127.0.0.1',
                         'connection_type': 'unicast', 'up_axis': 'y',
                         'autostart': 'activate', 'use_sim_time': True}]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('mocap4r2_gz_demos')
    return LaunchDescription([
        DeclareLaunchArgument('system', default_value='ros',
                              description='ros or optitrack (simulated Motive)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=world_for),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='mocap4r2_robot_gt', executable='robot_gt', name='robot_gt',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'robot_gt.yaml'), {'use_sim_time': True}]),
        Node(package='mocap4r2_marker_viz', executable='marker_viz', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(condition=IfCondition(LaunchConfiguration('rviz')),
             package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'demo.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
