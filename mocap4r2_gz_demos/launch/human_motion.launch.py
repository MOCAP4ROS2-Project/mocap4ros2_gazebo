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

"""Human motion demo: a walking mannequin tracked as a skeleton, joint angles, RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mocap4r2_gz_demos')
    gz = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'human_motion.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            os.path.join(gz, 'launch', 'gz_sim.launch.py'),
            launch_arguments={'gz_args': f'-r {world}'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='mocap4r2_motion_tools', executable='skeleton_publisher',
             output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='mocap4r2_marker_viz', executable='marker_viz', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(condition=IfCondition(LaunchConfiguration('rviz')),
             package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'demo.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
