#!/usr/bin/env python3
# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare('dummy_robot_bringup').find('dummy_robot_bringup')
    urdf_file = os.path.join(pkg_share, 'launch', 'single_rrbot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    rviz_config = os.path.join(
        get_package_share_directory('dummy_robot_test'),
        'conf',
        'dummy_robot.rviz'
        )    
    print(rviz_config)
    return LaunchDescription([
        Node(package='dummy_map_server', executable='dummy_map_server', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[rsp_params]),
        Node(package='dummy_sensors', executable='dummy_laser', output='screen'),
        Node(
            package='dummy_robot_test', 
            executable='dummy_joint_states', 
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
