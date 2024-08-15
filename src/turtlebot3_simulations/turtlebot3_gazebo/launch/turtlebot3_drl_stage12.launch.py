#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Ryan Shim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pause = LaunchConfiguration('pause', default='true')
    world_file_name = LaunchConfiguration('world_file_name', default='apartment.world').perform(context)
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', 'turtlebot3_drl_stage12', world_file_name)
    print(world)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    file = open('/tmp/drlnav_current_stage.txt', 'w')
    file.write("12\n")
    file.close()
    
    file = open('/tmp/drlnav_current_world.txt', 'w')
    file.write(world_file_name)
    file.close()
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world, 'pause' : pause}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ]

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file_name',
            default_value='apartment.world',
            description='Name of the world file to load'
        ),
        OpaqueFunction(function=launch_setup),
    ])