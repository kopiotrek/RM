# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# Author: Darby Lim

#INSTRUCTIONS:
# 1. colcon build
# 2. source install/setup.bash
# 3. ros2 launch slam_toolbox online_async_launch.py 
# 4. ros2 launch turtlebot3_navigation2 navigation2.launch.py 
# 5. ros2 launch turtlebot3_explorer turtlebot3_explorer.launch.py 

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rst_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # world_file_name = 'turtlebot3_worlds/' + TURTLEBOT3_MODEL + '.model'
    # world_file_name = 'turtlebot3_houses/waffle.model'
    world_file_name = 'turtlebot3_dqn_stage2/waffle.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'turtlebot3_dqn_stage2.yaml'))
    # map_dir = LaunchConfiguration(
        # 'map',
        # default=os.path.join(
        #     get_package_share_directory('turtlebot3_navigation2'),
        #     'map',
        #     'turtlebot3_world.yaml'))

    # param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_file_name = 'waffle_2.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'nav2_param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    slam_launch_file_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

            

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rst_launch_file_dir, '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
        

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([slam_launch_file_dir, '/online_async_launch.py']),
        #     launch_arguments={}.items(),
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])