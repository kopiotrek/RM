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

    rsp_pkg = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_file_name = 'turtlebot3_worlds/' + TURTLEBOT3_MODEL + '.model'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)
    namespace1 = 'trb_1'
    namespace2 = 'trb_2'

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'turtlebot3_world.yaml'))

    param_file_name_1 = 'waffle_1.yaml'
    param_file_name_2 = 'waffle_2.yaml'
    param_dir_1 = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name_1))
    param_dir_2 = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name_2))

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
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            name='verbose',
            default_value='false',
            description='Set to "true" to run verbose logging.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world,
                'verbose' : LaunchConfiguration('verbose')
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rsp_pkg, '/launch/robot_state_publisher.launch.py']),
            launch_arguments={'namespace': namespace1,
                              'use_sim_time': use_sim_time}.items(),
        ),
        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rsp_pkg, '/launch/robot_state_publisher.launch.py']),
            launch_arguments={'namespace': namespace2,
                              'use_sim_time': use_sim_time}.items(),
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'namespace': namespace1,
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_dir_1}.items(),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        #     launch_arguments={
        #         'namespace': namespace2,
        #         'map': map_dir,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_dir_2}.items(),
        # ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
