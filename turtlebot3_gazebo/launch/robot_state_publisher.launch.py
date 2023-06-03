import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import argparse
import xml.etree.ElementTree as ET
import rclpy

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from gazebo_msgs.srv import SpawnEntity


def launch_setup(context, *args, **kwargs):
    ####### DATA INPUT ##########
    # This is to access the argument variables. Otherwise we cant access the values
    box_bot_name = LaunchConfiguration('box_bot_name').perform(context)
    robot_file = LaunchConfiguration('robot_file').perform(context)
    robot_description_topic_name = "/" + box_bot_name + "_robot_description"
    robot_state_publisher_name = box_bot_name +  "_robot_state_publisher"
    joint_state_topic_name = "/" + box_bot_name + "/joint_states"


    ####### DATA INPUT END ##########

    package_description = "box_bot_description"    

    
    extension = robot_file.split(".")[1]

    if extension == "urdf":
        robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "urdf", robot_file)
        robot_desc = xacro.process_file(robot_desc_path)
        # We need to remap the transform (/tf) topic so each robot has its own.
        # We do this by adding `ROS argument entries` to the urdf file for
        # each plugin broadcasting a transform. These argument entries provide the
        # remapping rule, i.e. /tf -> /<robot_id>/tf
        tree = ET.parse(robot_desc_path)
        root = tree.getroot()
        imu_plugin = None
        diff_drive_plugin = None 
        print("DUPA0")
        for plugin in root.iter('plugin'):
            if 'turtlebot3_diff_drive' in plugin.attrib.values():
                diff_drive_plugin = plugin
                print("DUPA1")
            elif 'turtlebot3_imu' in plugin.attrib.values():
                imu_plugin = plugin
                print("DUPA2")

        # We change the namespace to the robots corresponding one
        tag_diff_drive_ros_params = diff_drive_plugin.find('ros')
        tag_diff_drive_ns = ET.SubElement(tag_diff_drive_ros_params, 'namespace')
        tag_diff_drive_ns.text = '/' + args.robot_namespace
        ros_tf_remap = ET.SubElement(tag_diff_drive_ros_params, 'remapping')
        ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'
        robot_desc = xacro.process_file(robot_desc_path, mappings={'box_bot_name' : box_bot_name})
    elif extension == "xacro":
        robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "robot", robot_file)
        # We load the XACRO file with ARGUMENTS


    else:
        assert False, "Extension of robot file not suppored = "+str(extension)
  

 
    xml = robot_desc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=robot_state_publisher_name,
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': xml}],
        remappings=[("/robot_description", robot_description_topic_name),
                    ("/joint_states", joint_state_topic_name)
                    ],
        output="screen"
    )


    return [robot_state_publisher_node]

def generate_launch_description(): 

    box_bot_name_arg = DeclareLaunchArgument('box_bot_name', default_value='box_bot')
    robot_file_arg = DeclareLaunchArgument('robot_file', default_value='box_bot.urdf')
    

    return LaunchDescription([
        box_bot_name_arg,
        robot_file_arg,
        OpaqueFunction(function = launch_setup)
        ])