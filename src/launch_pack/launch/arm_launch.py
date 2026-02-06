from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os
def generate_launch_description():
    
    arm_pub = Node(
        package="arm_pub",
        executable="arm_pub"
    )
    arm_calc = Node(
        package="leg_calc",
        executable="arm_calc"
    )
    return  LaunchDescription([arm_pub, arm_calc])