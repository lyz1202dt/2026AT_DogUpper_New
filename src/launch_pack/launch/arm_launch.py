from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory("dog"), 
        "arm", "arm.urdf"  
    )
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]  
    )

   
    arm_pub = Node(
        package="leg_driver",
        executable="arm_drive"
    )
    
    arm_calc = Node(
        package="leg_calc",
        executable="arm_calc"
    )

    # rviz2_config_path = os.path.join(
    #     get_package_share_directory("launch_pack"),  
    #     "rviz", "display_config.rviz"
    # )

    # rviz2 = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz2_config_path]  # 指定rviz配置文件
    # )

    return LaunchDescription([
        robot_state_pub,  
        arm_pub,         
        arm_calc        
    ])
