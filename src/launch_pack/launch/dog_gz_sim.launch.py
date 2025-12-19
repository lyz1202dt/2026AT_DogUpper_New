from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("dog"),
        "urdf", "dog.urdf"
    )

    controller_ymal=os.path.join(
        get_package_share_directory("launch_pack"),
        "config", "ros2_controller.yaml"
    )

    # 读取URDF内容
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    gz_sim_create=Node(package="ros_gz_sim",executable="create",
    arguments=["-name", "dog","-topic", "robot_description"],
    output="screen")
    
    # 启动 gz sim
    gezebo_start=ExecuteProcess(cmd=["gz", "sim", "-r", "empty.sdf"],output="screen")

    # 启动 controller manager 并加载配置文件中的 controller（不要把 yaml 当作 spawner 的参数）
    # controller_manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[controller_path],
    #     output="screen",
    # )

    ros2_control_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_ymal,
                    robot_desc,
                    {'hardware': 'gz_ros2_control'}
        ],
        output="screen"
    )

    # 使用 spawner 启动具体的 controller（这里的名字需要与 ros2_controller.yaml 中的键一致）
    spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dog_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # leg_calc = Node(
    #     package="leg_calc",
    #     executable="leg_calc"
    # )

    # rviz2_config_path=os.path.join(
    #     get_package_share_directory("launch_pack"),
    #     "rviz", "display_config.rviz"
    # )

    # rviz2 = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", rviz2_config_path]  # 可选，指定rviz配置文件
    # )
    return LaunchDescription([
        #gezebo_start,
        robot_state_pub,
        #gz_sim_create,
        ros2_control_manager,
        spawner_node,
    ])
