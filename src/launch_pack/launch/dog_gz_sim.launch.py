from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
import os

def generate_launch_description():

    dog_path = get_package_share_directory('dog')
    launch_path=get_package_share_directory('launch_pack')


    urdf_path = os.path.join(
        dog_path,
        "urdf", "dog.urdf"
    )
    with open(urdf_path, 'r') as inf:
        robot_desc = inf.read()

    controller_yaml=os.path.join(
        launch_path,
        "config", "ros2_controller.yaml"
    )

    controller_config_path = SetEnvironmentVariable(
        name='DOG_CONTROLLER_CONFIG',
        value=controller_yaml
    )
    
    model_path = os.path.join(dog_path, "..")
    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', 
        value=[model_path]
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    # 启动 gz sim
    gz_sim_start = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
    launch_arguments={'gz_args': '-r empty.sdf'}.items(),)

    gz_sim_create=Node(package="ros_gz_sim",executable="create",
    arguments=["-name", "dog",
               "-topic", "robot_description",
               "-x", "0.0",
                "-y", "0.0",
                "-z", "0.5",
                "-R", "0.0",
                "-P", "0.0", 
                "-Y", "0.0"],
    output="screen")

    spawner_activate_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dog_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(launch_path,"config", "bridge_config.yaml")}
            ],
        output='screen'
    )

    leg_calc = Node(
        package="leg_calc",
        executable="leg_calc"
    )

    rviz2_config_path=os.path.join(
        get_package_share_directory("launch_pack"),
        "rviz", "display_config.rviz"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config_path]  # 可选，指定rviz配置文件
    )

    rviz2_delayed = TimerAction(
    period=5.0,  # 延迟 3 秒
    actions=[rviz2])

    return LaunchDescription([
        leg_calc,
        rviz2_delayed,

        controller_config_path,
        set_gz_resource_path,
        robot_state_pub,
        gz_sim_start,
        gz_sim_create,
        spawner_activate_controller,
        bridge_node
    ])
