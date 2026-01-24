
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
    gazebo_model_path = "/usr/share/gazebo-11/models:/usr/share/gazebo/models"
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=f"{model_path}:{gazebo_model_path}"
    )
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f"{model_path}:{gazebo_model_path}"
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}]
    )

    # 启动 gz sim

    world_pkg_path = get_package_share_directory('dog')
    world_path = os.path.join(world_pkg_path, 'world', 'world.sdf')


    gz_sim_start = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
    launch_arguments={'gz_args': f'{world_path}'}.items(),)

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
        controller_config_path,
        set_gz_resource_path,
        set_gazebo_model_path,
        robot_state_pub,
        spawner_activate_controller,
        gz_sim_start,
        gz_sim_create,
        bridge_node,
        leg_calc,
        rviz2_delayed
    ])

"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    # =====================================================
    # 1. 获取包路径
    # =====================================================
    dog_path = get_package_share_directory('dog')
    launch_path = get_package_share_directory('launch_pack')

    # =====================================================
    # 2. 读取 URDF,供 robot_state_publisher 和 Gazebo 使用
    # =====================================================
    urdf_path = os.path.join(dog_path, "urdf", "dog.urdf")
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # =====================================================
    # 3. ros2_control 控制器配置文件路径
    #    通过环境变量传给自定义 controller 插件
    # =====================================================
    controller_yaml = os.path.join(
        launch_path, "config", "ros2_controller.yaml"
    )

    controller_config_path = SetEnvironmentVariable(
        name='DOG_CONTROLLER_CONFIG',
        value=controller_yaml
    )

    # =====================================================
    # 4. Gazebo 模型 / 资源路径
    # =====================================================
    model_path = os.path.join(dog_path, "..")
    gazebo_model_path = "/usr/share/gazebo-11/models:/usr/share/gazebo/models"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{model_path}:{gazebo_model_path}"
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=f"{model_path}:{gazebo_model_path}"
    )

    # =====================================================
    # 5. robot_state_publisher
    #    - 发布 /robot_description
    #    - 发布 TF
    #    Gazebo / RViz / bridge 都依赖它
    # =====================================================
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen"
    )

    # =====================================================
    # 6. 启动 Gazebo 仿真环境
    # =====================================================
    world_path = os.path.join(dog_path, 'world', 'world.sdf')

    gz_sim_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': world_path}.items(),
    )

    # =====================================================
    # 7. 在 Gazebo 中创建机器人实体
    #    注意：必须等 Gazebo 启动完成
    # =====================================================
    gz_sim_create = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "dog",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
        ],
        output="screen"
    )

    # =====================================================
    # 8. ros2_control 控制器加载
    #    依赖：
    #    - Gazebo 中 ros2_control plugin 已加载
    #    - /controller_manager 可用
    # =====================================================
    spawner_activate_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dog_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    # =====================================================
    # 9. ROS ↔ Gazebo 桥接节点
    #    用 yaml 统一管理 topic 映射
    # =====================================================
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                launch_path, "config", "bridge_config.yaml"
            )
        }],
        output='screen'
    )

    # =====================================================
    # 10. 业务节点（腿部运动学 / 控制计算）
    #     依赖：
    #     - bridge
    #     - joint_state
    #     - controller 已激活
    # =====================================================
    leg_calc = Node(
        package="leg_calc",
        executable="leg_calc",
        output="screen"
    )

    # =====================================================
    # 11. RViz
    # =====================================================
    rviz2_config_path = os.path.join(
        launch_path, "rviz", "display_config.rviz"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz2_config_path],
        output="screen"
    )

    # =====================================================
    # 12. 启动顺序控制（核心）
    # =====================================================
    create_delayed = TimerAction(
        period=3.0,        # 等 Gazebo 启动完成
        actions=[gz_sim_create]
    )

    bridge_delayed = TimerAction(
        period=4.0,        # Gazebo + 机器人已存在
        actions=[bridge_node]
    )

    controller_delayed = TimerAction(
        period=5.0,        # controller_manager 就绪
        actions=[spawner_activate_controller]
    )

    leg_calc_delayed = TimerAction(
        period=6.0,        # 控制链路全部打通
        actions=[leg_calc]
    )

    rviz2_delayed = TimerAction(
        period=7.0,        # 所有 topic 都已经存在
        actions=[rviz2]
    )

    # =====================================================
    # 13. 返回 LaunchDescription
    # =====================================================
    return LaunchDescription([
        # ---- 环境变量（必须最先）----
        controller_config_path,
        set_gz_resource_path,
        set_gazebo_model_path,

        # ---- 核心基础节点 ----
        robot_state_pub,
        gz_sim_start,

        # ---- 分阶段启动 ----
        create_delayed,
        bridge_delayed,
        controller_delayed,
        leg_calc_delayed,
        rviz2_delayed,
    ])
"""











