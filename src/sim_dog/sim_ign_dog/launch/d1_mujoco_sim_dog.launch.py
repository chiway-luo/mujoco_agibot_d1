from launch import LaunchDescription
from launch_ros.actions import Node
import os
# 封装终端指令相关类--------------
from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable   #FindExecutable(name="ros2")
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition #判断是否执行
from launch.conditions import IfCondition #判断是否执行
# from launch.conditions import UnlessCondition #取反
# from launch.substitutions import PythonExpression #运行时计算表达式
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction
# 事件相关----------------------
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, OpaqueFunction
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
# urdf文件处理相关--------------
# from launch_ros.parameter_descriptions import ParameterValue
# from launch.substitutions import Command
from launch.actions import TimerAction
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
import xacro
from launch_ros.actions import SetRemap

def create_nodes(context, *args, **kwargs):
    create_node = LaunchDescription()
    use_rviz = LaunchConfiguration("rviz")

    model_dir = "/tmp/mujoco" # MJCF文件生成目录
    model_xml = os.path.join(model_dir, "main.xml") # MJCF文件路径

    edu_description_share = get_package_share_directory("edu_description")
    sim_ign_dog_share = get_package_share_directory("sim_ign_dog")
    mujoco_ros2_control_share = get_package_share_directory("mujoco_ros2_control")

    robot_urdf_path = os.path.join(edu_description_share, "urdf", "edu_mujoco.urdf.xacro")
    robot_description = {
        "robot_description": xacro.process_file(robot_urdf_path).toprettyxml(indent="  ")
    }

    ros2_control_params_file = os.path.join(sim_ign_dog_share, "config", "d1_mujoco_controllers.yaml")
    sensor_params_file = os.path.join(sim_ign_dog_share, "config", "d1_mujoco_sensors.yaml")

    xacro2mjcf = Node(
        package="mujoco_ros2_control",
        executable="xacro2mjcf.py",
        parameters=[
            {"robot_descriptions": [robot_description["robot_description"]]},
            {"input_files": [os.path.join(edu_description_share, "urdf", "scene.xml")]},
            {"output_file": model_xml},
            {"mujoco_files_path": model_dir},
            {"base_link": "base_link"},
            {"floating": True},
            {"initial_position": "0 0 0.75"},
            {"initial_orientation": "0 0 0"},
        ],
        output="screen",
    )
    create_node.add_action(xacro2mjcf)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
        remappings=[("/joint_states", "/get_joint_states")],
    )
    create_node.add_action(robot_state_publisher)


    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            robot_description,
            ros2_control_params_file,
            sensor_params_file,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": model_xml},
            {"show_gui": True},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
            ("/joint_states", "/get_joint_states"),
            ("/legs_controller/joint_trajectory", "/joint_command"),
        ],
        output="screen",
    )

    # 发布mujoco中的关节信息到 ROS2 的 /joint_states 话题，供 CHAMP 和其他节点使用
    # joint_state_broadcaster = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", ["/", "controller_manager"]],
    #     output="screen",
    #     remappings=[
    #         ("/joint_states", "/get_joint_states")
    #     ]
    # )
    # create_node.add_action(joint_state_broadcaster)

    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", "--controller-manager", ["/", "controller_manager"], "--param-file", ros2_control_params_file],
        output="screen",
    )

    # 发布机器人位姿到 /tf，供 CHAMP 和其他节点使用；如果你的模型没有 IMU，可以把这个改成 base_pose_broadcaster
    base_pose_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["base_pose_broadcaster", "--controller-manager", ["/", "controller_manager"], "--param-file", ros2_control_params_file],
        output="screen",
    )

    # legs_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["legs_controller", "--controller-manager", ["/", "controller_manager"], "--param-file", ros2_control_params_file],
    #     output="screen",
    # )
    # create_node.add_action(legs_controller)

    rviz_config = os.path.join(sim_ign_dog_share, "rviz", "d1_nav2.rviz")
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )


    #附加内容
    # If你的 controller_manager 实际在模型命名空间下（例如 /model/go2/controller_manager），启动时把这个参数改掉
    # ld.add_action(DeclareLaunchArgument('controller_manager', default_value='/controller_manager'))
    create_node.add_action(DeclareLaunchArgument('controller_manager', default_value='/controller_manager'))
    create_node.add_action(DeclareLaunchArgument('controller_manager_timeout', default_value='60.0'))
    create_node.add_action(DeclareLaunchArgument('controllers_delay', default_value='4.0'))
    create_node.add_action(DeclareLaunchArgument('champ_delay', default_value='1.0'))
    create_node.add_action(DeclareLaunchArgument('mujoco_start_delay', default_value='3.0'))


    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout')
    controller_manager = LaunchConfiguration('controller_manager')
    controllers_delay = LaunchConfiguration('controllers_delay')
    champ_delay = LaunchConfiguration('champ_delay')
    mujoco_start_delay = LaunchConfiguration('mujoco_start_delay')

    # 控制器生成器：等待 controller_manager 服务可用，且按顺序启动（先 joint_state_broadcaster 再 legs_controller）
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='jsb_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        output='screen',
        remappings=[
            ("/joint_states", "/get_joint_states"),
        ]
    )

    legs_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='legs_spawner',
        arguments=[
            'legs_controller',
            '--controller-manager', controller_manager,
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        output='screen',
        remappings=[
            ("/joint_states", "/get_joint_states"),
            ("/legs_controller/joint_trajectory","/joint_command")
            # 如果 legs_controller 的 action 接口是 /legs_controller/joint_trajectory，
            # 且 CHAMP 发送到 /joint_command，则需要这个 remapping；

        ]
    )


    # create_node.add_action(TimerAction(period=controllers_delay, actions=[jsb_spawner]))

    #启动cham
    config_pkg_share = os.path.join(get_package_share_directory('edu_config'))
    descr_pkg_share = os.path.join(get_package_share_directory('edu_description'))

    champ_bringup_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('champ_bringup'),
                'launch',
                'bringup.launch.py'
            )
        ),
        launch_arguments={
            # "description_path": default_model_path,
            # "joints_map_path": joints_config,
            # "links_map_path": links_config,
            # "gait_config_path": gait_config,
            # "use_sim_time": LaunchConfiguration("use_sim_time"),
            # "robot_name": LaunchConfiguration("robot_name"),
            # "gazebo": "true",
            # "lite": LaunchConfiguration("lite"),
            # "rviz": LaunchConfiguration("rviz"),
            # "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            # "hardware_connected": "false",
            # "publish_foot_contacts": "false",
            # "close_loop_odom": "true",
            'use_sim_time': 'true',
            'description_path': os.path.join(descr_pkg_share,'urdf','edu.urdf'),
            'rviz': 'false',#仿真环境已经启动rviz了
            'gazebo': 'true',#在gazebo中运行
            'base_link_frame': 'base_link',#d1_dog模型为base_link
            'publish_odom_tf': 'false',#不发布odom到base的tf,由ekf负责
            'publish_foot_contacts': 'false',#仿真未提供 foot_contacts
            'use_foot_contacts': 'false',#仿真未提供 foot_contacts 时禁用
            'use_base_to_footprint_ekf': 'false',#禁用 base_to_footprint EKF
            'use_footprint_to_odom_ekf': 'true',#启用 footprint_to_odom EKF
            # 'joint_controller_topic': 'legs_controller/joint_trajectory',#关节控制话题 默认joint_group_effort_controller/joint_trajectory
            'joint_controller_topic': 'joint_command',#关节控制话题
            'gait_config_path': os.path.join(config_pkg_share,'config','gait','gait.yaml'),
            'joints_map_path': os.path.join(config_pkg_share,'config','joints','joints.yaml'),
            'links_map_path': os.path.join(config_pkg_share,'config','links','links.yaml'),

            "lite": 'true', #使用精简模式
            "hardware_connected": 'false', #不连接真实硬件
            "close_loop_odom": 'true', #使用闭环里程计
        }.items(),
    )

    champ_remap = GroupAction([
            SetRemap(
                src='/joint_states',
                dst='/get_joint_states'
            ),
            # 如果 CHAMP 内部用的是相对话题 joint_states，也可以加这一条
            SetRemap(
                src='joint_states',
                dst='get_joint_states'
            ),
            champ_bringup_launch
        ]
    )


    #### 启动顺序控制
    # 先启动 CHAMP 和 controller spawner。spawner 会等待 mujoco_node 内部的 controller_manager；
    # mujoco_node 延迟到最后启动，减少仿真先跑、控制命令后到导致的倒地问题。

    start_mujoco = RegisterEventHandler(
        OnProcessExit(
            target_action=xacro2mjcf,
            on_exit=[
                imu_broadcaster,
                base_pose_broadcaster,
                rviz_node,
                TimerAction(
                    period=champ_delay,
                    actions=[
                        champ_remap,
                        TimerAction(period=mujoco_start_delay, actions=[mujoco_node]),
                    ],
                ),
            ],
        )
    )
    create_node.add_action(start_mujoco)

    create_node.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=base_pose_broadcaster,
                on_exit=[jsb_spawner],
            )
        )
    )


    create_node.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[legs_spawner],
            )
        )
    )


    return create_node.entities


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Start rviz2"),
            OpaqueFunction(function=create_nodes),
        ]
    )
