import os
import math
import xml.etree.ElementTree as ET
from xml.dom import minidom

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


NEUTRAL_JOINT_POSITIONS = {
    "FL_ABAD_JOINT": 0.0,
    "FL_HIP_JOINT": 1.0475260019302368,
    "FL_KNEE_JOINT": -1.9930626153945923,
    "FR_ABAD_JOINT": 0.0,
    "FR_HIP_JOINT": 1.0475260019302368,
    "FR_KNEE_JOINT": -1.9930626153945923,
    "RL_ABAD_JOINT": 0.0,
    "RL_HIP_JOINT": 1.0475260019302368,
    "RL_KNEE_JOINT": -1.9930626153945923,
    "RR_ABAD_JOINT": 0.0,
    "RR_HIP_JOINT": 1.0475260019302368,
    "RR_KNEE_JOINT": -1.9930626153945923,
}

FOOT_COLLISION_MESHES = {
    "FL_FOOT_LINK",
    "FR_FOOT_LINK",
    "RL_FOOT_LINK",
    "RR_FOOT_LINK",
}

ACTIVE_COLLISION_MESHES = {"BASE_LINK", *FOOT_COLLISION_MESHES}


# 用途：把 MuJoCo keyframe 中的 qpos/qvel/ctrl 数值列表转换成 MJCF 需要的空格分隔字符串。
# 说明：限制有效数字长度，避免生成过长的小数字字符串，同时保持足够的仿真精度。
def format_mjcf_numbers(values):
    return " ".join(f"{value:.12g}" for value in values)



# 用途：准备 /tmp/mujoco 输出目录，并清理上一次生成的 scene.xml。
# 说明：避免旧的 MJCF 场景文件残留，确保每次 launch 都使用本次转换得到的新模型。
def prepare_mujoco_output(model_dir, model_scene_xml):
    os.makedirs(model_dir, exist_ok=True)
    if os.path.lexists(model_scene_xml):
        os.unlink(model_scene_xml)


# 用途：修正转换后的 MuJoCo 碰撞参数，只保留机身和足端参与主要碰撞。
# 说明：降低非关键连杆碰撞带来的抖动和自碰撞风险，并为足端设置更适合落地接触的摩擦与求解参数。
def patch_mujoco_collisions(model_dir):
    model_xml = os.path.join(model_dir, "mujoco_description_formatted.xml")
    tree = ET.parse(model_xml)
    root = tree.getroot()

    for geom in root.findall(".//geom"):
        if geom.get("class") != "collision":
            continue

        mesh_name = geom.get("mesh")
        if mesh_name in ACTIVE_COLLISION_MESHES:
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
        else:
            geom.set("contype", "0")
            geom.set("conaffinity", "0")

        if mesh_name in FOOT_COLLISION_MESHES:
            geom.set("priority", "2")
            geom.set("condim", "6")
            geom.set("friction", "1.2 0.04 0.002")
            geom.set("solref", "0.006 4")
            geom.set("solimp", "0.95 0.99 0.001")

    ET.indent(tree, space="  ")
    tree.write(model_xml, encoding="unicode", xml_declaration=False)


# 用途：根据 spawn_x/spawn_y/spawn_z/spawn_yaw 写入 MuJoCo 初始 keyframe。
# 说明：MuJoCo keyframe 需要完整 qpos/qvel/ctrl；这里用模型默认状态补齐，再覆盖 base 位姿和预设站立关节角。
def write_spawn_keyframe(context, *args, **kwargs):
    keyframe_name = LaunchConfiguration("initial_keyframe").perform(context).strip()
    if not keyframe_name:
        return []

    model_dir = "/tmp/mujoco"
    model_scene_xml = os.path.join(model_dir, "scene.xml")
    spawn_x = float(LaunchConfiguration("spawn_x").perform(context))
    spawn_y = float(LaunchConfiguration("spawn_y").perform(context))
    spawn_z = float(LaunchConfiguration("spawn_z").perform(context))
    spawn_yaw = float(LaunchConfiguration("spawn_yaw").perform(context))

    import mujoco

    patch_mujoco_collisions(model_dir)

    model = mujoco.MjModel.from_xml_path(model_scene_xml)
    free_joint_id = None
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            free_joint_id = joint_id
            break
    if free_joint_id is None:
        raise RuntimeError(f"No free joint found in {model_scene_xml}")

    qpos = [float(value) for value in model.qpos0]
    qvel = [0.0] * model.nv
    ctrl = [0.0] * model.nu
    half_yaw = 0.5 * spawn_yaw
    free_qpos_adr = model.jnt_qposadr[free_joint_id]
    qpos[free_qpos_adr : free_qpos_adr + 7] = [
        spawn_x,
        spawn_y,
        spawn_z,
        math.cos(half_yaw),
        0.0,
        0.0,
        math.sin(half_yaw),
    ]

    for joint_name, joint_position in NEUTRAL_JOINT_POSITIONS.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise RuntimeError(f"Joint '{joint_name}' not found in {model_scene_xml}")
        qpos[int(model.jnt_qposadr[joint_id])] = joint_position

    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id][0])
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if joint_name in NEUTRAL_JOINT_POSITIONS:
            ctrl[actuator_id] = NEUTRAL_JOINT_POSITIONS[joint_name]

    tree = ET.parse(model_scene_xml)
    root = tree.getroot()
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")

    spawn_key = None
    for key in keyframe.findall("key"):
        if key.get("name") == keyframe_name:
            spawn_key = key
            break
    if spawn_key is None:
        spawn_key = ET.SubElement(keyframe, "key", {"name": keyframe_name})

    spawn_key.set("qpos", format_mjcf_numbers(qpos))
    spawn_key.set("qvel", format_mjcf_numbers(qvel))
    spawn_key.set("ctrl", format_mjcf_numbers(ctrl))

    ET.indent(tree, space="  ")
    tree.write(model_scene_xml, encoding="unicode", xml_declaration=False)
    return []


# 用途：创建本 launch 文件的全部 ROS 2 launch action 和 node。
# 说明：负责生成 MuJoCo 模型、启动 robot_state_publisher/MuJoCo/controller spawner/RViz/CHAMP，并定义它们的启动顺序。
def create_nodes(context, *args, **kwargs):
    ld = LaunchDescription()

    model_dir = "/tmp/mujoco"
    model_scene_xml = os.path.join(model_dir, "scene.xml")
    prepare_mujoco_output(model_dir, model_scene_xml)

    edu_description_share = get_package_share_directory("edu_description")
    sim_mujoco_dog_share = get_package_share_directory("sim_mujoco_dog")
    mujoco_config_share = get_package_share_directory("mujoco_config")

    robot_urdf_path = os.path.join(mujoco_config_share, "config", "edu_mujoco.urdf.xacro")
    stand_robot_urdf_path = os.path.join(edu_description_share, "urdf", "edu_mujoco.urdf")
    scene_xml_path = os.path.join(mujoco_config_share, "config", "scene.xml")
    mujoco_inputs_path = os.path.join(mujoco_config_share, "config", "mujoco_inputs.xml")
    ros2_control_params_file = os.path.join(sim_mujoco_dog_share, "config", "d1_mujoco_controllers.yaml")

    # 含 mujoco 配置段的 描述文件, 用于 mujoco_ros2_control_node
    robot_description_xml = xacro.process_file(
        robot_urdf_path,
        mappings={
            "mujoco_model": model_scene_xml,
            "headless": LaunchConfiguration("headless").perform(context),
            "sim_speed_factor": LaunchConfiguration("sim_speed_factor").perform(context),
            "camera_publish_rate": LaunchConfiguration("camera_publish_rate").perform(context),
            "lidar_publish_rate": LaunchConfiguration("lidar_publish_rate").perform(context),
            "initial_keyframe": LaunchConfiguration("initial_keyframe").perform(context),
        },
    ).toprettyxml(indent="  ")
    robot_description = {"robot_description": robot_description_xml}
    # mjcf_source_xml = strip_mujoco_tags(robot_description_xml)

    # 标准 urdf 描述文件（不含 mujoco 配置段），用于 robot_state_publisher 和 ros2_control_node
    stand_robot_description_xml = xacro.process_file(stand_robot_urdf_path).toprettyxml(indent="  ")
    

    xacro2mjcf = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "mujoco_ros2_control",
            "robot_description_to_mjcf.sh",
            "--robot_description",
            stand_robot_description_xml,
            "--mujoco_inputs",
            mujoco_inputs_path,
            "--output",
            model_dir,
            "--save_only",
            "--scene",
            scene_xml_path,
            "--add_free_joint",
        ],
        output="screen",
    )
    ld.add_action(xacro2mjcf)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
        remappings=[("/joint_states", "/get_joint_states")],
    )
    ld.add_action(robot_state_publisher)

    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_control_params_file,
            {
                "use_sim_time": True,
                "headless": ParameterValue(
                    LaunchConfiguration("headless"),
                    value_type=bool,
                ),
                "sim_speed_factor": ParameterValue(
                    LaunchConfiguration("sim_speed_factor"),
                    value_type=float,
                ),
                "camera_publish_rate": ParameterValue(
                    LaunchConfiguration("camera_publish_rate"),
                    value_type=float,
                ),
                "lidar_publish_rate": ParameterValue(
                    LaunchConfiguration("lidar_publish_rate"),
                    value_type=float,
                ),
            },
        ],
        output="screen",
        remappings=[("/joint_states", "/get_joint_states")],
    )

    controller_manager = LaunchConfiguration("controller_manager")
    controller_manager_timeout = LaunchConfiguration("controller_manager_timeout")

    # 启动 imu_broadcaster、joint_state_broadcaster、legs_controller
    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="imu_spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            controller_manager_timeout,
            "--param-file",
            ros2_control_params_file,
        ],
        output="screen",
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="jsb_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            controller_manager_timeout,
        ],
        output="screen",
    )
    legs_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="legs_spawner",
        arguments=[
            "legs_controller",
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            controller_manager_timeout,
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(sim_mujoco_dog_share, "rviz", "d1_nav2.rviz")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    config_pkg_share = get_package_share_directory("edu_config")
    champ_bringup_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "description_path": robot_urdf_path,
            "rviz": "false",
            "gazebo": "true",
            "base_link_frame": "base_link",
            "publish_odom_tf": "false",
            "publish_foot_contacts": "false",
            "use_foot_contacts": "false",
            "use_base_to_footprint_ekf": "false",
            "use_footprint_to_odom_ekf": "false",
            "joint_controller_topic": "legs_controller/joint_trajectory",
            "gait_config_path": os.path.join(config_pkg_share, "config", "gait", "gait.yaml"),
            "joints_map_path": os.path.join(config_pkg_share, "config", "joints", "joints.yaml"),
            "links_map_path": os.path.join(config_pkg_share, "config", "links", "links.yaml"),
            "lite": "true",
            "hardware_connected": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    champ_remap = GroupAction(
        [
            SetRemap(src="/joint_states", dst="/get_joint_states"),
            SetRemap(src="joint_states", dst="get_joint_states"),
            champ_bringup_launch,
        ]
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=xacro2mjcf,
                on_exit=[
                    OpaqueFunction(function=write_spawn_keyframe),
                    rviz_node,
                    TimerAction(
                        period=LaunchConfiguration("mujoco_start_delay"),
                        actions=[
                            mujoco_node,
                            TimerAction(
                                period=LaunchConfiguration("controllers_delay"),
                                actions=[
                                    imu_broadcaster,
                                    jsb_spawner,
                                ],
                            ),
                        ],
                    ),
                ],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[legs_spawner],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=legs_spawner,
                on_exit=[
                    TimerAction(
                        period=LaunchConfiguration("champ_delay"),
                        actions=[champ_remap],
                    )
                ],
            )
        )
    )

    return ld.entities


# 用途：声明可从命令行覆盖的 launch 参数，并把 create_nodes 挂到 OpaqueFunction 中延迟求值。
# 说明：延迟求值可以在 launch context 可用后读取 LaunchConfiguration，并据此生成 robot_description 与节点参数。
def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false", description="Start rviz2"),
            DeclareLaunchArgument("headless", default_value="false", description="Run MuJoCo without GUI"),
            DeclareLaunchArgument("sim_speed_factor", default_value="-1.0"),
            DeclareLaunchArgument("camera_publish_rate", default_value="20.0"),
            DeclareLaunchArgument("lidar_publish_rate", default_value="10.0"),
            DeclareLaunchArgument("initial_keyframe", default_value="spawn"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.4"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="60.0"),
            DeclareLaunchArgument("controllers_delay", default_value="4.0"),
            DeclareLaunchArgument("champ_delay", default_value="1.0"),
            DeclareLaunchArgument("mujoco_start_delay", default_value="1.0"),
            OpaqueFunction(function=create_nodes),
        ]
    )




# 弃用函数

# 用途：从完整 robot_description XML 中移除 <mujoco> 配置段。
# 说明：robot_description_to_mjcf.sh 接收的是标准机器人描述，MuJoCo 专用配置通过单独的 mujoco_inputs.xml 提供。
# def strip_mujoco_tags(xml_text):
#     dom = minidom.parseString(xml_text)
#     for mujoco_node in list(dom.getElementsByTagName("mujoco")):
#         mujoco_node.parentNode.removeChild(mujoco_node)
#     return dom.toxml()