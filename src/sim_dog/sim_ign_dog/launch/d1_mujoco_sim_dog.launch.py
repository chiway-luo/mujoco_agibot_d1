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


STAND_JOINT_POSITIONS = [
    0.0,
    1.0475260019302368,
    -1.9930626153945923,
    0.0,
    1.0475260019302368,
    -1.9930626153945923,
    0.0,
    1.0475260019302368,
    -1.9930626153945923,
    0.0,
    1.0475260019302368,
    -1.9930626153945923,
]


def format_mjcf_numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def strip_mujoco_tags(xml_text):
    dom = minidom.parseString(xml_text)
    for mujoco_node in list(dom.getElementsByTagName("mujoco")):
        mujoco_node.parentNode.removeChild(mujoco_node)
    return dom.toxml()


def prepare_mujoco_output(model_dir, model_scene_xml):
    os.makedirs(model_dir, exist_ok=True)
    if os.path.lexists(model_scene_xml):
        os.unlink(model_scene_xml)


def write_spawn_mujoco_inputs(source_path, output_path, context):
    spawn_x = float(LaunchConfiguration("spawn_x").perform(context))
    spawn_y = float(LaunchConfiguration("spawn_y").perform(context))
    spawn_z = float(LaunchConfiguration("spawn_z").perform(context))
    spawn_yaw = float(LaunchConfiguration("spawn_yaw").perform(context))

    half_yaw = 0.5 * spawn_yaw
    qpos = [
        spawn_x,
        spawn_y,
        spawn_z,
        math.cos(half_yaw),
        0.0,
        0.0,
        math.sin(half_yaw),
        *STAND_JOINT_POSITIONS,
    ]
    qvel = [0.0] * 18

    tree = ET.parse(source_path)
    root = tree.getroot()
    raw_inputs = root.find("raw_inputs")
    if raw_inputs is None:
        raise RuntimeError(f"Missing <raw_inputs> in {source_path}")

    keyframe = raw_inputs.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(raw_inputs, "keyframe")

    spawn_key = None
    for key in keyframe.findall("key"):
        if key.get("name") == "spawn":
            spawn_key = key
            break
    if spawn_key is None:
        spawn_key = ET.SubElement(keyframe, "key", {"name": "spawn"})

    spawn_key.set("qpos", format_mjcf_numbers(qpos))
    spawn_key.set("qvel", format_mjcf_numbers(qvel))
    spawn_key.set("ctrl", format_mjcf_numbers(STAND_JOINT_POSITIONS))

    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="unicode", xml_declaration=False)
    return output_path


def create_nodes(context, *args, **kwargs):
    ld = LaunchDescription()

    model_dir = "/tmp/mujoco"
    model_scene_xml = os.path.join(model_dir, "scene.xml")
    prepare_mujoco_output(model_dir, model_scene_xml)

    edu_description_share = get_package_share_directory("edu_description")
    sim_ign_dog_share = get_package_share_directory("sim_ign_dog")

    robot_urdf_path = os.path.join(edu_description_share, "urdf", "edu_mujoco.urdf.xacro")
    scene_xml_path = os.path.join(edu_description_share, "urdf", "scene.xml")
    mujoco_inputs_template_path = os.path.join(edu_description_share, "urdf", "mujoco_inputs.xml")
    mujoco_inputs_path = write_spawn_mujoco_inputs(
        mujoco_inputs_template_path,
        os.path.join(model_dir, "mujoco_inputs.xml"),
        context,
    )
    ros2_control_params_file = os.path.join(
        sim_ign_dog_share, "config", "d1_mujoco_controllers.yaml"
    )

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
    mjcf_source_xml = strip_mujoco_tags(robot_description_xml)

    xacro2mjcf = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "mujoco_ros2_control",
            "robot_description_to_mjcf.sh",
            "--robot_description",
            mjcf_source_xml,
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
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(sim_ign_dog_share, "rviz", "d1_nav2.rviz")],
        output="screen",
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


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Start rviz2"),
            DeclareLaunchArgument("headless", default_value="false", description="Run MuJoCo without GUI"),
            DeclareLaunchArgument("sim_speed_factor", default_value="-1.0"),
            DeclareLaunchArgument("camera_publish_rate", default_value="20.0"),
            DeclareLaunchArgument("lidar_publish_rate", default_value="10.0"),
            DeclareLaunchArgument("initial_keyframe", default_value="spawn"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.45"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
            DeclareLaunchArgument("controller_manager_timeout", default_value="60.0"),
            DeclareLaunchArgument("controllers_delay", default_value="4.0"),
            DeclareLaunchArgument("champ_delay", default_value="1.0"),
            DeclareLaunchArgument("mujoco_start_delay", default_value="1.0"),
            OpaqueFunction(function=create_nodes),
        ]
    )
