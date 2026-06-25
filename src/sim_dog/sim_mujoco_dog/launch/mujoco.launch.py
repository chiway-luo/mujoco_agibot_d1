import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


FORWARDED_ARGUMENTS = [
    ("rviz", "true"),
    ("headless", "false"),
    ("sim_speed_factor", "-1.0"),
    ("camera_publish_rate", "20.0"),
    ("lidar_publish_rate", "10.0"),
    ("initial_keyframe", "spawn"),
    ("spawn_x", "0.0"),
    ("spawn_y", "0.0"),
    ("spawn_z", "0.6"),
    ("spawn_yaw", "0.0"),
    ("controller_manager", "/controller_manager"),
    ("controller_manager_timeout", "60.0"),
    ("controllers_delay", "4.0"),
    ("champ_delay", "1.0"),
    ("mujoco_start_delay", "1.0"),
]


def generate_launch_description():
    launch_file = os.path.join(
        get_package_share_directory("sim_ign_dog"),
        "launch",
        "d1_mujoco_sim_dog.launch.py",
    )

    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=default_value)
                for name, default_value in FORWARDED_ARGUMENTS
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    name: LaunchConfiguration(name)
                    for name, _ in FORWARDED_ARGUMENTS
                }.items(),
            ),
        ]
    )
