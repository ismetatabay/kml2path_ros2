from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from glob import glob

import os

def generate_launch_description():
    rviz_cfg_path = os.path.join(get_package_share_directory('kml2path_ros2'),
                                 'config', 'kml2path.rviz')
    kml2path = Node(
        package="kml2path_ros2",
        executable="kml2path",
        name="kml2path_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"kml_filepath": "src/kml2path_ros2/data/test.kml"}
        ]
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg_path],
    )
    return LaunchDescription([
        kml2path,
        rviz2
    ])
