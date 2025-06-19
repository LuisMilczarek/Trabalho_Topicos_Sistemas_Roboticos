import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("stage_ros2"),"launch","stage.launch.py")),
        launch_arguments={
            "world":"new_cave",
            "enforce_prefixes":"false",
            "one_tf_tree":"true"
        }.items()
    )

    bug_node = Node(
        package="stage_top_roboticos",
        executable="bug",
        name="bug"
    )

    return LaunchDescription(
        [
            stage_launch,
            bug_node
        ]
    )