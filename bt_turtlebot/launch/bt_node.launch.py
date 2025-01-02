import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bt_node = get_package_share_directory('bt_turtlebot')

    bt_node = Node(
        package="bt_turtlebot",
        executable="ros_control",
        name="turtle_control",
        parameters=[{
            "location_file": os.path.join(pkg_bt_node, "config", "map_locations.yaml")
        }]
    )
    return LaunchDescription([
        bt_node
    ])