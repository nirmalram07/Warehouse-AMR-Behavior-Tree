import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    nav2_pkg = get_package_share_directory('nav2_bringup')
    spawn_pkg = get_package_share_directory('turtlebot3_spawn')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={'use_sim_time': use_sim_time,
                          'map' : os.path.join(spawn_pkg, 'map', 'factory_map.yaml'),
                          'params_file' : os.path.join(spawn_pkg, 'config', 'nav2_params.yaml')}.items()
    )

    rviz_launc_conf = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(nav2_pkg,'rviz','nav2_default_view.rviz')
            ]
    )
    
    set_amcl_pose = Node(
        package='turtlebot3_spawn',
        executable='amcl_pose_intializer'
    )

    return LaunchDescription([
        nav2_launch,
        set_amcl_pose,
        rviz_launc_conf
    ])