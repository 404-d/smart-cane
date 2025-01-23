from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore

def generate_launch_description():
    node = Node(
        package='pkg_helloworld_py',
        executable='helloworld',
    )
    return LaunchDescription([node])