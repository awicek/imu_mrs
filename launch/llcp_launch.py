from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

pkg_name = "mrs_llcp_ros"
this_pkg_path = get_package_share_directory(pkg_name)

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrs_llcp_ros',
            executable='MrsLlcpRos_LlcpNode',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'config_private': this_pkg_path + '/config/private/llcp.yaml'},
            ]
        )
    ])