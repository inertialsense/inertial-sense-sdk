from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    package = 'inertial_sense_ros'
    yaml_path = get_package_share_path(package)/'launch/example_params.yaml'
    return LaunchDescription([
        Node(
            package = package,
            executable = 'inertial_sense_node',
            arguments = [str(yaml_path)],
            emulate_tty=True
        )
    ])
