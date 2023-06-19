# <launch>
	# <node name="inertial_sense_node" pkg="inertial_sense_ros" exec="inertial_sense_node" 
		# output="screen" args="$(find inertial_sense_ros)/launch/example_params.yaml"/>
# </launch>
import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    package = 'inertial_sense_ros'
    yaml_path = get_package_share_path(package)/'launch/params.yaml'
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package = package,
            executable = 'inertial_sense_node',
            arguments = [str(yaml_path)],
            emulate_tty=True
        )
    ])
