from launch import LaunchDescription
from launch_ros.actions import Node
from os.path import expanduser

param_path = expanduser("~/xxxx.yaml")

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='an_ins_driver',
			namespace='ins',
			executable='an_ins_driver_node',
			name='an_ins_driver_node',
			parameters=[param_path],
		),
		
		Node(
			package='tf2_ros',
			namespace='tfs_static',
			executable='static_transform_publisher',
			name='ins_transform',
			arguments=['0','0','0','0','0','0', 'base_link', 'ins'],
		)

	])
