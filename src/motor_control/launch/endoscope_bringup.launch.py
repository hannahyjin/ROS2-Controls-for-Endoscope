from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='motor_control',
			executable='stepper_joy_node',
			output='screen'
		),
		Node(
			package='pump_control',
			executable='pump_control_node',
			output='screen'
		),
		Node(
			package='joy_linux',
			executable='joy_linux_node',
			output='screen'
		)
	])
