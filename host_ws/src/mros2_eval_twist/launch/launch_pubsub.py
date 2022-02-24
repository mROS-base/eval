from launch import LaunchDescription
from launch_ros.actions import Node

# Note: `node_`, `prefix` and `output` will be removed on Foxy
def generate_launch_description():
  return LaunchDescription([
    Node(
      package='mros2_echoback_twist_calc_time',
      node_executable='pub_node',
      node_name='pub_mros2',
			prefix=['stdbuf -o L'],
			output="screen"
    ),
    Node(
      package='mros2_echoback_twist_calc_time',
      node_executable='sub_node',
      node_name='mros2_sub',
			prefix=['stdbuf -o L'],
			output="screen"
    )
  ])