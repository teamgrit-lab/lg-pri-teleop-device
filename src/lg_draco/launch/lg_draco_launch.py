import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
import os, json, requests
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lg_draco'),
        'config',
        'config.yaml'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    nodes = []
    robot_id = config.get('robot_id', 'default_robot')

    input_topic = config.get('input_topic', "/camera/camera/depth/color/points")
    nodes.append(Node(
        package='lg_draco',
        executable='draco_sender',
        name="draco_sender",
        parameters=[
            {'input_topic': input_topic},
            {'host': config['host']},
            {'port': config['port']},
            {'endpoint': str(config['endpoint']).replace("#####", robot_id)},
            {'voxel_size': float(config['voxel_size'])},
            {'skip_length': float(config['skip_length'])},
        ]
    ))
    return LaunchDescription(nodes)
