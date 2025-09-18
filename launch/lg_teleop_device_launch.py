import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
import os, json, requests
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Locate the config file in the share folder
    config_file = os.path.join(
        get_package_share_directory('lg_teleop_device'),
        'config',
        'config.yaml'
    )

    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)

    nodes = []
    robot_id = config.get('robot_id', 'default_robot')
    
    publish_config = config.get('publish', {})
    media_config = publish_config.get('medias_topics', None)
    if media_config is not None:
    # Add media nodes
        if 'medias_topics' in config['publish']:
            for media in config['publish']['medias_topics']:
                print(f"Adding media node: {media['name']} with host: {config['host']}, port: {config['port']}, endpoint: {media['endpoint'].replace('#####', robot_id)}")
                nodes.append(Node(
                    package='lg_teleop_device',
                    executable='topic_media_node',
                    name=media['name'],
                    parameters=[
                        {'name': media['name']},
                        {'topic': media['topic']},
                        {'width': media['width']},
                        {'height': media['height']},
                        {'host': config['host']},
                        {'port': int(config['port'])},
                        {'endpoint': str(media['endpoint']).replace("#####", robot_id)},
                    ]
                ))

    lidar_config = publish_config.get('lidar', None)
    if lidar_config is not None:
        # Add lidar nodes
        if 'lidar' in config['publish']:
            for lidar in config['publish']['lidar']:
                print(f"Adding lidar node: {lidar['name']} with host: {config['host']}, port: {config['port']}, endpoint: {lidar['endpoint'].replace('#####', robot_id)}")
                nodes.append(Node(
                    package='lg_teleop_device',
                    executable='lidar_node',
                    name=lidar['name'],
                    parameters=[
                        {'name': lidar['name']},
                        {'pointcloud_topic1': lidar['pointcloud_topic1']},
    #                    {'pointcloud_topic2': lidar['pointcloud_topic2']},
    #                    {'pointcloud_topic3': lidar['pointcloud_topic3']},
                        {'host': config['host']},
                        {'port': int(config['port'])},
                        {'endpoint': str(lidar['endpoint']).replace("#####", robot_id)},
                    ]
                ))

    sensor_config = publish_config.get('sensor', None)
    if sensor_config is not None:
        if 'sensor' in config['publish']:
            for joint in config['publish']['sensor']:
                print(f"Adding lidar node: {joint['name']} with host: {config['host']}, port: {config['port']}, endpoint: {joint['endpoint'].replace('#####', robot_id)}")
                nodes.append(Node(
                    package='lg_teleop_device',
                    executable='joint_states_node',
                    name=joint['name'],
                    parameters=[
                        {'name': joint['name']},
                        {'host': config['host']},
                        {'port': int(config['port'])},
                        {'endpoint': str(joint['endpoint']).replace("#####", robot_id)},
                        {'topic': joint['topic']}
                    ]
                ))

    subscribe_config = config.get('subscribe', {})

    control_config = subscribe_config.get('control_topics', None)
    if control_config is not None:
        # Add control node
        if 'subscribe' in config and 'control_topics' in config['subscribe']:
            for control in config['subscribe']['control_topics']:
                print(f"Adding control node: {control['name']} with host: {config['host']}, port: {config['port']}, endpoint: {control['endpoint'].replace('#####', robot_id)}")
                if control['name'] == 'gamepad_control':
                    nodes.append(Node(
                        package='lg_teleop_device',
                        executable='gamepad_control_node',
                        name=control['name'],
                        parameters=[
                            {'name': control['name']},
                            {'host': config['host']},
                            {'port': int(config['port'])},
                            {'endpoint': str(control['endpoint']).replace("#####", robot_id)},
                            {'gamepad_control_topic': control['topic']}
                        ]
                    ))
                elif control['name'] == 'waist_control':
                    nodes.append(Node(
                        package='lg_teleop_device',
                        executable='waist_control_node',
                        name=control['name'],
                        parameters=[
                            {'name': control['name']},
                            {'host': config['host']},
                            {'port': int(config['port'])},
                            {'endpoint': str(control['endpoint']).replace("#####", robot_id)},
                            {'waist_control_topic': control['topic']},
                            {'waist_control_speed_mm_per_sec': control.get('waist_control_speed_mm_per_sec', 20.0)}
                        ]
                    ))
                elif 'arm' in control['name']:
                    nodes.append(Node(
                        package='lg_teleop_device',
                        executable='arm_control_node',
                        name=control['name'],
                        parameters=[
                            {'name': control['name']},
                            {'host': config['host']},
                            {'port': int(config['port'])},
                            {'endpoint': str(control['endpoint']).replace("#####", robot_id)},
                            {'tf_target_frame': control['tf_target_frame']}
                        ]
                    ))
                elif control['name'] == 'head':
                    nodes.append(Node(
                        package='lg_teleop_device',
                        executable='head_control_node',
                        name=control['name'],
                        parameters=[
                            {'name': control['name']},
                            {'host': config['host']},
                            {'port': int(config['port'])},
                            {'endpoint': str(control['endpoint']).replace("#####", robot_id)},
                        ]
                    ))
                elif control['name'] == 'joint_states':
                    nodes.append(Node(
                        package='lg_teleop_device',
                        executable='joint_states_subscription_node',
                        name=control['name'],
                        parameters=[
                            {'name': control['name']},
                            {'host': config['host']},
                            {'port': int(config['port'])},
                            {'endpoint': str(control['endpoint']).replace("#####", robot_id)},
                            {'topic': control['topic']}
                        ]
                    ))

    nodes.append(Node(
        package='lg_teleop_device',
        executable='network_node',
        name='network_node',
        parameters=[
            {'name': 'network_node'},
            {'host': config['host']},
            {'port': int(config['port'])},
            {'endpoint': f'/pang/ws/pub?channel=instant&name={robot_id}&track=status&mode=bundle'},
        ]
    ))
    
    return LaunchDescription(nodes)
