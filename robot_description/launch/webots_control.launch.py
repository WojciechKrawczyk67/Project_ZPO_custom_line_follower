import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_proto_from_urdf(urdf_str, proto_output_path):
    """Generate Webots PROTO on the fly from URDF.
    
    Removes virtual root links (base_footprint) so base_link becomes the root Solid
    with inertia, bounding box and physics, allowing the robot to be dynamic.
    """
    root = ET.fromstring(urdf_str)

    for link in list(root.findall('link')):
        if link.get('name') == 'base_footprint':
            root.remove(link)
    for joint in list(root.findall('joint')):
        if joint.get('name') == 'base_joint':
            root.remove(joint)
        elif (joint.find('child') is not None and joint.find('child').get('link') == 'base_link' and
              joint.find('parent') is not None and joint.find('parent').get('link') == 'base_footprint'):
            root.remove(joint)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(ET.tostring(root, encoding='unicode'))
        tmp_clean_urdf = f.name

    try:
        subprocess.run([
            sys.executable, '-m', 'urdf2webots.importer',
            '--input', tmp_clean_urdf,
            '--output', proto_output_path
        ], capture_output=True, text=True, check=True)
        print(f"[webots_control] Proto generated: {proto_output_path}")
    except Exception as e:
        print(f"[webots_control] Warning: urdf2webots generation failed: {e}")
    finally:
        if os.path.exists(tmp_clean_urdf):
            os.remove(tmp_clean_urdf)


def generate_launch_description():
    # Ensure uniform temporary directory across Webots and ROS nodes
    os.environ['WEBOTS_TMPDIR'] = '/tmp'
    os.environ['TMPDIR'] = '/tmp'

    pkg_description = get_package_share_directory('robot_description')

    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    controller_config_file = os.path.join(pkg_description, 'config', 'controllers.yaml')
    world_file = os.path.join(pkg_description, 'worlds', 'robot_world.wbt')
    proto_file = os.path.join(pkg_description, 'worlds', 'Robot_line_follower.proto')

    # Avoid ROS 2 Jazzy controller_manager bug where any argument containing
    # the substring 'robot_description' is stripped, which breaks '--params-file'.
    tmp_controller_config = '/tmp/controllers.yaml'
    shutil.copyfile(controller_config_file, tmp_controller_config)

    doc = xacro.process_file(xacro_file, mappings={'use_webots_hardware': 'true'})
    urdf_str = doc.toxml()

    # Automatically generate/update Webots PROTO on the fly from current XACRO
    generate_proto_from_urdf(urdf_str, proto_file)

    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf') as tmp_urdf:
        tmp_urdf.write(urdf_str)
        tmp_urdf_path = tmp_urdf.name

    webots = WebotsLauncher(
        world=world_file,
        ros2_supervisor=False,
        mode='realtime',
        gui = True,
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_str,
            'use_sim_time': False
        }]
    )

    webots_driver = WebotsController(
        robot_name='robot',
        parameters=[
            {
                'robot_description': tmp_urdf_path,
                'use_sim_time': False,
                'set_robot_state_publisher': False
            },
            tmp_controller_config,
        ],
        respawn=True
    )

    controller_manager_timeout = ['--controller-manager-timeout', '100']

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--param-file', tmp_controller_config] + controller_manager_timeout,
    )

    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['ackermann_controller', '--param-file', tmp_controller_config] + controller_manager_timeout,
    )

    waiting_nodes = WaitForControllerConnection(
        target_driver=webots_driver,
        nodes_to_start=[joint_state_broadcaster_spawner]
    )

    delay_ackermann = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner]
        )
    )

    shutdown_on_webots_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=webots,
            on_exit=[
                launch.actions.EmitEvent(event=launch.events.Shutdown())
            ],
        )
    )

    return LaunchDescription([
        webots,
        robot_state_pub_node,
        webots_driver,
        waiting_nodes,
        delay_ackermann,
        shutdown_on_webots_exit,
    ])
