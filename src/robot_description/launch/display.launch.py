from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    

    pkg_share = get_package_share_directory('robot_description')
    
    default_urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    default_rviz_path = PathJoinSubstitution([pkg_share, 'rviz', 'robot.rviz'])


    robot_description_content = ParameterValue(
        Command(['xacro ', default_urdf_path]), 
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])