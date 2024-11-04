import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop',
            prefix='xterm -e',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2','service','call','/spawn','turtlesim/srv/Spawn', '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}'],
            output='screen'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='turtle1_tf2_broadcaster',
            parameters=[{'turtlename': 'turtle1'}]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle2_tf2_broadcaster',
            name='turtle2_tf2_broadcaster',
            parameters=[{'turtlename': 'turtle2'}]
        ),
        Node(
            package='learning_tf2_py',
            executable='carrot_tf2_broadcaster',
            name='carrot_tf2_broadcaster',
            parameters=[{'radius': 2.0, 'direction_of_rotation': 1}]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='turtle2_tf2_listener'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('learning_tf2_py'), 'config', 'carrot.rviz')]
        )
    ])