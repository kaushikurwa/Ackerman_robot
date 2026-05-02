import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    bringup_pkg   = get_package_share_directory('my_robot_bringup')
    desc_pkg      = get_package_share_directory('my_robot_description')

    urdf_path     = os.path.join(desc_pkg,   'urdf',   'my_robot.urdf.xacro')
    world_path    = os.path.join(bringup_pkg, 'worlds', 'shapes.sdf')
    bridge_config = os.path.join(bringup_pkg, 'config', 'bridge.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch', 'ign_gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'ign_args': f'-r {world_path}'
        }.items()
    )

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-name',  'ackerman_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1'
        ]
    )

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}]
    )

    image_bridge = Node(
        package='ros_ign_image',
        executable='image_bridge',
        output='screen',
        arguments=['/ackerman/camera/image_raw']
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        image_bridge,
    ])