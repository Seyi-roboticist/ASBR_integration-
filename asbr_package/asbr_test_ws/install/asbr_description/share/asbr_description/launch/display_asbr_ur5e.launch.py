import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Define the URDF file path
    urdf_file = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[-1],
        'share',
        'asbr_description',
        'urdf',
        'asbr.urdf.xacro'
    )

    return LaunchDescription([
        # Declare the launch argument for the URDF file
        DeclareLaunchArgument(
            name='urdf_file',
            default_value=urdf_file,
            description='URDF file'
        ),

        # Run xacro to convert .xacro to .urdf
        ExecuteProcess(
            cmd=['xacro', LaunchConfiguration('urdf_file'), '-o', 'output.urdf'],
            output='screen'
        ),

        # Start robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('urdf_file')])}]
        ),

        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(os.pathsep)[-1],
                'share',
                'asbr_description',
                'launch',
                'ur5e.rviz'
            )]
        ),
    ])
