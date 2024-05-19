import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'manipulator'
    file_subpath = 'urdf/manipulator.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )


    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'rover_arm'],
                    output='screen')


    rviz2 = Node( name='rviz2',
                       package='rviz2',
                       executable='rviz2', 
                       arguments=['-d', [os.path.join(get_package_share_directory(pkg_name), 'config', 'config.rviz')]]
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )
    
    hand_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller"],
    )

    joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcaster"],
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz2,
        gazebo,
        spawn_entity,
        arm_controller,
        hand_controller,
        joint_broadcaster
    ])
