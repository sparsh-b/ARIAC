# pull in some Python launch modules.

import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    trial_name = LaunchConfiguration('trial_name').perform(context)
    
    log_file_path = os.getenv('HOME') + '/.ariac2023/log/gazebo/' +  trial_name + '/' + 'state.log'

    ariac_plugins_dir = get_package_share_directory('ariac_plugins')

    # Declare the launch argument for extra Gazebo arguments
    extra_gazebo_args = DeclareLaunchArgument(
        'extra_gazebo_args',
        default_value=f'-p {log_file_path} -s {ariac_plugins_dir}/../../lib/libLogPlaybackPlugin.so --pause --verbose',
        description='Extra arguments for Gazebo')

    # Include the gzserver launch file with the specified extra Gazebo arguments
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_ros_share_dir, '/launch/gzserver.launch.py'
        ]),
        launch_arguments={'extra_gazebo_args': LaunchConfiguration('extra_gazebo_args')}.items()
    )

    arguments = [extra_gazebo_args, gzserver]
    return arguments


def generate_launch_description():

    declared_arguments = []

    # Declare the launch argument to conditionally enable GUI
    declared_arguments.append(DeclareLaunchArgument(
        'no_gui',
        default_value='True',
        description='Launch Gazebo without GUI'
    ))
    
    declared_arguments.append(DeclareLaunchArgument(
        'log_file',
        default_value='state',
        description='Team name'
    ))

    declared_arguments.append(DeclareLaunchArgument(
        'trial_name',
        default_value='',
        description='Trial name'
    ))

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gzclient.launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("no_gui")),
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)] + [gzclient])
