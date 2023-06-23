# pull in some Python launch modules.

import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

# This function is needed.
def generate_launch_description():
    '''
    Function which generates the launch description for the ariac_playback.launch.py file.
    '''

    # Declare the launch argument to conditionally enable GUI
    no_gui = DeclareLaunchArgument(
        'no_gui',
        default_value='True',
        description='Launch Gazebo without GUI'
    )
    
    team_name = DeclareLaunchArgument(
        'team_name',
        default_value='',
        description='Team name'
    )
    
    trial_name = DeclareLaunchArgument(
        'trial_name',
        default_value='',
        description='Trial name'
    )

    log_file_path = os.getenv('HOME') + '/.ariac/log/gazebo/state.log'
    # Determine the ariac_plugins package share directory
    ariac_plugins_dir = get_package_share_directory('ariac_plugins')
    # print('ariac_plugins_dir', ariac_plugins_dir)

    # Declare the launch argument for extra Gazebo arguments
    extra_gazebo_args = DeclareLaunchArgument(
        'extra_gazebo_args',
        default_value=f'-p {log_file_path} -s {ariac_plugins_dir}/../../lib/libLogPlaybackPlugin.so --pause --verbose',
        description='Extra arguments for Gazebo')

    # Determine the Gazebo package share directory
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')

    # Include the gzserver launch file with the specified extra Gazebo arguments
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gazebo_ros_share_dir, '/launch/gzserver.launch.py'
        ]),
        launch_arguments={'extra_gazebo_args': LaunchConfiguration('extra_gazebo_args')}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gzclient.launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("no_gui")),
    )

    ld = LaunchDescription()
    ld.add_action(extra_gazebo_args)
    ld.add_action(gazebo_server)
    ld.add_action(no_gui)
    ld.add_action(gazebo_client)
    return ld
