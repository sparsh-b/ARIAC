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
    team_name = LaunchConfiguration('team_name').perform(context)
    trial_name = LaunchConfiguration('trial_name').perform(context)

    if not team_name:
        log_file_path = os.getenv('HOME') + '/.ariac/log/gazebo/state.log'
    else:
        log_file_path = os.getenv(
            'HOME') + '/.ariac/log/gazebo/' + team_name + '/' + trial_name + '/state.log'

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
        'team_name',
        default_value='',
        description='Team name'
    ))

    declared_arguments.append(DeclareLaunchArgument(
        'trial_name',
        default_value='',
        description='Trial name'
    ))

    # Determine the Gazebo package share directory
    # gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')

    # # Include the gzserver launch file with the specified extra Gazebo arguments
    # declared_arguments.append(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         gazebo_ros_share_dir, '/launch/gzserver.launch.py'
    #     ]),
    #     launch_arguments={'extra_gazebo_args': LaunchConfiguration('extra_gazebo_args')}.items()
    # ))

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gzclient.launch.py"]
        ),
        condition=IfCondition(LaunchConfiguration("no_gui")),
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)] + [gzclient])


# This function is needed.
# def generate_launch_description():
#     '''
#     Function which generates the launch description for the ariac_playback.launch.py file.
#     '''

#     # Declare the launch argument to conditionally enable GUI
#     no_gui = DeclareLaunchArgument(
#         'no_gui',
#         default_value='True',
#         description='Launch Gazebo without GUI'
#     )

#     team_name = DeclareLaunchArgument(
#         'team_name',
#         default_value='',
#         description='Team name'
#     )

#     trial_name = DeclareLaunchArgument(
#         'trial_name',
#         default_value='',
#         description='Trial name'
#     )

#     team_name_str = ''
#     print("*"*80)
#     print('team_name', str(LaunchConfiguration('team_name')))
#     print('trial_name', LaunchConfiguration('trial_name'))
#     print("*"*80)
#     trial_name_str = ''

#     if team_name_str == '':
#         log_file_path = os.getenv('HOME') + '/.ariac/log/gazebo/state.log'
#     else:
#         log_file_path = os.getenv('HOME') + '/.ariac/log/gazebo' + team_name_str + '/' + trial_name_str + '/gazebo/state.log'

#     # log_file_path = os.getenv('HOME') + '/.ariac/log/gazebo/state.log'
#     # Determine the ariac_plugins package share directory
#     ariac_plugins_dir = get_package_share_directory('ariac_plugins')
#     # print('ariac_plugins_dir', ariac_plugins_dir)

#     # Declare the launch argument for extra Gazebo arguments
#     extra_gazebo_args = DeclareLaunchArgument(
#         'extra_gazebo_args',
#         default_value=f'-p {log_file_path} -s {ariac_plugins_dir}/../../lib/libLogPlaybackPlugin.so --pause --verbose',
#         description='Extra arguments for Gazebo')

#     # Determine the Gazebo package share directory
#     gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')

#     # Include the gzserver launch file with the specified extra Gazebo arguments
#     gazebo_server = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             gazebo_ros_share_dir, '/launch/gzserver.launch.py'
#         ]),
#         launch_arguments={'extra_gazebo_args': LaunchConfiguration('extra_gazebo_args')}.items()
#     )

#     gazebo_client = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             [FindPackageShare("gazebo_ros"), "/launch", "/gzclient.launch.py"]
#         ),
#         condition=IfCondition(LaunchConfiguration("no_gui")),
#     )

#     ld = LaunchDescription()
#     ld.add_action(team_name)
#     ld.add_action(trial_name)
#     ld.add_action(extra_gazebo_args)
#     ld.add_action(gazebo_server)
#     ld.add_action(no_gui)
#     ld.add_action(gazebo_client)
#     return ld
