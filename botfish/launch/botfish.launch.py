import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ROS packages
    pkg_botfish = get_package_share_directory('botfish')

    # Manipulation params
    cell_offset = LaunchConfiguration('cell_offset', default='0.05')
    end_effector = LaunchConfiguration('end_effector', default='left_hand_d')
    reference_link = LaunchConfiguration('reference_link', default='left_arm_podest_link')
    grab_height = LaunchConfiguration('grab_height', default='-0.152')
    move_height = LaunchConfiguration('move_height', default='-0.152')
    goal_tolerance = LaunchConfiguration('goal_tolerance', default='0.00625')
    max_velocity = LaunchConfiguration('wheelbase', default='0.2')
    max_acceleration = LaunchConfiguration('max_acceleration', default='0.2')
    planning_time = LaunchConfiguration('planning_time', default='10.0')

    manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_botfish, 'launch'),
            '/include/manipulation/manipulation.launch.py'
        ]),
        launch_arguments={
            'cell_offset': cell_offset,
            'end_effector': end_effector,
            'reference_link': reference_link,
            'grab_height': grab_height,
            'move_height': move_height,
            'goal_tolerance': goal_tolerance,
            'max_velocity': max_velocity,
            'max_acceleration': max_acceleration,
            'planning_time': planning_time
        }.items(),
    )

    """debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_botfish, 'launch'),
            'include/ui/ui.launch.py'
        ]),
    )

    engine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_botfish, 'launch'),
            'include/engine/engine.launch.py'
        ]),
    )"""

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('cell_offset',
                              default_value='0.05',
                              description='offset in x, y to the middle of a cell from the bottom right corner'),
        DeclareLaunchArgument('end_effector',
                              default_value='left_hand_d',
                              description='link that will end up at the specified location'),
        DeclareLaunchArgument('reference_link',
                              default_value='left_arm_podest_link',
                              description='Point of reference for planning/executing'),
        DeclareLaunchArgument('grab_height',
                              default_value='-0.152',
                              description='Height to be at vertically to be able to interact with pieces'),
        DeclareLaunchArgument('move_height',
                              default_value='-0.152',
                              description='Height to be at to move around the board'),
        DeclareLaunchArgument('goal_tolerance',
                              default_value='0.00625',
                              description='Tolerance in meters that the end_effector can be in to consider movement successful'),
        DeclareLaunchArgument('max_velocity',
                              default_value='0.2',
                              description='Percentage of max velocity to run the arms at'),
        DeclareLaunchArgument('max_acceleration',
                              default_value='0.2',
                              description='Percentage of max acceleration to run the arms at'),
        DeclareLaunchArgument('planning_time',
                              default_value='10.0',
                              description='Amount of time in seconds moveit is allowed to plan for'),

        # Nodes
        manipulation
        #debug,
        #engine
    ])