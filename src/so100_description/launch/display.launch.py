# -*- coding: utf-8 -*-

# -----------------------------------------------------------------------------
# Launch file for visualizing the so100_description robot model in RViz2.
#
# Loads the robot description, starts robot_state_publisher,
# optionally launches joint_state_publisher_gui, and starts RViz2.
# -----------------------------------------------------------------------------

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

# Package name (must match package.xml and CMakeLists.txt)
PACKAGE_NAME = 'so100_description'

def launch_setup(context, *args, **kwargs):
    """Deferred function to configure and return nodes after evaluating launch arguments."""
    pkg_share = get_package_share_directory(PACKAGE_NAME)

    # Retrieve evaluated launch configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_file = LaunchConfiguration('urdf_file')

    # Construct the full path to the URDF/XACRO file
    urdf_model_path = PathJoinSubstitution([pkg_share, 'urdf', urdf_file])

    # Process the URDF/XACRO file
    try:
        robot_description_content = Command(['xacro ', urdf_model_path])
    except Exception as e:
        print(f"\nError processing XACRO file '{context.perform_substitution(urdf_model_path)}'. "
              f"Ensure 'python3-xacro' is installed and the file is valid. Error: {e}\n")
        robot_description_content = "" # Provide empty string on failure

    # --- Define Nodes ---

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_model_path]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_jsp_gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]


def generate_launch_description():
    """Declares launch arguments and sets up the launch description."""
    pkg_share = get_package_share_directory(PACKAGE_NAME)

    # Default paths and filenames (assuming standard structure)
    default_rviz_config_path = os.path.join(pkg_share, 'config', 'urdf.rviz')
    default_urdf_filename = 'SO_5DOF_ARM100_8j_URDF.SLDASM.urdf'

    # --- Declare Launch Arguments ---

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    use_jsp_gui_arg = DeclareLaunchArgument(
        'use_jsp_gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui')

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file')

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_filename,
        description='Name of the URDF/XACRO file in the urdf folder'
    )

    # --- Create Launch Description ---
    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(use_jsp_gui_arg)
    ld.add_action(rviz_config_file_arg)
    ld.add_action(urdf_file_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld