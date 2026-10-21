"""Start Gazebo Harmonic and a standing, wheel-driven Go2-W, with optional RViz."""

import os
import shlex
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def value(context, name):
    """
    @brief Resolve a launch argument in the current context.

    @param context: Active launch context.
    @param name: Name of the launch argument.
    @return: Resolved argument as a string.
    """
    return LaunchConfiguration(name).perform(context)


def enabled(context, name):
    """
    @brief Resolve and validate a boolean launch argument.

    @param context: Active launch context.
    @param name: Name of the boolean launch argument.
    @return: True when the argument is true, otherwise False.
    @throws ValueError: If the argument is neither true nor false, ignoring case.
    """
    setting = value(context, name).lower()

    if setting not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')

    return setting == 'true'


def resource_environment():
    """
    @brief Prepare Gazebo mesh, world, and control-plugin search paths.

    @return: Copy of the process environment with installed resource paths prepended
            and existing search paths preserved.
    """
    env = os.environ.copy()
    description = Path(get_package_share_directory('go2w_description'))
    simulation = Path(get_package_share_directory('go2w_gazebo'))

    # Mesh URIs include the package name, so Gazebo searches its parent share directory.
    additions = {
        'GZ_SIM_RESOURCE_PATH': [str(description.parent), str(simulation / 'worlds')],
        'GZ_SIM_SYSTEM_PLUGIN_PATH': [str(Path(get_package_prefix('gz_ros2_control')) / 'lib')],
    }

    for name, paths in additions.items():
        # Keep resources supplied by the caller available after the package paths.
        if env.get(name):
            paths.append(env[name])

        env[name] = os.pathsep.join(paths)

    return env


def world_name(path):
    """
    @brief Read the world name from a single-world SDF file.

    @param path: Path to the world SDF file.
    @return: Name of the world declared in the file.
    @throws ValueError: If the file does not contain exactly one named world.
    """
    root = ET.parse(path).getroot()
    worlds = root.findall('world')

    if len(worlds) != 1 or not worlds[0].get('name'):
        raise ValueError('The world file must contain exactly one named <world>')

    return worlds[0].get('name')


def launch_simulation(context):
    """
    @brief Configure the environment and compose Gazebo and robot spawning.

    @param context: Launch context containing the world, GUI, and robot arguments.
    @return: Ordered environment actions followed by Gazebo and spawn launch includes.
    @throws ValueError: If the world file is missing or launch arguments are invalid.
    """
    simulation = Path(get_package_share_directory('go2w_gazebo'))
    world = Path(value(context, 'world')).expanduser().resolve()

    if not world.is_file():
        raise ValueError(f'World file does not exist: {world}')

    # Gazebo services use the SDF world name, which can differ from the file name.
    name = world_name(world)
    arguments = ['-r', '-v', '3']

    if not enabled(context, 'gui'):
        # Sensor rendering still needs a rendering backend when the GUI is disabled.
        arguments.extend(['-s', '--headless-rendering'])

    arguments.append(str(world))

    # Delegate Gazebo startup to ros_gz_sim and quote arguments for paths with spaces.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
        launch_arguments={'gz_args': shlex.join(arguments),
                          'on_exit_shutdown': 'true'}.items())

    # Forward robot options to the same spawn sequence used for an external world.
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(simulation / 'launch/spawn.launch.py')),
        launch_arguments={
            'world_name': name,
            'bridge_clock': 'true',
            'rviz': value(context, 'rviz'),
            **{key: value(context, key)
               for key in ('robot_name', 'x', 'y', 'z', 'R', 'P', 'Y', 'startup_timeout',
                           'enable_sensors', 'sensors_config', 'enable_lowstate',
                           'lowstate_config')}
        }.items())

    # Environment actions apply to both the Gazebo server and the spawning process.
    environment = resource_environment()

    return [
        *[SetEnvironmentVariable(name, environment[name])
          for name in ('GZ_SIM_RESOURCE_PATH', 'GZ_SIM_SYSTEM_PLUGIN_PATH')],
        gazebo,
        spawn]


def generate_launch_description():
    """
    @brief Declare simulation arguments and register deferred startup.

    @return: Launch description for Gazebo, the robot, and optional RViz.
    """
    simulation = Path(get_package_share_directory('go2w_gazebo'))

    defaults = {
        'world': (str(simulation / 'worlds/empty.sdf'), 'Absolute path to a single-world SDF.'),
        'robot_name': ('go2w', 'Gazebo entity name; one robot per ROS domain.'),

        'x': ('0.0', 'Initial x position [m].'),
        'y': ('0.0', 'Initial y position [m].'),
        'z': ('0.43', 'Initial base height [m].'),
        'R': ('0.0', 'Initial roll [rad].'),
        'P': ('0.0', 'Initial pitch [rad].'),
        'Y': ('0.0', 'Initial yaw [rad].'),

        'gui': ('true', 'Open the Gazebo client.'),
        'rviz': ('true', 'Open RViz after controller activation.'),

        'enable_sensors': ('true', 'Simulate the configured IMU, LiDAR and RGB camera sensors.'),
        'sensors_config': (str(simulation / 'config/sensors.yaml'), 'Sensor configuration YAML.'),

        'enable_lowstate': ('true', 'Publish native Unitree low-level telemetry.'),
        'lowstate_config': (str(simulation / 'config/lowstate.yaml'),
                            'LowState ROS parameters YAML.'),

        'startup_timeout': ('60.0', 'Wall-clock timeout for world and controller readiness [s].'),
    }

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default,
                                      description=description)
          for name, (default, description) in defaults.items()],
        # Resolve paths and options after launch argument overrides are available.
        OpaqueFunction(function=launch_simulation)])
