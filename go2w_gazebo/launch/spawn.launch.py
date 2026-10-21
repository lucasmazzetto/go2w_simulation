"""Prepare the Xacro description and spawn one robot into a running Gazebo world."""

import asyncio
import math
import os
import re
import signal
import tempfile
import xml.etree.ElementTree as ET
from functools import partial
from pathlib import Path

import xacro
import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueCoroutine,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LEGS = ('FL', 'FR', 'RL', 'RR')
LEG_JOINTS = [f'{leg}_{part}_joint' for leg in LEGS for part in ('hip', 'thigh', 'calf')]
ROBOT_DESCRIPTION_TOPIC = '/go2w_gazebo/robot_description'
SENSOR_NAMES = ('body_imu', 'lidar_imu', 'lidar')


def load_settings(path):
    """
    @brief Load and validate numeric simulation settings from ROS parameter YAML.

    @param path: YAML file containing the /**/ros__parameters settings section.
    @return: Dictionary of validated simulation settings.
    @throws ValueError: If a value has an invalid type or lies outside its allowed range.
    """
    settings = yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']

    for name, value in settings.items():
        # Python treats booleans as integers; reject them explicitly for numeric settings.
        is_number = not isinstance(value, bool) and isinstance(value, (float, int))

        if not is_number or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        
        if name.startswith('standing_'):
            # Joint angles may be negative; their URDF limits are checked during preparation.
            continue

        if value <= 0:
            raise ValueError(f'{name} must be positive')
        
    if not isinstance(settings['controller_update_rate'], int):
        raise ValueError('controller_update_rate must be an integer')
    
    if settings['position_proportional_gain'] > 1:
        raise ValueError('position_proportional_gain must not exceed 1')

    return settings


def load_sensors(path):
    """
    @brief Load sensor settings and validate frames, sampling, and camera options.

    @param path: Sensor configuration YAML file.
    @return: Validated sensor dictionary, with a disabled camera section if absent.
    @throws ValueError: If the sensor configuration violates the supported schema or limits.
    """
    settings = yaml.safe_load(Path(path).read_text())

    if not isinstance(settings, dict):
        raise ValueError('Sensor configuration must contain body_imu, lidar_imu and lidar')

    configured_sensors = set(settings)
    required_sensors = set(SENSOR_NAMES)
    missing_sensors = required_sensors - configured_sensors
    unknown_sensors = configured_sensors - required_sensors - {'camera'}

    if missing_sensors or unknown_sensors:
        raise ValueError('Sensor configuration must contain body_imu, lidar_imu and lidar')
    
    for name in SENSOR_NAMES:
        sensor = settings[name]

        if not isinstance(sensor, dict) or not isinstance(sensor.get('enabled'), bool):
            raise ValueError(f'{name}.enabled must be a YAML boolean')
        
        frame = sensor.get('frame')
        valid_frame = isinstance(frame, str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', frame)

        if not valid_frame:
            raise ValueError(f'{name}.frame must be an unscoped TF frame name')
        
        for key, value in sensor.items():
            if key in ('enabled', 'frame'):
                continue
        
            is_number = not isinstance(value, bool) and isinstance(value, (int, float))
        
            if not is_number or not math.isfinite(value):
                raise ValueError(f'{name}.{key} must be finite')

            positive_only = key not in ('temperature', 'vertical_min_angle', 'vertical_max_angle')
        
            if key.endswith('noise_stddev'):
                if value < 0:
                    raise ValueError(f'{name}.{key} must be nonnegative')
        
            elif positive_only and value <= 0:
                raise ValueError(f'{name}.{key} must be positive')
        
        if sensor['update_rate'] > 1000:
            raise ValueError(f'{name}.update_rate exceeds the bundled 1000 Hz physics rate')
    
    if len({settings[name]['frame'] for name in SENSOR_NAMES}) != 3:
        raise ValueError('Sensor frames must be distinct')

    lidar = settings['lidar']

    for key in ('horizontal_samples', 'vertical_samples'):
        if not isinstance(lidar[key], int) or lidar[key] < 2:
            raise ValueError(f'lidar.{key} must be an integer >= 2')
    
    if lidar['vertical_samples'] > 65536:
        raise ValueError('Virtual rows must fit the UINT16 ring field')
    
    if not 0 <= lidar['vertical_min_angle'] < lidar['vertical_max_angle'] <= math.pi / 2:
        raise ValueError('LiDAR vertical angles must lie in its local 0..pi/2 hemisphere')
    
    if lidar['range_min'] >= lidar['range_max']:
        raise ValueError('LiDAR minimum range must be smaller than maximum range')

    temperature = settings['body_imu']['temperature']

    if not isinstance(temperature, int) or not -128 <= temperature <= 127:
        raise ValueError('Body IMU temperature must fit the native int8 field')

    # Older sensor files omit the camera section and retain camera-disabled behavior.
    camera = settings.setdefault('camera', {'enabled': False})
    validate_camera(camera, {settings[name]['frame'] for name in SENSOR_NAMES})

    return settings


def validate_camera(camera, sensor_frames):
    """
    @brief Validate camera settings when the camera is enabled.

    @param camera: Camera configuration dictionary, including its enabled flag.
    @param sensor_frames: Frame names already reserved by the IMU and LiDAR sensors.
    @return: None on success or when the camera is disabled.
    @throws ValueError: If camera fields are invalid or frame names conflict.
    """
    if not isinstance(camera, dict) or not isinstance(camera.get('enabled'), bool):
        raise ValueError('camera.enabled must be a YAML boolean')
    
    if not camera['enabled']:
        return

    # Camera mounting and optical frames must be distinct from the other sensor frames.
    for key in ('frame', 'optical_frame'):
        frame = camera.get(key)
        valid_frame = isinstance(frame, str) and re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', frame)
    
        if not valid_frame:
            raise ValueError(f'camera.{key} must be an unscoped TF frame name')
    
    if len(sensor_frames | {camera['frame'], camera['optical_frame']}) != len(sensor_frames) + 2:
        raise ValueError('Camera and sensor frames must be distinct')
    
    topic_pattern = r'/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*'
    
    for key in ('image_topic', 'info_topic'):
        topic = camera.get(key)
        valid_topic = isinstance(topic, str) and re.fullmatch(topic_pattern, topic)
    
        if not valid_topic:
            raise ValueError(f'camera.{key} must be an absolute ROS topic name')
    
    if camera['image_topic'] == camera['info_topic']:
        raise ValueError('Camera image and info topics must be distinct')

    # Calibration uses a reference image size and is scaled to the requested output size.
    for key in ('width', 'height', 'reference_width', 'reference_height'):
        if type(camera.get(key)) is not int or camera[key] <= 0:
            raise ValueError(f'camera.{key} must be a positive integer')
    
    for key in ('update_rate', 'fx', 'fy', 'cx', 'cy', 'clip_near', 'clip_far'):
        value = camera.get(key)
        is_number = not isinstance(value, bool) and isinstance(value, (int, float))

        if not is_number or not math.isfinite(value):
            raise ValueError(f'camera.{key} must be finite')

        if value <= 0 and key not in ('cx', 'cy'):
            raise ValueError(f'camera.{key} must be positive')

    if camera['update_rate'] > 1000:
        raise ValueError('camera.update_rate exceeds the bundled 1000 Hz physics rate')

    cx_in_bounds = 0 <= camera['cx'] < camera['reference_width']
    principal_point_valid = cx_in_bounds and 0 <= camera['cy'] < camera['reference_height']

    if not principal_point_valid:
        raise ValueError('Camera principal point must lie within the reference image')

    if camera['clip_near'] >= camera['clip_far']:
        raise ValueError('Camera near clip must be smaller than far clip')

    for key in ('xyz', 'rpy'):
        values = camera.get(key)

        valid_shape = isinstance(values, list) and len(values) == 3

        finite_components = valid_shape and all(
            not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
            for v in values)

        if not finite_components:
            raise ValueError(f'camera.{key} must contain three finite numbers')


def prepare_simulation(description, simulation, settings_file, sensors_file,
                       controller_file, robot_name, directory,
                       world='default', bridge_clock=True):
    """
    @brief Adapt the upstream URDF, expand Xacro, and prepare ROS configuration.

    The description submodule stays untouched. Only its four wheel collisions
    become macro slots in a temporary copy; Xacro owns the replacement geometry.
    Launch validates the configuration files that Xacro reads directly.

    @param description: Share directory of go2w_description.
    @param simulation: Share directory of go2w_gazebo.
    @param settings_file: Source simulation settings YAML file.
    @param sensors_file: Source sensor YAML file, or None to disable all sensors.
    @param controller_file: Controller YAML path referenced by the generated model.
    @param robot_name: Name assigned to the simulated robot.
    @param directory: Existing writable directory for the temporary adapted description.
    @param world: Gazebo world name used in bridge topic substitutions.
    @param bridge_clock: Whether to include the Gazebo-to-ROS clock bridge.
    @return: Tuple of URDF XML, controller configuration, bridge entries, and sensor
            adapter parameters. Adapter parameters are None when sensors are disabled.
    @throws ValueError: If configuration, standing pose, sensor mounts, or frames are invalid.
    """
    description, simulation, directory = Path(description), Path(simulation), Path(directory)
    # Validate the same source files that Xacro will read when it generates the model.
    settings_file = Path(settings_file).expanduser().resolve()
    sensors_file = Path(sensors_file).expanduser().resolve() if sensors_file is not None else None
    settings = load_settings(settings_file)
    sensors = load_sensors(sensors_file) if sensors_file is not None else None

    robot = ET.parse(description / 'urdf/go2w_description.urdf').getroot()
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}

    # Reject an initial standing pose that would place any leg outside its physical limits.
    for name in LEG_JOINTS:
        position = settings[f'standing_{name.split("_")[1]}']
        limit = joints[name].find('limit')
  
        if not float(limit.get('lower')) <= position <= float(limit.get('upper')):
            raise ValueError(f'Standing position for {name} is outside its limits')

    # Xacro expands macros but cannot replace elements in an included plain URDF.
    # Put macro calls at those four existing collision locations before inclusion.
    ET.register_namespace('xacro', 'http://www.ros.org/wiki/xacro')
  
    for leg in LEGS:
        wheel = robot.find(f"link[@name='{leg}_foot']")

        for collision in wheel.findall('collision'):
            wheel.remove(collision)

        ET.SubElement(wheel, '{http://www.ros.org/wiki/xacro}wheel_collision',
                      side='1' if leg.endswith('L') else '-1')

    description_file = directory / 'description.urdf.xacro'
    description_file.write_text(ET.tostring(robot, encoding='unicode'))

    if sensors is not None:
        # Sensor macros inherit the existing fixed IMU and LiDAR mounting transforms.
        for frame in ('imu', 'radar'):
            joint = robot.find(f"joint/child[@link='{frame}']/..")
            link_exists = robot.find(f"link[@name='{frame}']") is not None
            fixed_joint = link_exists and joint is not None and joint.get('type') == 'fixed'
            mounted_to_base = fixed_joint and joint.find('parent').get('link') == 'base'
  
            if not mounted_to_base:
                raise ValueError(f'Expected fixed base mounting for {frame}')

        # Reject duplicate frames before Xacro can create an ambiguous TF tree.
        links = {link.get('name') for link in robot.findall('link')}
        joint_names = set(joints)
        new_frames = [sensors[name]['frame']
                      for name, parent in (('body_imu', 'imu'), ('lidar_imu', None),
                                           ('lidar', 'radar'))
                      if sensors[name]['enabled'] and sensors[name]['frame'] != parent]

        if sensors['camera']['enabled']:
            new_frames.extend(sensors['camera'][key] for key in ('frame', 'optical_frame'))

        for frame in new_frames:
            if frame in links or f'{frame}_sensor_mount' in joint_names:
                raise ValueError(f'Sensor frame or mount collides with an existing name: {frame}')

            links.add(frame)
            joint_names.add(f'{frame}_sensor_mount')

    # Xacro owns simulation geometry and sensors; launch supplies paths and upstream links.
    urdf = xacro.process_file(
        str(simulation / 'urdf/go2w.urdf.xacro'),
        mappings={
            'description_file': str(description_file),
            'settings_file': str(settings_file),
            'sensors_file': str(sensors_file) if sensors_file is not None else '',
            'visual_links': ' '.join(link.get('name') for link in robot.findall('link')
                                     if link.find('visual') is not None),
            'controller_file': str(controller_file),
            'robot_name': robot_name,
            'enable_sensors': str(sensors is not None).lower()
        }).toxml()

    controllers = yaml.safe_load((simulation / 'config/gz_ros2_control.yaml').read_text())
  
    controllers['controller_manager']['ros__parameters']['update_rate'] = (
        settings['controller_update_rate'])
  
    controllers['leg_controller']['ros__parameters']['joints'] = LEG_JOINTS

    # Derive controller wheel spacing from the description and the adapted collision offset.
    hip_y = float(joints['FL_hip_joint'].find('origin').get('xyz').split()[1])
    thigh_y = float(joints['FL_thigh_joint'].find('origin').get('xyz').split()[1])
    drive = controllers['drive_controller']['ros__parameters']
    drive.update({'wheel_radius': settings['wheel_radius'],
                  'wheel_separation': 2 * (hip_y + thigh_y + settings['wheel_lateral_offset']),
                  'cmd_vel_timeout': settings['cmd_vel_timeout']})

    for axis, kind in (('linear.x', 'linear'), ('angular.z', 'angular')):
        velocity, acceleration = (settings[f'max_{kind}_velocity'],
                                  settings[f'max_{kind}_acceleration'])
        drive.update({f'{axis}.max_velocity': velocity,
                      f'{axis}.min_velocity': -velocity,
                      f'{axis}.max_acceleration': acceleration,
                      f'{axis}.max_deceleration': -acceleration})

    # Build adapter parameters from the validated values used to generate the sensor model.
    parameters = None
    substitutions = {'world_name': world, 'robot_name': robot_name}

    if sensors is not None:
        parameters = {'use_sim_time': True}

        for name in SENSOR_NAMES:
            config = sensors[name]
            parameters.update({f'{name}_enabled': config['enabled'],
                               f'{name}_frame': config['frame']})

            for key in ('gyroscope_noise_stddev', 'accelerometer_noise_stddev'):
                if key in config:
                    parameters[f'{name}_{key}'] = float(config[key])

        parameters.update({'temperature': sensors['body_imu']['temperature'],
                           'range_min': float(sensors['lidar']['range_min']),
                           'range_max': float(sensors['lidar']['range_max']),
                           'camera_enabled': sensors['camera']['enabled']})

        camera = sensors['camera']

        if camera['enabled']:
            # Scale the intrinsic matrix independently along the image's horizontal/vertical axes.
            sx, sy = (camera['width'] / camera['reference_width'],
                      camera['height'] / camera['reference_height'])
  
            parameters.update({
                'camera_frame': camera['optical_frame'],
                'camera_width': camera['width'],
                'camera_height': camera['height'],
                'camera_image_topic': camera['image_topic'],
                'camera_info_topic': camera['info_topic'],
                'camera_intrinsics': [camera['fx'] * sx, 0.0, camera['cx'] * sx,
                                      0.0, camera['fy'] * sy, camera['cy'] * sy,
                                      0.0, 0.0, 1.0]})
  
            substitutions.update(camera_image_topic=camera['image_topic'],
                                 camera_info_topic=camera['info_topic'])

    # Omit disabled sensor bridges and leave clock ownership to the launch argument.
    bridges = []

    for entry in yaml.safe_load((simulation / 'config/ros_gz_bridge.yaml').read_text()):
        sensor = entry.pop('sensor', None)

        if sensor and (sensors is None or not sensors[sensor]['enabled']):
            continue

        if entry['ros_topic_name'] == '/clock' and not bridge_clock:
            continue

        for key in ('ros_topic_name', 'gz_topic_name'):
            entry[key] = entry[key].format(**substitutions)

        bridges.append(entry)

    return urdf, controllers, bridges, parameters


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


async def run_command(context, command, timeout):
    """
    @brief Run a bounded command asynchronously and clean up its subprocess group.

    @param context: Launch context supplying the subprocess environment.
    @param command: Executable and command-line arguments as a sequence of strings.
    @param timeout: Maximum wall-clock seconds to wait for command output and completion.
    @return: Decoded combined standard output and standard error.
    @throws RuntimeError: If the command exits with a nonzero status.
    @throws asyncio.TimeoutError: If command completion exceeds the timeout.
    """
    # Give each command its own process group so timeout cleanup only targets its children.
    process = await asyncio.create_subprocess_exec(
        *command, env=context.environment, start_new_session=True,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)

    try:
        output, _ = await asyncio.wait_for(process.communicate(), timeout)
        output = output.decode(errors='replace')

        if process.returncode != 0:
            raise RuntimeError(
                f'{command[0]} failed with exit code {process.returncode}: {output.strip()}')

        return output
    finally:
        # A CLI wrapper can exit before its children. Only signal this command's group.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

        await process.communicate()


async def poll_until(check, timeout, failure):
    """
    @brief Retry an asynchronous readiness check within a wall-clock deadline.

    @param check: Async callable accepting a per-attempt timeout in seconds and returning
                 a truthy value when ready. Per-attempt timeouts are retried.
    @param timeout: Total polling budget in wall-clock seconds.
    @param failure: Error message used when readiness is not reached.
    @return: None when the check succeeds.
    @throws RuntimeError: If the polling deadline expires.
    """
    # Use monotonic wall time because simulation time may not be available during startup.
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout

    while (remaining := deadline - loop.time()) > 0:
        try:
            if await check(min(3.0, remaining)):
                return
        except asyncio.TimeoutError:
            pass

        # Yield to launch so other processes and shutdown handlers remain responsive.
        await asyncio.sleep(min(0.25, max(0, deadline - loop.time())))

    raise RuntimeError(failure)


async def scene_models(context, world, timeout):
    """
    @brief Query the top-level model names in a Gazebo world.

    @param context: Launch context supplying the Gazebo command environment.
    @param world: Name of the running Gazebo world.
    @param timeout: Command timeout in wall-clock seconds.
    @return: List of model names parsed from the scene-information response.
    @throws RuntimeError: If the scene query command fails.
    @throws asyncio.TimeoutError: If the scene query exceeds the timeout.
    """
    output = await run_command(
        context,
        ['gz', 'service', '-s', f'/world/{world}/scene/info',
         '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
         '--timeout', '2000', '--req', ''],
        timeout)

    # Protobuf text prints each top-level model with its name first.
    return re.findall(r'^model \{\s+name: "([^"]+)"', output, re.MULTILINE)


async def world_ready(context, services, limit):
    """
    @brief Check that the world's creation and scene-information services exist.

    @param context: Launch context supplying the Gazebo command environment.
    @param services: Required Gazebo service names.
    @param limit: Maximum wall-clock seconds for this service-list query.
    @return: True when both required services are advertised.
    @throws RuntimeError: If the service-list query command fails.
    @throws asyncio.TimeoutError: If the query exceeds the timeout.
    """
    output = await run_command(context, ['gz', 'service', '--list'], limit)
    return services.issubset(output.splitlines())


async def model_present(context, world, name, limit):
    """
    @brief Check whether the requested robot appears in the Gazebo scene.

    @param context: Launch context supplying the Gazebo command environment.
    @param world: Name of the running Gazebo world.
    @param name: Requested robot entity name.
    @param limit: Maximum wall-clock seconds for this scene query.
    @return: True when the scene contains the requested entity name.
    @throws RuntimeError: If the scene query command fails.
    @throws asyncio.TimeoutError: If the query exceeds the timeout.
    """
    return name in await scene_models(context, world, limit)


async def spawn_robot(context, world, name, model_file, pose, timeout, after):
    """
    @brief Wait for Gazebo, insert the robot, and confirm it before continuing startup.

    @param context: Active launch context and subprocess environment.
    @param world: Name of the running Gazebo world.
    @param name: Unique entity name requested for the robot.
    @param model_file: Path to the generated URDF file accessible to the Gazebo server.
    @param pose: Initial x, y, z coordinates in metres and R, P, Y angles in radians.
    @param timeout: Wall-clock timeout in seconds for each readiness or insertion stage.
    @param after: Launch actions to release after the entity appears in the scene.
    @return: None after startup actions are released or shutdown cancels insertion.
    @throws RuntimeError: If the world is unavailable, the name exists, or insertion fails.
    """
    services = {f'/world/{world}/create', f'/world/{world}/scene/info'}
    world_check = partial(world_ready, context, services)
    model_check = partial(model_present, context, world, name)

    try:
        # Wait for both services before checking names or requesting insertion.
        print(f'Waiting for Gazebo services in world {world!r}', flush=True)
        
        await poll_until(world_check, timeout,
                         f'Timed out waiting for {sorted(services)}. '
                         'Is the world running with UserCommands and SceneBroadcaster?')

        try:
            existing = await scene_models(context, world, min(3.0, timeout))
        except asyncio.TimeoutError as error:
            raise RuntimeError('Timed out checking the existing Gazebo scene') from error

        # Refuse duplicate names before create can attach startup to an existing robot.
        if name in existing:
            raise RuntimeError(f'Robot insertion refused: entity {name!r} already exists in '
                               f'world {world!r}. Remove it before spawning again.')

        # Run the installed creator directly; launch owns its timeout and subprocess cleanup.
        creator = Path(get_package_prefix('ros_gz_sim')) / 'lib/ros_gz_sim/create'
        
        command = [str(creator), '-world', world,
                   '-name', name,
                   '-file', str(model_file),
                   '-allow_renaming', 'false']

        for coordinate, setting in pose.items():
            command.extend([f'-{coordinate}', str(setting)])

        try:
            output = await run_command(context, command, timeout)
        except asyncio.TimeoutError as error:
            raise RuntimeError('Robot insertion timed out') from error

        if output.strip():
            print(output.rstrip(), flush=True)

        # A create acknowledgement only queues insertion; confirm the model actually exists.
        await poll_until(model_check, timeout,
                         f'Robot insertion failed: entity {name!r} did not appear in the scene')
        print(f'Confirmed Gazebo entity {name!r}', flush=True)

        if not context.is_shutdown:
            # Release controller startup only after Gazebo confirms the entity exists.
            await context.emit_event(IncludeLaunchDescription(LaunchDescription(after)))
    except asyncio.CancelledError:
        # Launch shutdown cancels this coroutine as part of normal cleanup.
        if not context.is_shutdown:
            raise


def checked_exit(event, context, label, actions):
    """
    @brief Validate the process exit status and release the next startup actions.

    @param event: Process-exit event containing the return code.
    @param context: Launch context supplied by the event handler.
    @param label: Process description used in failure diagnostics.
    @param actions: Launch actions supplied by the callback binding.
    @return: The supplied actions when the process exits successfully.
    @throws RuntimeError: If the process exits with a nonzero status.
    """
    if event.returncode != 0:
        raise RuntimeError(f'{label} failed with exit code {event.returncode}')

    return actions


def startup_readiness(context, simulation, timeout, sensor_parameters):
    """
    @brief Build sensor and telemetry readiness gates before driving and optional RViz.

    @param context: Launch context containing RViz and LowState configuration arguments.
    @param simulation: Share directory of go2w_gazebo.
    @param timeout: Readiness timeout in wall-clock seconds.
    @param sensor_parameters: Sensor adapter parameters, or None when sensors are disabled.
    @return: Tuple of initial readiness actions and actions to start after controller activation.
    @throws ValueError: If a boolean argument is invalid or the LowState configuration is missing.
    """
    adapter = Node(package='go2w_gazebo',
                   executable='cmd_vel_adapter',
                   output='screen',
                   parameters=[{'use_sim_time': True}])

    # Driving and optional RViz start only after all enabled readiness checks succeed.
    ready = [adapter]

    if enabled(context, 'rviz'):
        ready.append(Node(package='rviz2',
                          executable='rviz2',
                          name='rviz2',
                          output='screen',
                          arguments=['-d', str(simulation / 'rviz/go2w.rviz')],
                          parameters=[{'use_sim_time': True}]))

    telemetry_actions = []
    telemetry_ready = ready
    telemetry_start = []

    if enabled(context, 'enable_lowstate'):
        lowstate_config = Path(value(context, 'lowstate_config')).expanduser().resolve()
        if not lowstate_config.is_file():
            raise ValueError(f'LowState configuration does not exist: {lowstate_config}')

        lowstate = Node(package='go2w_gazebo',
                        executable='lowstate_publisher',
                        output='screen',
                        parameters=[str(lowstate_config), {
                            'use_sim_time': True,
                            'startup_timeout': timeout,
                            'body_imu_enabled': bool(
                                sensor_parameters
                                and sensor_parameters.get('body_imu_enabled', False))}])

        # A separate check process exits successfully once native telemetry is ready.
        lowstate_readiness = Node(package='go2w_gazebo',
                                  executable='lowstate_publisher',
                                  name='lowstate_readiness',
                                  output='screen',
                                  parameters=[{'use_sim_time': True,
                                               'check_only': True,
                                               'startup_timeout': timeout}])

        telemetry_start = [lowstate]
        telemetry_ready = [lowstate_readiness]
        
        # Register both readiness progression and failure handling for the live publisher.
        telemetry_actions = [
            RegisterEventHandler(
                OnProcessExit(target_action=lowstate_readiness,
                              on_exit=partial(checked_exit,
                                              label='LowState readiness', actions=ready))),
            RegisterEventHandler(
                OnProcessExit(target_action=lowstate,
                              on_exit=partial(checked_exit,
                                              label='LowState publisher', actions=[])))]

    sensor_actions = []
    
    # Without sensors, controller activation proceeds directly to the telemetry gate.
    after_controllers = telemetry_start + telemetry_ready

    has_enabled_sensors = sensor_parameters is not None and any(
        sensor_parameters[f'{name}_enabled'] for name in (*SENSOR_NAMES, 'camera'))
    
    if has_enabled_sensors:
        sensors = Node(package='go2w_gazebo',
                       executable='sensor_adapter',
                       output='screen',
                       parameters=[sensor_parameters])

        sensor_readiness = Node(package='go2w_gazebo',
                                executable='sensor_adapter',
                                name='sensor_readiness',
                                output='screen',
                                parameters=[sensor_parameters,
                                            {'check_only': True, 'startup_timeout': timeout}])

        # Start telemetry alongside sensor checks, then check telemetry before enabling driving.
        after_controllers = telemetry_start + [sensor_readiness]
    
        sensor_actions = [
            RegisterEventHandler(
                OnProcessExit(target_action=sensor_readiness,
                              on_exit=partial(checked_exit,
                                              label='Sensor readiness', actions=telemetry_ready))),
            RegisterEventHandler(
                OnProcessExit(target_action=sensors,
                              on_exit=partial(checked_exit,
                                              label='Sensor adapter', actions=[]))),
            sensors]

    return telemetry_actions + sensor_actions, after_controllers


def cleanup_directory(context, directory):
    """
    @brief Remove the temporary robot resources when launch shuts down.

    @param context: Launch context supplied by the shutdown action.
    @param directory: TemporaryDirectory object retained by the action arguments.
    @return: None after the temporary directory is cleaned up.
    """
    directory.cleanup()


def spawn_actions(context):
    """
    @brief Prepare temporary robot resources and assemble the guarded spawn sequence.

    @param context: Launch context containing robot, world, pose, and configuration arguments.
    @return: Launch actions for cleanup, readiness, state publication, bridging, and insertion.
    @throws ValueError: If launch arguments or simulation configuration are invalid.
    """
    simulation = Path(get_package_share_directory('go2w_gazebo'))
    description = Path(get_package_share_directory('go2w_description'))

    # Validate identifiers, timing, and pose before creating temporary files or starting nodes.
    name = value(context, 'robot_name')
    
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
        raise ValueError('robot_name must start with a letter and contain letters, numbers or _')

    world = value(context, 'world_name')
    
    if not world or '/' in world:
        raise ValueError('world_name must be a nonempty Gazebo world name without /')

    timeout = float(value(context, 'startup_timeout'))
    
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('startup_timeout must be finite and positive')

    pose = {key: float(value(context, key)) for key in ('x', 'y', 'z', 'R', 'P', 'Y')}
    
    for coordinate, setting in pose.items():
        if not math.isfinite(setting):
            raise ValueError(f'{coordinate} must be finite')

    # Shared /dev/shm paths also work when Gazebo runs in another container.
    temporary_directory = tempfile.TemporaryDirectory(prefix='go2w_gazebo_', dir='/dev/shm')
    temporary = Path(temporary_directory.name)

    # Keep temporary model/configuration files available until launch shuts down.
    cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(
            function=cleanup_directory, args=[temporary_directory])]))

    controller_file = temporary / 'controllers.yaml'
    
    # Global sensor disable skips loading the file, even if its configured path is missing.
    sensors_file = (Path(value(context, 'sensors_config'))
                    if enabled(context, 'enable_sensors') else None)

    urdf, controllers, bridges, sensor_parameters = prepare_simulation(
        description, simulation, simulation / 'config/parameters.yaml', sensors_file,
        controller_file, name, temporary,
        world=world, bridge_clock=enabled(context, 'bridge_clock'))
    
    controller_file.write_text(yaml.safe_dump(controllers))

    model_file = temporary / 'go2w.urdf'
    model_file.write_text(urdf)

    bridge_file = temporary / 'bridges.yaml'
    bridge_file.write_text(yaml.safe_dump(bridges))

    # Publish the generated model for TF; force XML to stay a string during parameter parsing.
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               name='go2w_state_publisher',
               output='screen',
               remappings=[('robot_description', ROBOT_DESCRIPTION_TOPIC)],
               parameters=[{'robot_description': ParameterValue(urdf, value_type=str),
                            'use_sim_time': True}])

    # Use the filtered bridge configuration for the selected world and enabled sensors.
    bridge = Node(package='ros_gz_bridge',
                  executable='parameter_bridge',
                  name='gazebo_bridge',
                  parameters=[{'config_file': str(bridge_file), 'use_sim_time': True}],
                  output='screen')

    # Activate the joint broadcaster, leg hold, and wheel drive as a group before readiness.
    spawner = Node(package='controller_manager',
                   executable='spawner',
                   output='screen',
                   arguments=['joint_state_broadcaster', 'leg_controller', 'drive_controller',
                              '--activate-as-group',
                              '--controller-manager-timeout', str(timeout),
                              '--switch-timeout', str(timeout),
                              '--service-call-timeout', str(timeout),
                              '--param-file', str(controller_file)])

    readiness, after_controllers = startup_readiness(
        context, simulation, timeout, sensor_parameters)

    return [
        cleanup,
        *readiness,
        # Register the exit handler before insertion can release the controller spawner.
        RegisterEventHandler(
            OnProcessExit(target_action=spawner,
                          on_exit=partial(checked_exit,
                                          label='Controller activation',
                                          actions=after_controllers))),
        rsp,
        bridge,
        OpaqueCoroutine(coroutine=spawn_robot,
                        args=[world, name, model_file, pose, timeout, [spawner]])]


def generate_launch_description():
    """
    @brief Declare robot arguments and register deferred spawning into an existing world.

    @return: Launch description containing argument declarations and the spawn callback.
    """
    simulation = Path(get_package_share_directory('go2w_gazebo'))

    defaults = {'world_name': ('default', 'Name of the already-running Gazebo world.'),
                'robot_name': ('go2w', 'Gazebo entity name; one robot per ROS domain.'),

                'x': ('0.0', 'Initial x position [m].'),
                'y': ('0.0', 'Initial y position [m].'),
                'z': ('0.43', 'Initial base height [m].'),
                'R': ('0.0', 'Initial roll [rad].'),
                'P': ('0.0', 'Initial pitch [rad].'),
                'Y': ('0.0', 'Initial yaw [rad].'),

                'bridge_clock': ('false', 'Publish this world clock on /clock; enable only one bridge.'),
                'rviz': ('false', 'Open RViz after controller activation.'),

                'enable_sensors': ('true', 'Simulate the configured IMU, LiDAR and RGB camera sensors.'),
                'sensors_config': (str(simulation / 'config/sensors.yaml'), 'Sensor configuration YAML.'),

                'enable_lowstate': ('true', 'Publish native Unitree low-level telemetry.'),
                'lowstate_config': (str(simulation / 'config/lowstate.yaml'),
                                    'LowState ROS parameters YAML.'),

                'startup_timeout': ('60.0', 'Wall-clock timeout for world and controller readiness [s].')}

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default,
                                      description=description)
          for name, (default, description) in defaults.items()],
        # Prepare the model only after launch argument overrides have been resolved.
        OpaqueFunction(function=spawn_actions)])
