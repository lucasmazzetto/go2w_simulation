"""Check the simulation model against the manufacturer's joint and asset contract."""

import math
import xml.etree.ElementTree as ET

import pytest
import yaml

LEGS = ('FL', 'FR', 'RL', 'RR')
LEG_JOINTS = [f'{leg}_{part}_joint' for leg in LEGS for part in ('hip', 'thigh', 'calf')]
WHEEL_JOINTS = [f'{leg}_foot_joint' for leg in LEGS]
ROBOT_DESCRIPTION_TOPIC = '/go2w_gazebo/robot_description'


@pytest.fixture
def model(resources, launch_module, compose):
    description, simulation = resources
    settings = launch_module.load_settings(simulation / 'config/parameters.yaml')
    xml, controllers, _, _ = compose(settings=settings)
    return description, settings, ET.fromstring(xml), controllers


def test_simulation_preserves_visuals_and_inertias(model):
    description, _, generated, _ = model
    original = ET.parse(description / 'urdf/go2w_description.urdf').getroot()
    for link in original.findall('link'):
        result = generated.find(f"link[@name='{link.get('name')}']")
        assert result is not None
        for tag in ('inertial', 'visual'):
            # Xacro normalizes XML whitespace and attribute order.
            assert [ET.canonicalize(ET.tostring(e), strip_text=True) for e in link.findall(tag)] == [
                ET.canonicalize(ET.tostring(e), strip_text=True) for e in result.findall(tag)]
    for mesh in generated.findall('.//mesh'):
        uri = mesh.get('filename')
        assert uri.startswith('package://go2w_description/')
        assert (description / uri.removeprefix('package://go2w_description/')).is_file()


def test_control_interfaces_and_initial_pose(model):
    _, settings, robot, controllers = model
    controlled = {j.get('name'): j for j in robot.findall('ros2_control/joint')}
    assert set(controlled) == set(LEG_JOINTS + WHEEL_JOINTS)
    for name, joint in controlled.items():
        command = joint.find('command_interface').get('name')
        assert command == ('velocity' if name in WHEEL_JOINTS else 'position')
        initial = float(joint.find("state_interface[@name='position']/param").text)
        expected = 0 if name in WHEEL_JOINTS else settings[f'standing_{name.split("_")[1]}']
        assert initial == expected
    drive = controllers['drive_controller']['ros__parameters']
    assert drive['left_wheel_names'] == ['FL_foot_joint', 'RL_foot_joint']
    assert drive['right_wheel_names'] == ['FR_foot_joint', 'RR_foot_joint']
    assert drive['wheel_radius'] == settings['wheel_radius']
    assert math.isclose(drive['wheel_separation'], 0.380184)
    assert drive['open_loop'] is False


def test_wheel_collisions_follow_mesh_offsets(model):
    _, settings, robot, _ = model
    for side, sign in [('FL', 1), ('FR', -1), ('RL', 1), ('RR', -1)]:
        collisions = robot.findall(f"link[@name='{side}_foot']/collision")
        assert len(collisions) == 1
        origin = collisions[0].find('origin')
        assert float(origin.get('xyz').split()[1]) == sign * settings['wheel_lateral_offset']
        assert math.isclose(float(origin.get('rpy').split()[0]), math.pi / 2)
        cylinder = collisions[0].find('geometry/cylinder')
        assert float(cylinder.get('radius')) == settings['wheel_radius']
        assert float(cylinder.get('length')) == settings['wheel_width']


def test_control_and_rviz_use_simulation_description(model, resources):
    robot = model[2]
    plugin = robot.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']")
    remappings = [element.text for element in plugin.findall('ros/remapping')]
    assert f'/robot_description:={ROBOT_DESCRIPTION_TOPIC}' in remappings
    assert ROBOT_DESCRIPTION_TOPIC != '/robot_description'
    simulation = resources[1]
    rviz = yaml.safe_load((simulation / 'rviz/go2w.rviz').read_text())
    robot_display = next(display for display in rviz['Visualization Manager']['Displays']
                         if display['Class'] == 'rviz_default_plugins/RobotModel')
    assert robot_display['Description Topic']['Value'] == ROBOT_DESCRIPTION_TOPIC


def test_invalid_standing_posture_is_rejected(model, compose):
    description, settings, _, _ = model
    settings = {**settings, 'standing_calf': 0.0}
    with pytest.raises(ValueError, match='outside its limits'):
        compose(settings=settings)


def test_custom_settings_reach_xacro_and_controllers(model, compose):
    settings = {**model[1], 'wheel_radius': 0.09, 'wheel_width': 0.06,
                'wheel_lateral_offset': 0.05, 'wheel_friction': 0.7,
                'standing_thigh': 0.9, 'controller_update_rate': 250,
                'position_proportional_gain': 0.25,
                'max_linear_velocity': 0.3, 'cmd_vel_timeout': 0.8}
    xml, controllers, bridges, _ = compose(settings=settings, robot_name='custom_robot')
    robot = ET.fromstring(xml)
    assert robot.get('name') == 'custom_robot'
    wheel = robot.find("link[@name='FL_foot']/collision")
    assert float(wheel.find('geometry/cylinder').get('radius')) == 0.09
    assert float(wheel.find('geometry/cylinder').get('length')) == 0.06
    assert list(map(float, wheel.find('origin').get('xyz').split())) == [0, 0.05, 0]
    assert float(robot.find("gazebo[@reference='FL_foot']/mu1").text) == 0.7
    assert float(robot.find("ros2_control/joint[@name='FL_thigh_joint']/state_interface/param").text) == 0.9
    assert float(robot.find('gazebo/plugin/position_proportional_gain').text) == 0.25
    drive = controllers['drive_controller']['ros__parameters']
    assert drive['wheel_radius'] == 0.09
    assert drive['linear.x.max_velocity'] == 0.3
    assert drive['cmd_vel_timeout'] == 0.8
    assert controllers['controller_manager']['ros__parameters']['update_rate'] == 250
    assert bridges[0]['gz_topic_name'] == '/model/custom_robot/pose'


@pytest.mark.parametrize('bridge_clock', [False, True])
def test_physics_only_description_and_clock_bridge(compose, bridge_clock):
    xml, _, bridges, parameters = compose(bridge_clock=bridge_clock)
    robot = ET.fromstring(xml)
    assert robot.findall('gazebo/sensor') == []
    assert robot.find("link[@name='front_camera']") is None
    assert parameters is None
    assert {entry['ros_topic_name'] for entry in bridges} == (
        {'/clock', '/ground_truth/poses'} if bridge_clock else {'/ground_truth/poses'})
    if bridge_clock:
        assert bridges[0]['gz_topic_name'] == '/world/test_world/clock'


def test_direct_configuration_paths_with_spaces(resources, launch_module, tmp_path, monkeypatch):
    description, simulation = resources
    config = tmp_path / "configuration with spaces and 'quotes'"
    config.mkdir()
    settings_file = config / 'custom parameters.yaml'
    settings = launch_module.load_settings(simulation / 'config/parameters.yaml')
    settings['wheel_radius'] = 0.091
    settings_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': settings}}))
    sensors_file = config / 'custom sensors.yaml'
    sensors_file.write_text((simulation / 'config/sensors.yaml').read_text())
    directory = tmp_path / 'runtime with spaces'
    directory.mkdir()
    monkeypatch.chdir(tmp_path)
    xml, controllers, _, parameters = launch_module.prepare_simulation(
        description, simulation, settings_file.relative_to(tmp_path),
        sensors_file.relative_to(tmp_path), directory / 'controllers.yaml', 'paths', directory)
    robot = ET.fromstring(xml)
    wheel = robot.find("link[@name='FL_foot']/collision/geometry/cylinder")
    assert float(wheel.get('radius')) == controllers['drive_controller']['ros__parameters']['wheel_radius'] == 0.091
    assert len(robot.findall('gazebo/sensor')) == 4
    assert parameters['body_imu_enabled'] and parameters['camera_enabled']
    assert len(robot.findall('gazebo/visual/visibility_flags')) == len(robot.findall('link/visual'))
    assert {path.name for path in directory.iterdir()} == {'description.urdf.xacro'}


@pytest.mark.parametrize('key,value', [('wheel_radius', 0), ('controller_update_rate', True)])
def test_configuration_is_validated_before_expansion(model, compose, key, value):
    settings = {**model[1], key: value}
    with pytest.raises(ValueError, match=key):
        compose(settings=settings)


def test_xacro_skips_nonexistent_sensor_file_when_disabled(resources, launch_module, tmp_path):
    import xacro

    description, simulation = resources
    settings_file = simulation / 'config/parameters.yaml'
    launch_module.prepare_simulation(
        description, simulation, settings_file, None, tmp_path / 'controllers.yaml',
        'disabled', tmp_path)
    xml = xacro.process_file(str(simulation / 'urdf/go2w.urdf.xacro'), mappings={
        'description_file': str(tmp_path / 'description.urdf.xacro'),
        'settings_file': str(settings_file),
        'sensors_file': str(tmp_path / 'nonexistent.yaml'),
        'controller_file': str(tmp_path / 'controllers.yaml'),
        'robot_name': 'disabled', 'enable_sensors': 'false', 'visual_links': '',
    }).toxml()
    robot = ET.fromstring(xml)
    assert robot.findall('gazebo/sensor') == []
    assert robot.findall('gazebo/visual/visibility_flags') == []
