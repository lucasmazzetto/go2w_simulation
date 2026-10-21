"""Check sensor frames, physical invariants and installed configuration."""

import math
import xml.etree.ElementTree as ET

import pytest
import yaml

from test_utils import camera_intrinsics, rotation


def compose_pose(parent, local):
    xyz, rot = parent
    local_xyz, local_rot = local
    return ([xyz[i] + sum(rot[i][j] * local_xyz[j] for j in range(3)) for i in range(3)],
            [[sum(rot[i][k] * local_rot[k][j] for k in range(3))
              for j in range(3)] for i in range(3)])


def link_pose(robot, link):
    """Compose the description's joint origins independently of Xacro/Gazebo."""
    if link == 'base':
        return [0, 0, 0], rotation([0, 0, 0])
    joint = robot.find(f"joint/child[@link='{link}']/..")
    assert joint is not None and joint.get('type') == 'fixed'
    origin = joint.find('origin')
    xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
    rpy = list(map(float, origin.get('rpy', '0 0 0').split()))
    return compose_pose(link_pose(robot, joint.find('parent').get('link')),
                        (xyz, rotation(rpy)))


@pytest.fixture
def sensor_model(resources, launch_module, compose):
    simulation = resources[1]
    settings = launch_module.load_sensors(simulation / 'config/sensors.yaml')
    before = ET.fromstring(compose()[0])
    xml, _, bridges, parameters = compose(sensors=settings)
    bridges = [b for b in bridges if b['ros_topic_name'] not in ('/clock', '/ground_truth/poses')]
    return simulation, settings, before, ET.fromstring(xml), bridges, parameters


@pytest.fixture
def sensor_description(compose):
    def expand(settings, robot_name):
        xml, _, bridges, parameters = compose(sensors=settings, robot_name=robot_name)
        return (ET.fromstring(xml),
                [b for b in bridges if b['ros_topic_name'] not in ('/clock', '/ground_truth/poses')],
                parameters)
    return expand


def test_sensor_physical_invariants(sensor_model):
    _, _, before, after, _, _ = sensor_model
    for link in before.findall('link'):
        added = after.find(f"link[@name='{link.get('name')}']")
        assert ET.tostring(link) == ET.tostring(added)
    for joint in before.findall('joint'):
        added = after.find(f"joint[@name='{joint.get('name')}']")
        assert ET.tostring(joint) == ET.tostring(added)
    assert sum(j.get('type') != 'fixed' for j in after.findall('joint')) == 16
    assert after.find("link[@name='utlidar_imu']/inertial") is None
    assert all(v.text == '2' for v in after.findall('gazebo/visual/visibility_flags'))
    assert len(after.findall('gazebo/visual/visibility_flags')) == len(before.findall('link/visual'))


def test_sensor_poses_and_sdk_extrinsic(sensor_model):
    robot = sensor_model[3]
    sensors = {s.get('name'): s for s in robot.findall('gazebo/sensor')}
    assert len(sensors) == 4
    offset = [-0.007698, -0.014655, 0.00667]
    for name, parent, pose in (('body_imu', 'imu', [0]*6),
                               ('lidar', 'radar', [0]*6),
                               ('lidar_imu', 'radar', offset + [0]*3)):
        assert robot.find(f"gazebo/sensor[@name='{name}']/..").get('reference') == parent
        assert list(map(float, sensors[name].find('pose').text.split())) == pose
    fixed = robot.find("joint[@name='utlidar_imu_sensor_mount']")
    assert fixed.find('parent').get('link') == 'radar'
    assert list(map(float, fixed.find('origin').get('xyz').split())) == offset
    assert list(map(float, fixed.find('origin').get('rpy').split())) == [0]*3
    for name in ('body_imu', 'lidar_imu'):
        assert sensors[name].find('imu/orientation_reference_frame/custom_rpy').get('parent_frame') == 'world'


def test_lidar_sampling_and_bridges(sensor_model):
    _, _, _, robot, bridges, parameters = sensor_model
    lidar = robot.find("gazebo/sensor[@name='lidar']/lidar")
    horizontal = lidar.find('scan/horizontal')
    step = (float(horizontal.find('max_angle').text) - float(horizontal.find('min_angle').text)) / 119
    assert step == pytest.approx(2*math.pi/120)
    assert lidar.find('scan/vertical/samples').text == '18'
    assert lidar.find('visibility_mask').text == '1'
    assert next(b for b in bridges if b['ros_type_name'].endswith('/PointCloud2'))[
        'gz_topic_name'] == '/go2w_gazebo/example/lidar/points'
    assert all(b['direction'] == 'GZ_TO_ROS' for b in bridges)
    assert parameters['body_imu_enabled'] and parameters['lidar_imu_enabled']


@pytest.mark.parametrize('disabled', ['body_imu', 'lidar_imu', 'lidar'])
def test_individual_disabling_and_custom_frames(sensor_model, sensor_description, disabled):
    _, settings, before, _, _, _ = sensor_model
    settings[disabled]['enabled'] = False
    settings['body_imu']['frame'] = 'custom_body_imu'
    settings['lidar']['frame'] = 'front_lidar'
    settings['lidar_imu']['frame'] = 'front_lidar_imu'
    robot, bridges, parameters = sensor_description(settings, 'renamed')
    assert len(bridges) == 4 and not parameters[f'{disabled}_enabled']
    assert robot.find(f"gazebo/sensor[@name='{disabled}']") is None
    for name, parent in (('body_imu', 'imu'), ('lidar_imu', 'radar'), ('lidar', 'radar')):
        frame = settings[name]['frame']
        joint = robot.find(f"joint[@name='{frame}_sensor_mount']")
        if name == disabled:
            assert joint is None
        else:
            assert joint.find('parent').get('link') == parent
            assert robot.find(f"gazebo/sensor[@name='{name}']/gz_frame_id").text == frame
    settings['lidar_imu']['enabled'] = True
    settings['lidar_imu']['frame'] = 'base'
    with pytest.raises(ValueError, match='collides'):
        sensor_description(settings, 'renamed')


@pytest.mark.parametrize('section,key,value', [
    ('lidar', 'horizontal_samples', 1), ('lidar', 'range_max', 0.01),
    ('lidar', 'vertical_samples', 65537), ('body_imu', 'update_rate', float('nan')),
    ('lidar_imu', 'gyroscope_noise_stddev', -1), ('body_imu', 'temperature', 128),
    ('lidar', 'enabled', 'true'), ('lidar', 'frame', 'a/b'),
])
def test_invalid_sensor_configuration(sensor_model, tmp_path, section, key, value, compose):
    settings = sensor_model[1]
    settings[section][key] = value
    path = tmp_path / 'sensors.yaml'
    path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError):
        compose(sensors_file=path)


@pytest.mark.parametrize('changed_mounts', [False, True], ids=['default', 'translated_rotated'])
def test_converted_sdf_retains_sensors_and_masks(
        sensor_model, resources, compose, tmp_path, changed_mounts):
    import subprocess

    original = ET.parse(resources[0] / 'urdf/go2w_description.urdf').getroot()
    if changed_mounts:
        # Change only a disposable description, with rotation around every axis.
        for frame, xyz, rpy in (('imu', '0.13 -0.07 0.21', '0.37 -0.28 0.53'),
                                ('radar', '-0.18 0.12 0.09', '-0.41 0.63 -0.72')):
            origin = original.find(f"joint/child[@link='{frame}']/../origin")
            origin.set('xyz', xyz)
            origin.set('rpy', rpy)
        description = tmp_path / 'description'
        (description / 'urdf').mkdir(parents=True)
        ET.ElementTree(original).write(description / 'urdf/go2w_description.urdf')
        robot = ET.fromstring(compose(sensors=sensor_model[1], description=description)[0])
    else:
        robot = sensor_model[3]
    path = tmp_path / 'robot.urdf'
    path.write_text(ET.tostring(robot, encoding='unicode'))
    converted = subprocess.run(['gz', 'sdf', '-p', str(path)], check=True,
                               capture_output=True, text=True, timeout=30)
    sdf = ET.fromstring(converted.stdout)
    sensors = sdf.findall('.//sensor')
    assert {s.get('name') for s in sensors} == {'body_imu', 'lidar_imu', 'lidar', 'camera'}
    # Standard fixed-joint reduction moves all four sensors onto base.
    assert len(sdf.findall("model/link[@name='base']/sensor")) == 4
    camera_config = sensor_model[1]['camera']
    local_poses = {
        'body_imu': ('imu', [0]*3, [0]*3),
        'lidar': ('radar', [0]*3, [0]*3),
        'lidar_imu': ('radar', [-0.007698, -0.014655, 0.00667], [0]*3),
        'camera': ('base', camera_config['xyz'], camera_config['rpy']),
    }
    for sensor in sensors:
        name = sensor.get('name')
        parent, xyz, rpy = local_poses[name]
        expected_xyz, expected_rotation = compose_pose(
            link_pose(original, parent), (xyz, rotation(rpy)))
        after = [float(v) for v in sensor.find('pose').text.split()]
        assert after[:3] == pytest.approx(expected_xyz, abs=1e-6, rel=0), name
        for expected, actual in zip(expected_rotation, rotation(after[3:])):
            assert actual == pytest.approx(expected, abs=1e-5, rel=0), name
    visuals = sdf.findall('.//link/visual')
    assert visuals and all(v.find('visibility_flags').text == '2' for v in visuals)
    camera = sdf.find(".//sensor[@name='camera']/camera")
    assert camera.find('optical_frame_id').text == 'front_camera_optical'
    assert camera.find('image/width').text == '1280'
    for key, value in zip(('fx', 'fy', 'cx', 'cy'), camera_intrinsics(sensor_model[1]['camera'])):
        assert float(camera.find(f'lens/intrinsics/{key}').text) == pytest.approx(value)


@pytest.mark.parametrize('frame', ['imu', 'radar'])
@pytest.mark.parametrize('invalid', ['missing_link', 'nonfixed_joint', 'wrong_parent'])
def test_required_sensor_mount_validation(sensor_model, resources, compose, tmp_path, frame, invalid):
    original = ET.parse(resources[0] / 'urdf/go2w_description.urdf').getroot()
    joint = original.find(f"joint/child[@link='{frame}']/..")
    if invalid == 'missing_link':
        original.remove(original.find(f"link[@name='{frame}']"))
    elif invalid == 'nonfixed_joint':
        joint.set('type', 'revolute')
    else:
        joint.find('parent').set('link', 'FL_hip')
    description = tmp_path / 'description'
    (description / 'urdf').mkdir(parents=True)
    ET.ElementTree(original).write(description / 'urdf/go2w_description.urdf')
    with pytest.raises(ValueError, match=f'Expected fixed base mounting for {frame}'):
        compose(sensors=sensor_model[1], description=description)


def test_camera_mount_and_projection(sensor_model):
    _, settings, _, robot, bridges, parameters = sensor_model
    camera = settings['camera']
    mount = robot.find("joint[@name='front_camera_sensor_mount']")
    assert mount.find('parent').get('link') == 'base'
    assert list(map(float, mount.find('origin').get('xyz').split())) == camera['xyz']
    optical = robot.find("joint[@name='front_camera_optical_sensor_mount']")
    assert optical.find('parent').get('link') == 'front_camera'
    rot = rotation(list(map(float, optical.find('origin').get('rpy').split())))
    # Optical X points robot-right, optical Y down, optical Z forward.
    for row, expected in zip(rot, ((0, 0, 1), (-1, 0, 0), (0, -1, 0))):
        assert row == pytest.approx(expected)
    for name in ('front_camera', 'front_camera_optical'):
        assert len(robot.find(f"link[@name='{name}']")) == 0
    lens = robot.find("gazebo/sensor[@name='camera']/camera")
    assert lens.find('image/format').text == 'R8G8B8'
    fx, fy, cx, cy = camera_intrinsics(camera)
    assert float(lens.find('horizontal_fov').text) == pytest.approx(2 * math.atan(640 / fx))
    for key, value in zip(('p_fx', 'p_fy', 'p_cx', 'p_cy'), (fx, fy, cx, cy)):
        assert float(lens.find(f'lens/projection/{key}').text) == pytest.approx(value)
    assert all(float(v.text) == 0 for v in lens.findall('distortion/*'))
    assert parameters['camera_intrinsics'] == pytest.approx([fx, 0, cx, 0, fy, cy, 0, 0, 1])
    camera_bridges = [b for b in bridges if b['ros_topic_name'].startswith('/camera/')]
    assert {b['ros_type_name'] for b in camera_bridges} == {
        'sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo'}
    assert all('qos_profile' not in b and b['publisher_queue'] == 10 for b in camera_bridges)


def test_camera_scaling_customization_and_disabling(
        sensor_model, tmp_path, sensor_description, launch_module, compose):
    _, settings, before, _, _, _ = sensor_model
    camera = settings['camera']
    camera.update(width=640, height=360, frame='rgb_mount', optical_frame='rgb_optical',
                  image_topic='/rgb/image_raw', info_topic='/rgb/camera_info',
                  xyz=[0.3, 0.02, 0.05], rpy=[0.1, -0.2, 0.3])
    robot, bridges, parameters = sensor_description(settings, 'changed')
    assert parameters['camera_intrinsics'][0] == pytest.approx(864.39938 / 2)
    assert parameters['camera_intrinsics'][4] == pytest.approx(863.73849 / 2)
    assert parameters['camera_intrinsics'][2] == pytest.approx(639.19798 / 2)
    assert bridges[-1]['ros_topic_name'] == '/rgb/camera_info'
    lens = robot.find("gazebo/sensor[@name='camera']/camera")
    assert lens.find('image/width').text == '640'
    for key, value in zip(('fx', 'fy', 'cx', 'cy'), camera_intrinsics(camera)):
        assert float(lens.find(f'lens/intrinsics/{key}').text) == pytest.approx(value)
    settings['camera']['enabled'] = False
    robot, bridges, parameters = sensor_description(settings, 'disabled')
    assert len(bridges) == 3 and not parameters['camera_enabled']
    assert robot.find("link[@name='rgb_mount']") is None
    assert robot.find("gazebo/sensor[@name='camera']") is None
    del settings['camera']
    path = tmp_path / 'legacy.yaml'
    path.write_text(yaml.safe_dump(settings))
    assert launch_module.load_sensors(path)['camera'] == {'enabled': False}
    # Pass the untouched legacy file so Xacro cannot rely on launch's default injection.
    xml, _, bridges, parameters = compose(sensors_file=path, robot_name='legacy')
    assert not parameters['camera_enabled'] and len(bridges) == 4
    assert ET.fromstring(xml).find("gazebo/sensor[@name='camera']") is None
    assert 'camera' not in yaml.safe_load(path.read_text())


def test_camera_only(sensor_model, sensor_description):
    _, settings, before, _, _, _ = sensor_model
    for name in ('body_imu', 'lidar_imu', 'lidar'):
        settings[name]['enabled'] = False
    robot, bridges, parameters = sensor_description(settings, 'camera_only')
    assert len(bridges) == 2 and parameters['camera_enabled']
    assert {s.get('type') for s in robot.findall('gazebo/sensor')} == {'camera'}


@pytest.mark.parametrize('key,value', [
    ('enabled', 'true'), ('width', 0), ('height', 12.5), ('reference_width', True),
    ('reference_height', -1), ('fx', 0), ('fy', float('nan')), ('cx', 1280),
    ('cy', -1), ('update_rate', 1001), ('clip_near', 100), ('clip_far', -1),
    ('xyz', [0, 1]), ('rpy', [0, 0, float('inf')]), ('frame', 'a/b'),
    ('optical_frame', 'front_camera'), ('optical_frame', 'imu'),
    ('image_topic', 'camera/raw'), ('info_topic', '/camera/image_raw'),
])
def test_invalid_camera(sensor_model, tmp_path, key, value, compose):
    settings = sensor_model[1]
    settings['camera'][key] = value
    path = tmp_path / 'invalid_camera.yaml'
    path.write_text(yaml.safe_dump(settings))
    with pytest.raises(ValueError):
        compose(sensors_file=path)


def test_custom_sensor_values_reach_xacro_and_adapter(sensor_model, compose):
    settings = sensor_model[1]
    settings['body_imu'].update(update_rate=400.0, gyroscope_noise_stddev=0.02,
                                accelerometer_noise_stddev=0.03, temperature=32)
    settings['lidar_imu'].update(update_rate=200.0, gyroscope_noise_stddev=0.04,
                                 accelerometer_noise_stddev=0.05)
    settings['lidar'].update(range_min=0.1, range_max=12.0, horizontal_samples=60,
                             vertical_samples=24)
    xml, _, _, parameters = compose(sensors=settings)
    robot = ET.fromstring(xml)
    for name in ('body_imu', 'lidar_imu'):
        sensor = robot.find(f"gazebo/sensor[@name='{name}']")
        assert float(sensor.find('update_rate').text) == settings[name]['update_rate']
        for kind, key in (('angular_velocity', 'gyroscope_noise_stddev'),
                           ('linear_acceleration', 'accelerometer_noise_stddev')):
            assert parameters[f'{name}_{key}'] == settings[name][key]
            deviations = sensor.findall(f'imu/{kind}/*/noise/stddev')
            assert len(deviations) == 3
            assert all(float(v.text) == settings[name][key] for v in deviations)
    assert parameters['temperature'] == 32
    lidar = robot.find("gazebo/sensor[@name='lidar']/lidar")
    for key, field in (('range_min', 'min'), ('range_max', 'max')):
        assert parameters[key] == float(lidar.find(f'range/{field}').text) == settings['lidar'][key]
    for axis in ('horizontal', 'vertical'):
        assert int(lidar.find(f'scan/{axis}/samples').text) == settings['lidar'][f'{axis}_samples']


@pytest.mark.parametrize('frame_key', ['frame', 'optical_frame'])
def test_camera_frame_collision(sensor_model, frame_key, sensor_description):
    settings, robot = sensor_model[1], sensor_model[2]
    settings['camera'][frame_key] = 'base'
    with pytest.raises(ValueError, match='collides'):
        sensor_description(settings, 'collision')


def test_all_sensors_disabled(sensor_model, compose):
    settings = sensor_model[1]
    for sensor in settings.values():
        sensor['enabled'] = False
    xml, _, bridges, parameters = compose(sensors=settings)
    robot = ET.fromstring(xml)
    assert robot.findall('gazebo/sensor') == []
    assert robot.findall('gazebo/visual/visibility_flags') == []
    assert robot.find("link[@name='utlidar_imu']") is None
    assert robot.find("link[@name='front_camera_optical']") is None
    assert [entry['ros_topic_name'] for entry in bridges] == ['/ground_truth/poses']
    assert all(not parameters[f'{name}_enabled'] for name in
               ('body_imu', 'lidar_imu', 'lidar', 'camera'))
