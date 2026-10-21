#!/usr/bin/env python3
"""Manual headless integration checks for camera profiles; requires an isolated ROS domain."""

import copy
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from test_utils import stop


def main():
    share = Path(get_package_share_directory('go2w_gazebo'))
    baseline = yaml.safe_load((share / 'config/sensors.yaml').read_text())
    camera_only = copy.deepcopy(baseline)
    for name in ('body_imu', 'lidar_imu', 'lidar'):
        camera_only[name]['enabled'] = False
    camera_only['camera'].update(
        width=640, height=360, update_rate=10.0, frame='configured_camera',
        optical_frame='configured_optical', image_topic='/configured_camera/image_raw',
        info_topic='/configured_camera/camera_info', xyz=[0.32715, 0.01, 0.06],
        rpy=[0.05, -0.03, 0.07])
    disabled = copy.deepcopy(baseline)
    disabled['camera']['enabled'] = False
    legacy = copy.deepcopy(baseline)
    del legacy['camera']
    scenarios = [('camera_only', camera_only, True), ('disabled', disabled, True),
                 ('legacy', legacy, True), ('all_disabled', baseline, False)]
    with tempfile.TemporaryDirectory(prefix='go2w_camera_profiles_') as directory:
        for name, profile, enable_sensors in scenarios:
            config = Path(directory) / f'{name}.yaml'
            config.write_text(yaml.safe_dump(profile))
            with (Path(directory) / f'{name}.log').open('w+') as log:
                process = subprocess.Popen([
                    'ros2', 'launch', 'go2w_gazebo', 'sim.launch.py', 'gui:=false', 'rviz:=false',
                    f'world:={share / "worlds/test_camera.sdf"}', f'sensors_config:={config}',
                    f'enable_sensors:={str(enable_sensors).lower()}',
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                rclpy.init()
                node = rclpy.create_node('camera_profile_check')
                frames = set()
                subscription = node.create_subscription(
                    TFMessage, '/tf_static', lambda msg: frames.update(
                        t.child_frame_id for t in msg.transforms),
                    QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                try:
                    deadline = time.monotonic() + 80
                    while 'cmd_vel_adapter' not in node.get_node_names() or not frames:
                        assert process.poll() is None, f'{name} launch exited'
                        assert time.monotonic() < deadline, f'{name} driving readiness timed out'
                        rclpy.spin_once(node, timeout_sec=0.1)
                    if name == 'camera_only':
                        result = subprocess.run([
                            'ros2', 'run', 'go2w_gazebo', 'smoke_camera.py', '--seconds', '10',
                            '--sensors-config', str(config), '--check-targets',
                        ], timeout=160, capture_output=True, text=True)
                        assert result.returncode == 0, result.stdout + result.stderr
                        print(result.stdout, end='', flush=True)
                        for topic in ('/imu', '/imu/data', '/utlidar/imu', '/utlidar/cloud'):
                            assert node.count_publishers(topic) == 0, topic
                    else:
                        for topic in ('/camera/image_raw', '/camera/camera_info'):
                            assert node.count_publishers(topic) == 0, topic
                        assert 'front_camera' not in frames and 'front_camera_optical' not in frames
                    print(f'PASS: {name} profile starts driving with the expected camera state',
                          flush=True)
                except Exception:
                    log.flush()
                    log.seek(0)
                    print(log.read()[-15000:], flush=True)
                    raise
                finally:
                    node.destroy_node()
                    rclpy.shutdown()
                    stop(process, timeouts=(12, 5, 5))
        # Enabled camera with no publishers must fail within the wall-time bound.
        result = subprocess.run([
            'ros2', 'run', 'go2w_gazebo', 'sensor_adapter', '--ros-args',
            '-p', 'check_only:=true', '-p', 'body_imu_enabled:=false',
            '-p', 'lidar_imu_enabled:=false', '-p', 'lidar_enabled:=false',
            '-p', 'camera_enabled:=true', '-p', 'startup_timeout:=2.0',
        ], timeout=12, capture_output=True, text=True)
        assert result.returncode != 0 and 'No valid camera image' in result.stderr
        assert 'No valid camera info' in result.stderr
        print('PASS: missing camera streams fail readiness within the configured timeout', flush=True)


if __name__ == '__main__':
    main()
