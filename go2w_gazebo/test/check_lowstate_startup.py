#!/usr/bin/env python3
"""Headless launch/profile checks; owns test instances in an otherwise unused ROS domain."""

import copy
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from unitree_go.msg import LowState

from test_utils import stop


def main():
    share = Path(get_package_share_directory('go2w_gazebo'))
    sensors = yaml.safe_load((share / 'config/sensors.yaml').read_text())
    for section in sensors.values():
        section['enabled'] = False
    telemetry = yaml.safe_load((share / 'config/lowstate.yaml').read_text())
    telemetry['lowstate_publisher']['ros__parameters'].update(
        publish_rate=100.0, lf_publish_rate=20.0, **{'bms_state.soc': 88})
    with tempfile.TemporaryDirectory(prefix='go2w_lowstate_profiles_') as directory:
        directory = Path(directory)
        sensor_config = directory / 'sensors.yaml'
        sensor_config.write_text(yaml.safe_dump(sensors))
        telemetry_config = directory / 'lowstate.yaml'
        telemetry_config.write_text(yaml.safe_dump(telemetry))
        scenarios = [
            ('physics_only', ['enable_sensors:=false'], True, 100),
            ('no_body_imu', [f'sensors_config:={sensor_config}'], True, 100),
            ('telemetry_disabled', ['enable_sensors:=false', 'enable_lowstate:=false'], False, 100),
            ('custom_telemetry', ['enable_sensors:=false', f'lowstate_config:={telemetry_config}'],
             True, 88),
        ]
        for name, arguments, enabled, soc in scenarios:
            with (directory / f'{name}.log').open('w+') as log:
                process = subprocess.Popen([
                    'ros2', 'launch', 'go2w_gazebo', 'sim.launch.py',
                    'gui:=false', 'rviz:=false', *arguments,
                ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                rclpy.init()
                node = rclpy.create_node('lowstate_startup_check')
                received = []
                subscription = node.create_subscription(LowState, '/lowstate', received.append, 100)
                try:
                    deadline = time.monotonic() + 80
                    while 'cmd_vel_adapter' not in node.get_node_names():
                        assert process.poll() is None, f'{name}: launch exited'
                        assert time.monotonic() < deadline, f'{name}: driving startup timed out'
                        rclpy.spin_once(node, timeout_sec=0.1)
                    if enabled:
                        while not received:
                            assert time.monotonic() < deadline, 'Missing LowState'
                            rclpy.spin_once(node, timeout_sec=0.1)
                        msg = received[-1]
                        assert msg.bms_state.soc == soc
                        assert list(msg.imu_state.quaternion) == [1, 0, 0, 0]
                        assert list(msg.imu_state.accelerometer) == [0, 0, 0]
                        assert msg.imu_state.temperature == 25
                        assert all(m.q == m.q_raw for m in msg.motor_state)
                        assert node.count_publishers('/lf/lowstate') == 1
                        if name == 'physics_only':
                            result = subprocess.run([
                                'ros2', 'run', 'go2w_gazebo', 'check_lowstate.py',
                                '--live', '--no-imu',
                            ], timeout=160, capture_output=True, text=True)
                            assert result.returncode == 0, result.stdout + result.stderr
                            print(result.stdout, end='', flush=True)
                        if name == 'custom_telemetry':
                            client = node.create_client(GetParameters,
                                                        '/lowstate_publisher/get_parameters')
                            assert client.wait_for_service(timeout_sec=5)
                            future = client.call_async(GetParameters.Request(
                                names=['publish_rate', 'lf_publish_rate']))
                            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                            assert future.done()
                            assert [v.double_value for v in future.result().values] == [100, 20]
                    else:
                        assert node.count_publishers('/lowstate') == 0
                        assert node.count_publishers('/lf/lowstate') == 0
                        assert 'lowstate_publisher' not in node.get_node_names()
                    print(f'PASS: {name} reaches driving readiness with expected native telemetry',
                          flush=True)
                except Exception:
                    log.seek(0)
                    print(log.read()[-14000:], flush=True)
                    raise
                finally:
                    node.destroy_node()
                    rclpy.shutdown()
                    stop(process)

        invalid = copy.deepcopy(telemetry)
        invalid['lowstate_publisher']['ros__parameters']['bms_state.soc'] = 101
        telemetry_config.write_text(yaml.safe_dump(invalid))
        with (directory / 'invalid.log').open('w+') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'go2w_gazebo', 'sim.launch.py',
                'gui:=false', 'rviz:=false', 'enable_sensors:=false',
                f'lowstate_config:={telemetry_config}',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                assert process.wait(timeout=80) != 0
                log.seek(0)
                output = log.read()
                assert 'bms_state.soc: Value outside native bounds' in output
                assert 'cmd_vel_adapter-' not in output
                print('PASS: invalid startup telemetry fails before driving is enabled', flush=True)
            finally:
                stop(process)


if __name__ == '__main__':
    main()
