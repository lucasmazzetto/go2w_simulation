#!/usr/bin/env python3
"""Bounded native telemetry checks. Default: synthetic inputs; --live: running simulation."""

import argparse
import collections
import math
import struct
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from unitree_go.msg import IMUState, LowState

from test_utils import stop

JOINTS = [f'{leg}_{part}_joint' for leg in ('FR', 'FL', 'RR', 'RL')
          for part in ('hip', 'thigh', 'calf')] + [
              f'{leg}_foot_joint' for leg in ('FR', 'FL', 'RR', 'RL')]


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--no-imu', action='store_true')
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('lowstate_check')
    full, low, source, imu_source = [], [], [], []
    clock_time = [0.0]
    subscriptions = [
        node.create_subscription(LowState, '/lowstate', full.append, 1000),
        node.create_subscription(LowState, '/lf/lowstate', low.append, 1000),
        node.create_subscription(JointState, '/joint_states', source.append, 1000),
        node.create_subscription(IMUState, '/imu', imu_source.append, 1000),
        node.create_subscription(Clock, '/clock', lambda m: clock_time.__setitem__(
            0, m.clock.sec + m.clock.nanosec * 1e-9), 1000),
    ]
    process = None
    log = tempfile.TemporaryFile(mode='w+')
    sim = 1.0
    publish_feedback = True
    joint_pub = node.create_publisher(JointState, '/joint_states', 10)
    imu_pub = node.create_publisher(IMUState, '/imu', 10)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    command = node.create_publisher(Twist, '/cmd_vel', 10)

    def pump():
        nonlocal sim
        if not args.live:
            sim = round(sim + 0.005, 9)
            clock = Clock()
            clock.clock.sec = int(sim)
            clock.clock.nanosec = round((sim - int(sim)) * 1e9)
            clock_pub.publish(clock)
            if publish_feedback:
                joint = JointState()
                joint.header.stamp = clock.clock
                joint.name = list(reversed(JOINTS))
                joint.position = [i + 0.125 for i in reversed(range(16))]
                joint.velocity = [sim * 2 + i for i in reversed(range(16))]
                joint.effort = [-i - 0.375 for i in reversed(range(16))]
                joint_pub.publish(joint)
                if not args.no_imu:
                    imu = IMUState()
                    imu.quaternion = [1.0, 0.0, 0.0, 0.0]
                    imu.accelerometer = [0.0, 0.0, 9.81]
                    imu.temperature = 25
                    imu_pub.publish(imu)
        # Drain incoming messages without a fixed number of callback assumptions.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0)
        time.sleep(0.005)

    def until(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, 'Timed out waiting for telemetry'
            if process:
                assert process.poll() is None, 'LowState node exited unexpectedly'
            pump()

    def advance(seconds):
        start = clock_time[0]
        until(lambda: clock_time[0] - start >= seconds, max(30, seconds * 15))

    def set_parameters(values, successful=True, target='lowstate_publisher'):
        client = node.create_client(SetParametersAtomically, f'/{target}/set_parameters_atomically')
        until(lambda: client.service_is_ready())
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter(name, value=value).to_parameter_msg()
                              for name, value in values.items()]
        future = client.call_async(request)
        until(future.done)
        result = future.result().result
        assert result.successful == successful, result.reason
        node.destroy_client(client)

    try:
        if not args.live:
            process = subprocess.Popen([
                'ros2', 'run', 'go2w_gazebo', 'lowstate_publisher', '--ros-args',
                '-p', 'use_sim_time:=true', '-p', 'startup_timeout:=10.0',
                '-p', f'body_imu_enabled:={str(not args.no_imu).lower()}',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        until(lambda: full and low and source and (args.no_imu or imu_source))
        initial = full[-1]
        assert initial.bms_state.soc == 100 and initial.bms_state.current == 0
        assert initial.bms_state.cell_vol.tolist() == [4200] * 8 + [0] * 7
        assert math.isclose(initial.power_v, 33.6, abs_tol=1e-4)
        assert initial.temperature_ntc1 == initial.temperature_ntc2 == 25
        assert initial.crc == 0 and not any(initial.head) and not any(initial.wireless_remote)
        for i, m in enumerate(initial.motor_state):
            assert m.temperature == (25 if i < 16 else 0)
            assert m.mode == (1 if i < 16 else 0)
        if args.no_imu:
            assert list(initial.imu_state.quaternion) == [1, 0, 0, 0]
            assert list(initial.imu_state.accelerometer) == [0, 0, 0]

        for topic in ('/lowstate', '/lf/lowstate'):
            endpoints = node.get_publishers_info_by_topic(topic)
            assert len(endpoints) == 1, topic
            qos = endpoints[0].qos_profile
            assert qos.reliability == ReliabilityPolicy.RELIABLE
            if qos.history != HistoryPolicy.UNKNOWN:
                assert qos.depth == 10

        full.clear()
        low.clear()
        source.clear()
        imu_source.clear()
        until(lambda: len(full) > 1 and (full[-1].tick - full[0].tick) * 0.001 >= args.seconds,
              max(30, args.seconds * 15))
        elapsed = (full[-1].tick - full[0].tick) * 0.001
        full_rate = (len(full) - 1) / elapsed
        low_elapsed = (low[-1].tick - low[0].tick) * 0.001
        low_rate = (len(low) - 1) / low_elapsed
        assert abs(full_rate / 200 - 1) < 0.1, full_rate
        assert abs(low_rate / 50 - 1) < 0.1, low_rate
        snapshots = collections.defaultdict(list)
        for msg in full:
            snapshots[msg.tick].append(msg)
        # Ignore edge samples where delivery began before this capture interval.
        # Compare fields, not CDR bytes: transport alignment padding is not message data.
        assert all(any(m == original for original in snapshots[m.tick]) for m in low[2:-2])
        measured = set()
        for msg in source:
            indices = {name: i for i, name in enumerate(msg.name)}
            if len(msg.effort) != len(msg.name):
                continue
            measured.add(tuple(f32(values[indices[name]]) for name in JOINTS
                               for values in (msg.position, msg.velocity, msg.effort)))
        checked = 0
        def imu_key(msg):
            return tuple(msg.quaternion) + tuple(msg.gyroscope) + tuple(msg.accelerometer) + \
                tuple(msg.rpy) + (msg.temperature,)

        imu_values = {imu_key(m) for m in imu_source}
        for msg in full[10:-10]:
            actual = tuple(v for m in msg.motor_state[:16] for v in (m.q, m.dq, m.tau_est))
            assert actual in measured, 'Native motor values do not match a source joint sample'
            for m in msg.motor_state:
                assert m.q == m.q_raw and m.dq == m.dq_raw and m.ddq == m.ddq_raw
            if not args.no_imu:
                assert imu_key(msg.imu_state) in imu_values
            checked += 1
        assert checked > 10
        print(f'PASS: {checked} native samples, {full_rate:.2f}/{low_rate:.2f} Hz, '
              'direct joint/IMU mapping, raw mirrors, LF snapshots and QoS', flush=True)

        temperatures = [40] * 16 + [0] * 4
        set_parameters({'bms_state.soc': 75, 'power_v': 31.0, 'power_a': -1.5,
                        'bms_state.current': -1500, 'temperature_ntc1': 45,
                        'motor_state.temperature': temperatures,
                        'bms_state.bq_ntc': [30, 31], 'foot_force': [1, 2, 3, 4]})
        until(lambda: full[-1].bms_state.soc == 75 and full[-1].motor_state[15].temperature == 40)
        assert full[-1].power_v == 31 and full[-1].power_a == -1.5
        assert full[-1].bms_state.current == -1500 and full[-1].temperature_ntc1 == 45
        assert list(full[-1].bms_state.cell_vol) == [4200] * 8 + [0] * 7
        assert list(full[-1].foot_force) == [1, 2, 3, 4]
        for values in ({'bms_state.soc': 101}, {'temperature_ntc1': 128},
                       {'power_v': float('nan')}, {'motor_state.temperature': [20]},
                       {'bms_state.soc': 'bad'}, {'publish_rate': 100.0},
                       {'bms_state.soc': 20, 'temperature_ntc1': 128}):
            set_parameters(values, successful=False)
        advance(0.2)
        assert full[-1].bms_state.soc == 75 and full[-1].temperature_ntc1 == 45
        print('PASS: live scalar/array updates, independent voltages, bounds, types and atomic rejection',
              flush=True)

        if args.live and not args.no_imu:
            set_parameters({'temperature': 42}, target='sensor_adapter')
            until(lambda: full[-1].imu_state.temperature == 42 and imu_source[-1].temperature == 42)
            set_parameters({'temperature': 128}, successful=False, target='sensor_adapter')
            set_parameters({'temperature': 25}, target='sensor_adapter')
            print('PASS: native IMU temperature updates reach /imu and LowState', flush=True)
        if args.live:
            start_q = [m.q for m in full[-1].motor_state[12:16]]
            start = clock_time[0]
            while clock_time[0] - start < 3:
                msg = Twist()
                msg.linear.x = 0.2
                command.publish(msg)
                pump()
            command.publish(Twist())
            assert any(abs(m.q - q) > 0.1 for m, q in zip(full[-1].motor_state[12:16], start_q))
            assert full[-1].bms_state.soc == 75 and full[-1].motor_state[0].temperature == 40
            print('PASS: motion updates native wheel feedback without changing configured battery/temperature',
                  flush=True)
        else:
            # Stale inputs must suspend both streams despite an advancing clock.
            publish_feedback = False
            advance(1.2)
            sizes = len(full), len(low)
            advance(0.2)
            assert sizes == (len(full), len(low))
            publish_feedback = True
            until(lambda: len(full) > sizes[0] and len(low) > sizes[1])
            sim = 0.1
            full.clear()
            until(lambda: full and full[-1].tick < 1000)
            # First post-reset feedback has no usable derivative history.
            assert any(abs(m.motor_state[0].ddq) < 1e-6 for m in full)
            print('PASS: stale streams suspend/recover and clock reset clears acceleration history', flush=True)

        # Leave a live simulation with its original defaults.
        set_parameters({'bms_state.soc': 100, 'power_v': 33.6, 'power_a': 0.0,
                        'bms_state.current': 0, 'temperature_ntc1': 25,
                        'motor_state.temperature': [25] * 16 + [0] * 4,
                        'bms_state.bq_ntc': [25, 25], 'foot_force': [0] * 4})
    except Exception:
        log.seek(0)
        print(log.read()[-12000:], flush=True)
        raise
    finally:
        if args.live:
            command.publish(Twist())
        if process:
            stop(process)
        log.close()
        node.destroy_node()
        rclpy.shutdown()

    if not args.live:
        result = subprocess.run([
            'ros2', 'run', 'go2w_gazebo', 'lowstate_publisher', '--ros-args',
            '-p', 'check_only:=true', '-p', 'startup_timeout:=1.0',
        ], timeout=8, capture_output=True, text=True)
        assert result.returncode != 0 and 'LowState startup timed out' in result.stderr
        print('PASS: missing native feedback fails bounded startup readiness', flush=True)


if __name__ == '__main__':
    main()
