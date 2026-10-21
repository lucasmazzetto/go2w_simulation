#!/usr/bin/env python3
"""Check the default sensor profile in a running, isolated simulation."""

import argparse
import math
import struct
import time
from collections import defaultdict

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage
from unitree_go.msg import IMUState

from test_utils import rotate, yaw


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--check-turn', action='store_true',
                        help='Drive a short turn and compare both gyros to Gazebo movement.')
    parser.add_argument('--room-half-size', type=float,
                        help='Optional axis-aligned verification room inner-wall distance [m].')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('go2w_sensor_check')
    received, samples = {}, defaultdict(list)
    statics, subscriptions = {}, []
    expected = {'body': 500.0, 'lidar_imu': 250.0, 'cloud': 10.0, 'alias': 10.0}
    failures = []
    native_count = 0
    turn_rates = defaultdict(list)
    checking_turn = False
    command_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    geometry_checked = False
    aliases = {}

    def on_imu(key, frame, msg):
        received[key] = msg
        samples[key].append(stamp(msg.header))
        if checking_turn:
            gyro = msg.angular_velocity
            turn_rates[key].append((stamp(msg.header), rotate(
                msg.orientation, [gyro.x, gyro.y, gyro.z])[2]))
        if msg.header.frame_id != frame:
            failures.append(f'{key} frame {msg.header.frame_id}')
        values = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                  msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        if not all(math.isfinite(v) for v in values):
            failures.append(f'{key} non-finite IMU')

    def on_cloud(key, msg):
        nonlocal geometry_checked
        received[key] = msg
        samples[key].append(stamp(msg.header))
        fields = {f.name: (f.offset, f.datatype, f.count) for f in msg.fields}
        assert set(fields) == {'x', 'y', 'z', 'intensity', 'ring', 'time'}
        assert all(fields[k][1:] == (7, 1) for k in fields if k != 'ring')
        assert fields['ring'][1:] == (4, 1)
        assert msg.header.frame_id == 'radar' and msg.height == 1
        assert msg.width <= 2160 and msg.row_step == len(msg.data) == msg.width*msg.point_step
        identity = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if identity in aliases:
            other_key, other = aliases.pop(identity)
            if other_key != key:
                assert msg == other, 'Cloud aliases differ'
                received['alias_match'] = True
        else:
            aliases[identity] = key, msg
        if len(aliases) > 100:
            aliases.pop(next(iter(aliases)))
        xyz = []
        for i in range(msg.width):
            offset = i*msg.point_step
            p = [struct.unpack_from('<f', msg.data, offset+fields[k][0])[0] for k in ('x', 'y', 'z')]
            assert all(math.isfinite(v) for v in p)
            assert 0.05 <= math.sqrt(sum(v*v for v in p)) <= 20.00001
            assert p[2] >= -0.001, 'Point outside sensor-local hemisphere'
            assert struct.unpack_from('<H', msg.data, offset+fields['ring'][0])[0] < 18
            assert struct.unpack_from('<f', msg.data, offset+fields['time'][0])[0] == 0
            xyz.append(p)
        if args.room_half_size and not geometry_checked and xyz and 'poses' in received:
            # Match returns against known world surfaces using independently received TF.
            transform = statics.get('radar')
            pose = received['poses'].poses[0]
            if transform:
                errors = []
                for point in xyz:
                    local = rotate(transform.rotation, point)
                    local = [local[i] + v for i, v in enumerate((transform.translation.x,
                             transform.translation.y, transform.translation.z))]
                    world = rotate(pose.orientation, local)
                    world = [world[i] + v for i, v in enumerate((pose.position.x,
                             pose.position.y, pose.position.z))]
                    errors.append(min(abs(world[2]), abs(abs(world[0])-args.room_half_size),
                                      abs(abs(world[1])-args.room_half_size)))
                assert sum(e < 0.05 for e in errors)/len(errors) > 0.95, (
                    'Cloud does not align with verification room surfaces')
                geometry_checked = True

    def on_native(msg):
        nonlocal native_count
        native_count += 1
        received['native'] = msg
        assert all(math.isfinite(v) for sequence in (msg.quaternion, msg.gyroscope,
                   msg.accelerometer, msg.rpy) for v in sequence)
        assert msg.temperature == 25

    for key, frame, topic in [('body', 'imu', '/imu/data'),
                              ('lidar_imu', 'utlidar_imu', '/utlidar/imu')]:
        subscriptions.append(node.create_subscription(
            Imu, topic, lambda m, key=key, frame=frame: on_imu(key, frame, m), 100))
    for key, topic in [('cloud', '/utlidar/cloud'), ('alias', '/pointcloud')]:
        subscriptions.append(node.create_subscription(
            PointCloud2, topic, lambda m, key=key: on_cloud(key, m), 10))
    subscriptions.append(node.create_subscription(IMUState, '/imu', on_native, 100))
    subscriptions.append(node.create_subscription(
        PoseArray, '/ground_truth/poses', lambda m: received.update(poses=m), 10))
    subscriptions.append(node.create_subscription(
        TFMessage, '/tf_static',
        lambda m: statics.update({t.child_frame_id: t.transform for t in m.transforms}),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    try:
        deadline = time.monotonic() + max(120, args.seconds*15)
        while True:
            assert time.monotonic() < deadline, f'Sensor check stalled: {list(received)}'
            rclpy.spin_once(node, timeout_sec=0.01)
            assert not failures, failures[:3]
            if all(len(samples[key]) > 1 and samples[key][-1]-samples[key][0] >= args.seconds
                   for key in expected) and 'native' in received and received.get('alias_match'):
                break
        for key, frequency in expected.items():
            times = samples[key]
            assert all(b > a for a, b in zip(times, times[1:])), f'{key} stamps not increasing'
            rate = (len(times)-1)/(times[-1]-times[0])
            assert abs(rate/frequency-1) <= 0.1, f'{key} rate {rate:.2f} vs {frequency}'
            print(f'PASS: {key} {rate:.2f} Hz over {times[-1]-times[0]:.2f} simulated seconds', flush=True)
        assert abs(native_count/len(samples['body']) - 1) < 0.1, 'Native IMU count differs'
        assert {'imu', 'radar', 'utlidar_imu'} <= set(statics), 'Missing sensor TF'
        authorities = node.get_publishers_info_by_topic('/tf_static')
        assert len(authorities) == 1 and authorities[0].node_name == 'go2w_state_publisher', (
            'Sensor TFs must have only robot_state_publisher as authority')
        for key in ('body', 'lidar_imu'):
            imu = received[key]
            a = imu.linear_acceleration
            # At rest the quaternion rotates specific force into world +Z.
            force = rotate(imu.orientation, [a.x, a.y, a.z])
            assert abs(force[2]-9.81) < 0.5 and math.hypot(*force[:2]) < 0.5, (
                f'{key} stationary gravity convention: {force}')
        body, native = received['body'], received['native']
        if 'poses' in received:
            q = received['poses'].poses[0].orientation
            assert abs(sum(a*b for a, b in zip(
                [q.w, q.x, q.y, q.z], [body.orientation.w, body.orientation.x,
                                       body.orientation.y, body.orientation.z]))) > 0.999, (
                'Body orientation does not match Gazebo world/spawn orientation')
        assert abs(sum(a*b for a, b in zip(native.quaternion,
            [body.orientation.w, body.orientation.x, body.orientation.y, body.orientation.z]))) > 0.999
        if args.room_half_size:
            assert geometry_checked, 'No obstacle geometry checked'
            print('PASS: cloud aligns with known room surfaces', flush=True)
        print('PASS: schema, aliases, TF, native ordering, finite IMUs and stationary gravity', flush=True)
        if args.check_turn:
            assert 'poses' in received
            start_yaw = yaw(received['poses'].poses[0])
            start_time = samples['body'][-1]
            last_publish = -math.inf
            checking_turn = True
            while samples['body'][-1] - start_time < 3.0:
                assert time.monotonic() < deadline, 'Turning check stalled'
                rclpy.spin_once(node, timeout_sec=0.01)
                if samples['body'][-1] - last_publish >= 0.05:
                    command = Twist()
                    command.angular.z = 0.4
                    command_publisher.publish(command)
                    last_publish = samples['body'][-1]
            checking_turn = False
            actual = yaw(received['poses'].poses[0])-start_yaw
            actual = math.atan2(math.sin(actual), math.cos(actual))
            assert actual > 0.15, f'No actual turn: {actual}'
            for key in ('body', 'lidar_imu'):
                data = turn_rates[key]
                integral = sum((b[0]-a[0])*(a[1]+b[1])/2 for a, b in zip(data, data[1:]))
                assert abs(integral-actual) < 0.08, f'{key} gyro integral {integral} vs Gazebo {actual}'
            print(f'PASS: both gyros agree with actual Gazebo yaw change {actual:.3f} rad', flush=True)
    finally:
        if args.check_turn:
            command_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
