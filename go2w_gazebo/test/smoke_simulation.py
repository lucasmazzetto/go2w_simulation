#!/usr/bin/env python3
"""Validate a running simulation: stand, drive, turn, stop, and command expiry."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseArray, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from test_utils import yaw


def difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stand-seconds', type=float, default=30.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('go2w_simulation_check')
    received = {}
    subscriptions = []
    for message_type, topic, key in [
        (JointState, '/joint_states', 'joints'), (Odometry, '/odom', 'odom'),
        (PoseArray, '/ground_truth/poses', 'poses'), (Clock, '/clock', 'clock'),
        (TFMessage, '/tf', 'tf'),
    ]:
        subscriptions.append(node.create_subscription(
            message_type, topic, lambda msg, key=key: received.update({key: msg}), 10))
    forwarded = []
    subscriptions.append(node.create_subscription(
        TwistStamped, '/drive_controller/cmd_vel', lambda msg: forwarded.append(msg), 10))
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def simulation_time():
        stamp = received['clock'].clock
        return stamp.sec + stamp.nanosec * 1e-9

    def pose():
        assert len(received['poses'].poses) == 1, 'Expected only the Gazebo model pose'
        return received['poses'].poses[0]

    def check_standing():
        joints = received['joints']
        assert len(joints.name) == 16, 'Expected 16 movable joints'
        assert all(math.isfinite(x) for x in joints.position + joints.velocity)
        odometry = received['odom']
        assert all(math.isfinite(x) for x in (
            odometry.pose.pose.position.x, odometry.pose.pose.position.y,
            odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w,
            odometry.twist.twist.linear.x, odometry.twist.twist.angular.z)), (
                'Non-finite wheel odometry')
        positions = dict(zip(joints.name, joints.position))
        for name, actual in positions.items():
            if not name.endswith('foot_joint'):
                target = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}[name.split('_')[1]]
                assert abs(actual - target) < 0.12, f'Posture error: {name}={actual}'
        actual = pose()
        assert 0.25 < actual.position.z < 0.65, f'Robot fell or lifted: z={actual.position.z}'
        q = actual.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        assert abs(roll) < 0.25 and abs(pitch) < 0.25, f'Robot tilted: {roll}, {pitch}'

    def step(seconds, linear=None, angular=0.0):
        start = simulation_time()
        deadline = time.monotonic() + max(30.0, seconds * 10)
        last_publish = -math.inf
        while simulation_time() - start < seconds:
            assert time.monotonic() < deadline, 'Simulation clock stalled'
            rclpy.spin_once(node, timeout_sec=0.02)
            check_standing()
            if linear is not None and simulation_time() - last_publish >= 0.05:
                command = Twist()
                command.linear.x, command.angular.z = linear, angular
                publisher.publish(command)
                last_publish = simulation_time()

    def wheel_speed():
        return max(abs(v) for n, v in zip(
            received['joints'].name, received['joints'].velocity) if n.endswith('foot_joint'))

    try:
        deadline = time.monotonic() + 90
        while len(received) < 5 or publisher.get_subscription_count() == 0:
            assert time.monotonic() < deadline, (
                f'Missing topics or command adapter: {list(received)}')
            rclpy.spin_once(node, timeout_sec=0.1)
        step(args.stand_seconds)
        print(f'PASS: stable standing for {args.stand_seconds:g} simulation seconds', flush=True)
        start = pose()
        odom_start = received['odom'].pose.pose
        step(3.0, 0.2)
        end = pose()
        forward = ((end.position.x - start.position.x) * math.cos(yaw(start)) +
                   (end.position.y - start.position.y) * math.sin(yaw(start)))
        assert forward > 0.2, f'No actual forward motion: {forward}'
        odom_end = received['odom'].pose.pose
        odom_forward = ((odom_end.position.x - odom_start.position.x) * math.cos(yaw(odom_start)) +
                        (odom_end.position.y - odom_start.position.y) * math.sin(yaw(odom_start)))
        assert odom_forward > 0.2, f'Wheel odometry missed forward motion: {odom_forward}'
        print(f'PASS: forward ground-truth displacement {forward:.3f} m', flush=True)
        step(2.0, 0.0)
        assert wheel_speed() < 0.1, 'Explicit stop did not stop wheels'
        start = pose()
        odom_start = received['odom'].pose.pose
        step(3.0, -0.2)
        end = pose()
        reverse = ((end.position.x - start.position.x) * math.cos(yaw(start)) +
                   (end.position.y - start.position.y) * math.sin(yaw(start)))
        assert reverse < -0.2, f'No actual reverse motion: {reverse}'
        odom_end = received['odom'].pose.pose
        odom_reverse = ((odom_end.position.x - odom_start.position.x) * math.cos(yaw(odom_start)) +
                        (odom_end.position.y - odom_start.position.y) * math.sin(yaw(odom_start)))
        assert odom_reverse < -0.2, f'Wheel odometry missed reverse motion: {odom_reverse}'
        print(f'PASS: reverse ground-truth displacement {reverse:.3f} m', flush=True)
        step(2.0, 0.0)
        start = yaw(pose())
        odom_start_yaw = yaw(received['odom'].pose.pose)
        step(3.0, 0.0, 0.4)
        turn = difference(yaw(pose()), start)
        assert turn > 0.15, f'No actual positive yaw rotation: {turn}'
        assert difference(yaw(received['odom'].pose.pose), odom_start_yaw) > 0.15, (
            'Wheel odometry missed positive yaw rotation')
        print(f'PASS: ground-truth yaw rotation {turn:.3f} rad', flush=True)
        step(2.0)
        assert wheel_speed() < 0.1, 'Command timeout did not stop wheels'
        before = len(forwarded)
        invalid = Twist()
        invalid.linear.x = math.nan
        publisher.publish(invalid)
        step(0.5)
        assert len(forwarded) == before, 'Non-finite command was forwarded'
        print('PASS: explicit stop, command expiry, non-finite rejection, odometry and TF',
              flush=True)
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
