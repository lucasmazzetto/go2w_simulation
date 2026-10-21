#!/usr/bin/env python3
"""Validate a stationary simulated camera, optionally against test_camera.sdf targets."""

import argparse
import math
import time
from collections import OrderedDict
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

from test_utils import camera_intrinsics, rotate, rotation


def stamp(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def close(actual, expected, label, tolerance=1e-5):
    assert len(actual) == len(expected) and all(
        math.isfinite(a) and abs(a-b) < tolerance for a, b in zip(actual, expected)), label


def check_targets(image, info, pose, mount, optical):
    targets = [(2.0, 0.4, 0.75), (2.0, -0.4, 0.75), (2.0, 0.0, 0.22)]
    sums = [[0, 0, 0] for _ in targets]
    for v in range(0, image.height, 2):
        for u in range(0, image.width, 2):
            offset = v * image.step + u * 3
            rgb = image.data[offset:offset + 3]
            channel = max(range(3), key=lambda c: rgb[c])
            if rgb[channel] > 120 and all(rgb[channel] > 1.7 * rgb[c]
                                         for c in range(3) if c != channel):
                sums[channel][0] += u
                sums[channel][1] += v
                sums[channel][2] += 1
    for color, point, (sum_u, sum_v, count) in zip(('red', 'green', 'blue'), targets, sums):
        assert count >= 5, f'{color} target missing from image'
        local = rotate(pose.orientation, [p-t for p, t in zip(point, (
            pose.position.x, pose.position.y, pose.position.z))], inverse=True)
        local = rotate(mount.rotation, [p-t for p, t in zip(local, (
            mount.translation.x, mount.translation.y, mount.translation.z))], inverse=True)
        local = rotate(optical.rotation, local, inverse=True)
        assert local[2] > 0, 'Target is behind the optical frame'
        expected = (info.k[0] * local[0] / local[2] + info.k[2],
                    info.k[4] * local[1] / local[2] + info.k[5])
        actual = (sum_u/count, sum_v/count)
        assert math.dist(actual, expected) < 5.0, (
            f'{color} target projection {actual} vs CameraInfo prediction {expected}')
    print('PASS: forward view, optical axes and target projection within 5 pixels', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--sensors-config', type=Path, default=Path(
        get_package_share_directory('go2w_gazebo')) / 'config/sensors.yaml')
    parser.add_argument('--check-targets', action='store_true',
                        help='Check the red/green/blue targets in test_camera.sdf; robot stationary.')
    args = parser.parse_args()
    assert math.isfinite(args.seconds) and args.seconds > 0
    config = yaml.safe_load(args.sensors_config.read_text()).get('camera', {'enabled': False})
    assert config['enabled'], 'Selected profile disables the camera'
    fx, fy, cx, cy = camera_intrinsics(config)
    expected_k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    expected_p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
    rclpy.init()
    node = rclpy.create_node('go2w_camera_check')
    received, statics = {}, {}
    images, infos = OrderedDict(), OrderedDict()
    image_times, info_times, matches = [], [], set()

    def on_image(msg):
        assert msg.encoding == 'rgb8' and msg.header.frame_id == config['optical_frame']
        assert (msg.width, msg.height) == (config['width'], config['height'])
        assert msg.step >= msg.width * 3 and len(msg.data) == msg.step * msg.height
        assert stamp(msg) > 0
        assert not image_times or stamp(msg) > image_times[-1], 'Image stamps not increasing'
        image_times.append(stamp(msg))
        images[stamp(msg)] = msg
        if len(images) > 8:
            images.popitem(last=False)

    def on_info(msg):
        assert msg.header.frame_id == config['optical_frame']
        assert (msg.width, msg.height) == (config['width'], config['height'])
        assert msg.distortion_model == 'plumb_bob'
        close(msg.d, [0] * 5, 'Nonzero distortion')
        close(msg.k, expected_k, 'Incorrect K')
        close(msg.r, [1, 0, 0, 0, 1, 0, 0, 0, 1], 'Incorrect R')
        close(msg.p, expected_p, 'Incorrect P')
        assert not info_times or stamp(msg) > info_times[-1], 'CameraInfo stamps not increasing'
        info_times.append(stamp(msg))
        infos[stamp(msg)] = msg
        if len(infos) > 8:
            infos.popitem(last=False)

    subscriptions = [
        node.create_subscription(Image, config['image_topic'], on_image, 10),
        node.create_subscription(CameraInfo, config['info_topic'], on_info, 10),
        node.create_subscription(TFMessage, '/tf_static', lambda msg: statics.update(
            {t.child_frame_id: t for t in msg.transforms}),
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(PoseArray, '/ground_truth/poses',
                                 lambda msg: received.update(poses=msg), 10),
    ]
    targets_checked = not args.check_targets
    try:
        deadline = time.monotonic() + max(120, args.seconds * 15)
        while True:
            assert time.monotonic() < deadline, 'Camera check timed out'
            rclpy.spin_once(node, timeout_sec=0.01)
            common = set(images) & set(infos)
            matches.update(common)
            if (not targets_checked and common and 'poses' in received
                    and config['frame'] in statics and config['optical_frame'] in statics):
                key = max(common)
                check_targets(images[key], infos[key], received['poses'].poses[0],
                              statics[config['frame']].transform,
                              statics[config['optical_frame']].transform)
                targets_checked = True
            if (len(image_times) > 1 and len(info_times) > 1
                    and min(image_times[-1]-image_times[0], info_times[-1]-info_times[0])
                    >= args.seconds * 1e9 and len(matches) >= 10 and targets_checked):
                break
        for label, times in [('image', image_times), ('CameraInfo', info_times)]:
            rate = (len(times)-1) * 1e9 / (times[-1]-times[0])
            assert abs(rate / config['update_rate'] - 1) <= 0.1, f'{label} rate: {rate:.2f}'
            print(f'PASS: {label} {rate:.2f} Hz over {args.seconds:g}+ simulated seconds', flush=True)
        assert len(matches) >= 0.9 * min(len(image_times), len(info_times)), 'Unsynchronized camera topics'
        mount, optical = statics[config['frame']], statics[config['optical_frame']]
        assert mount.header.frame_id == 'base' and optical.header.frame_id == config['frame']
        close([mount.transform.translation.x, mount.transform.translation.y,
               mount.transform.translation.z], config['xyz'], 'Incorrect mount translation')
        for basis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            close(rotate(mount.transform.rotation, basis),
                  [sum(a*b for a, b in zip(row, basis)) for row in rotation(config['rpy'])],
                  'Incorrect mount rotation')
        close(rotate(optical.transform.rotation, [0, 0, 1]), [1, 0, 0], 'Optical Z not forward')
        close(rotate(optical.transform.rotation, [1, 0, 0]), [0, -1, 0], 'Optical X not right')
        close(rotate(optical.transform.rotation, [0, 1, 0]), [0, 0, -1], 'Optical Y not down')
        authorities = node.get_publishers_info_by_topic('/tf_static')
        assert len(authorities) == 1 and authorities[0].node_name == 'go2w_state_publisher'
        for topic in (config['image_topic'], config['info_topic']):
            publishers = node.get_publishers_info_by_topic(topic)
            assert len(publishers) == 1
            assert publishers[0].qos_profile.reliability == rclpy.qos.ReliabilityPolicy.RELIABLE
            # Some RMWs do not expose a remote writer's history/depth through
            # discovery. Check it when available; bridge config fixes depth=10.
            if publishers[0].qos_profile.history != rclpy.qos.HistoryPolicy.UNKNOWN:
                assert publishers[0].qos_profile.depth == 10
        print('PASS: rgb8 layout, synchronized calibration, optical TF and reliable depth-10 topics',
              flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
