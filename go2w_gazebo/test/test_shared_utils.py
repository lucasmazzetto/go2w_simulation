"""Known-value math checks and real subprocess regressions for test helpers."""

import math
import os
import select
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from test_utils import camera_intrinsics, rotate, rotation, stop, yaw


def test_camera_intrinsics_scale_each_image_axis():
    camera = dict(fx=400, fy=600, cx=400, cy=300, reference_width=800,
                  reference_height=600, width=400, height=150)
    assert camera_intrinsics(camera) == (200, 150, 200, 75)


@pytest.mark.parametrize('rpy,expected', [
    ((0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
    ((math.pi/2, 0, 0), ((1, 0, 0), (0, 0, -1), (0, 1, 0))),
    ((0, math.pi/2, 0), ((0, 0, 1), (0, 1, 0), (-1, 0, 0))),
    ((0, 0, math.pi/2), ((0, -1, 0), (1, 0, 0), (0, 0, 1))),
    ((math.pi/2, 0, math.pi/2), ((0, 0, 1), (1, 0, 0), (0, 1, 0))),
])
def test_rotation_basis_vectors(rpy, expected):
    for actual, row in zip(rotation(rpy), expected):
        assert actual == pytest.approx(row, abs=1e-12)


@pytest.mark.parametrize('axis,vector,expected', [
    ('x', [0, 1, 0], [0, 0, 1]),
    ('y', [0, 0, 1], [1, 0, 0]),
    ('z', [1, 0, 0], [0, 1, 0]),
])
def test_quaternion_forward_and_inverse_rotation(axis, vector, expected):
    components = dict(x=0, y=0, z=0, w=math.sqrt(0.5))
    components[axis] = math.sqrt(0.5)
    quaternion = SimpleNamespace(**components)
    assert rotate(quaternion, vector) == pytest.approx(expected, abs=1e-12)
    assert rotate(quaternion, expected, inverse=True) == pytest.approx(vector, abs=1e-12)


@pytest.mark.parametrize('angle', [-math.pi, -math.pi/2, 0, math.pi/2, math.pi])
def test_yaw_from_pose(angle):
    pose = SimpleNamespace(orientation=SimpleNamespace(
        x=0, y=0, z=math.sin(angle/2), w=math.cos(angle/2)))
    assert yaw(pose) == pytest.approx(angle, abs=1e-12)


@pytest.fixture
def start_process():
    processes = []

    def start(code):
        process = subprocess.Popen(
            [sys.executable, '-u', '-c', code], start_new_session=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        processes.append(process)
        assert select.select([process.stdout], [], [], 5)[0], 'Child did not become ready'
        assert process.stdout.readline().strip() == 'ready'
        return process

    yield start
    # Independent fallback cleanup also runs when the helper under test fails.
    for process in processes:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=3)
        process.stdout.close()
        process.stderr.close()


STUBBORN_CHILD = '''
import signal
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
print('ready', flush=True)
signal.pause()
'''


def test_stop_allows_graceful_exit(start_process):
    process = start_process('''
import signal
import sys
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
print('ready', flush=True)
signal.pause()
''')
    stop(process, timeouts=(1, 0.1, 1))
    assert process.returncode == 0


def test_stop_forces_stubborn_process_without_touching_another_group(start_process):
    target = start_process(STUBBORN_CHILD)
    unrelated = start_process(STUBBORN_CHILD)
    started = time.monotonic()
    stop(target, timeouts=(0.05, 0.05, 1))
    assert time.monotonic() - started < 3
    assert target.returncode == -signal.SIGKILL
    assert unrelated.poll() is None


def test_stop_cleans_children_after_parent_has_exited(start_process):
    process = start_process(
        'import subprocess, sys\n'
        f'subprocess.Popen([sys.executable, "-u", "-c", {STUBBORN_CHILD!r}])\n')
    assert process.wait(timeout=3) == 0
    # The orphan keeps stdout open, proving that cleanup must outlive the parent.
    assert not select.select([process.stdout], [], [], 0)[0]
    started = time.monotonic()
    stop(process, timeouts=(0.05, 0.05, 1))
    assert time.monotonic() - started < 3
    assert select.select([process.stdout], [], [], 3)[0], 'Surviving child still holds stdout'
    assert os.read(process.stdout.fileno(), 1) == b''


def test_stop_tolerates_already_exited_process_and_repeated_cleanup(start_process):
    process = start_process("print('ready', flush=True)")
    assert process.wait(timeout=3) == 0
    stop(process, timeouts=(0.05, 0.05, 1))
    stop(process, timeouts=(0.05, 0.05, 1))
    assert process.returncode == 0
