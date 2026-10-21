"""Standard-library helpers shared by tests and installed verification scripts."""

import math
import os
import signal
import time


def camera_intrinsics(camera):
    """Scale fx, fy, cx and cy from the reference camera resolution."""
    return (camera['fx'] * camera['width'] / camera['reference_width'],
            camera['fy'] * camera['height'] / camera['reference_height'],
            camera['cx'] * camera['width'] / camera['reference_width'],
            camera['cy'] * camera['height'] / camera['reference_height'])


def rotation(rpy):
    """Return the rotation matrix for URDF roll, pitch and yaw."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))


def rotate(quaternion, vector, inverse=False):
    """Rotate a vector by a unit quaternion exposing x, y, z and w attributes."""
    q, v = quaternion, vector
    x, y, z = (-q.x, -q.y, -q.z) if inverse else (q.x, q.y, q.z)
    t = (2 * (y*v[2] - z*v[1]), 2 * (z*v[0] - x*v[2]), 2 * (x*v[1] - y*v[0]))
    cross = (y*t[2] - z*t[1], z*t[0] - x*t[2], x*t[1] - y*t[0])
    return [v[i] + q.w*t[i] + cross[i] for i in range(3)]


def yaw(pose):
    """Return yaw in radians from a pose's unit quaternion."""
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stop(process, timeouts=(8, 4, 4)):
    """Stop an owned process group created with start_new_session=True.

    Escalate even when the direct child has exited but descendants survive.
    After SIGKILL, reap the direct child without waiting for orphaned zombies.
    """
    try:
        if os.getpgid(process.pid) != process.pid:
            raise ValueError('Cleanup requires an owned process started with start_new_session=True')
    except ProcessLookupError:
        pass  # The group may still contain children of the exited leader.

    for sig, timeout in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), timeouts):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.wait(timeout=timeout)
            return
        if sig == signal.SIGKILL:
            process.wait(timeout=timeout)
            return

        deadline = time.monotonic() + timeout
        while True:
            process.poll()  # Reap the leader so it cannot keep an empty group alive.
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                process.wait(timeout=max(0, deadline - time.monotonic()))
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(0.05, remaining))
