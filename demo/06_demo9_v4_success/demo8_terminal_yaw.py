#!/usr/bin/env python3
"""Demo8 terminal-yaw geometry and bounded acceptance checks (ROS-free)."""
import math


def wrap_rad(angle):
    if not math.isfinite(angle):
        raise ValueError('non-finite angle')
    return math.atan2(math.sin(angle), math.cos(angle))


def demo_yaw_to_world_deg(forward_world, demo_yaw_deg):
    """Demo F=0 deg; left is +90 deg; map/world ENU yaw convention."""
    fx, fy = [float(v) for v in forward_world]
    yaw = float(demo_yaw_deg)
    if not all(map(math.isfinite, (fx, fy, yaw))):
        raise ValueError('non-finite yaw transform inputs')
    if abs(math.hypot(fx, fy) - 1.0) > 0.03:
        raise ValueError('Demo forward axis is not unit length')
    if not -180 <= yaw <= 180:
        raise ValueError('yaw_deg outside [-180,180]')
    return math.degrees(wrap_rad(math.atan2(fy, fx) + math.radians(yaw)))


def yaw_error_deg(actual_yaw_rad, world_yaw_deg):
    return math.degrees(abs(wrap_rad(math.radians(world_yaw_deg) - actual_yaw_rad)))


class TerminalYawGate:
    """Requires continuous measured pose/yaw/speed tolerance for stable_s."""

    def __init__(self, goal_xyz, yaw_world_deg, xy_tol=0.15, z_tol=0.10,
                 speed_tol=0.12, yaw_tol_deg=5.0, stable_s=1.0):
        self.goal_xyz = tuple(float(v) for v in goal_xyz)
        self.yaw_world_deg = float(yaw_world_deg)
        self.xy_tol = float(xy_tol)
        self.z_tol = float(z_tol)
        self.speed_tol = float(speed_tol)
        self.yaw_tol_deg = float(yaw_tol_deg)
        self.stable_s = float(stable_s)
        self.since = None
        self.last_now = None
        if not all(math.isfinite(v) for v in (*self.goal_xyz, self.yaw_world_deg)):
            raise ValueError('non-finite terminal yaw goal')

    def update(self, now, xyz, linear_velocity, yaw_rad, setpoint_ok):
        xyz = tuple(float(v) for v in xyz)
        vel = tuple(float(v) for v in linear_velocity)
        yaw_rad = float(yaw_rad)
        now = float(now)
        if not all(math.isfinite(v) for v in (now, *xyz, *vel, yaw_rad)):
            raise ValueError('invalid odometry for yaw alignment')
        if self.last_now is not None and now < self.last_now:
            raise ValueError('non-monotonic clock')
        self.last_now = now
        dxy = math.hypot(xyz[0] - self.goal_xyz[0], xyz[1] - self.goal_xyz[1])
        dz = abs(xyz[2] - self.goal_xyz[2])
        speed = math.sqrt(sum(v*v for v in vel))
        yaw_error = yaw_error_deg(yaw_rad, self.yaw_world_deg)
        valid = (bool(setpoint_ok) and dxy < self.xy_tol and dz < self.z_tol
                 and speed < self.speed_tol and yaw_error <= self.yaw_tol_deg)
        if valid:
            if self.since is None:
                self.since = now
        else:
            self.since = None
        elapsed = 0.0 if self.since is None else now - self.since
        return elapsed >= self.stable_s, (dxy, dz, speed, yaw_error, elapsed)
