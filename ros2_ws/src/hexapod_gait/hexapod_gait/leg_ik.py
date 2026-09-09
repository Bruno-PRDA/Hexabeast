"""Inverse kinematics for one 3-DoF leg: coxa (yaw), femur (pitch), tibia (pitch).

Pure functions, no ROS. The leg frame has its origin at the coxa pivot, +X
outward along the leg's rest direction, +Y to its left, +Z up. Joint angle zero
is a straight leg; positive femur/tibia angles lift the link.
"""
import math

from . import robot_config as cfg


def _clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def solve(x, y, z, coxa=cfg.COXA, femur=cfg.FEMUR, tibia=cfg.TIBIA):
    """Joint angles (coxa, femur, tibia) in radians that put the foot at (x, y, z).

    Targets outside the reachable annulus are clamped to the nearest reachable
    point rather than returning NaN, so a bad gait parameter gives a visibly
    wrong pose instead of a crashed controller.
    """
    coxa_angle = math.atan2(y, x)
    horiz = math.hypot(x, y) - coxa
    vert = z
    dist = math.hypot(horiz, vert)
    dist = _clamp(dist, abs(femur - tibia) + 1e-3, femur + tibia - 1e-3)
    bearing = math.atan2(vert, horiz)
    # Law of cosines; the +acos branch is the knee-up solution a hexapod uses.
    femur_angle = bearing + math.acos(_clamp((femur * femur + dist * dist - tibia * tibia) / (2.0 * femur * dist)))
    knee = math.acos(_clamp((femur * femur + tibia * tibia - dist * dist) / (2.0 * femur * tibia)))
    return (coxa_angle, femur_angle, knee - math.pi)


def foot_position(angles, coxa=cfg.COXA, femur=cfg.FEMUR, tibia=cfg.TIBIA):
    """Forward kinematics: foot position in the leg frame for the given angles."""
    c, f, t = angles
    horiz = coxa + femur * math.cos(f) + tibia * math.cos(f + t)
    vert = femur * math.sin(f) + tibia * math.sin(f + t)
    return (horiz * math.cos(c), horiz * math.sin(c), vert)


def to_servo(angles):
    """Joint angles -> servo angles relative to each servo's centre, radians."""
    return tuple(a - math.radians(c) for a, c in zip(angles, cfg.JOINT_CENTER_DEG))


def within_limits(angles):
    travel = math.radians(cfg.JOINT_TRAVEL_DEG)
    return all(abs(s) <= travel for s in to_servo(angles))
