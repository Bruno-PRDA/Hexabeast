"""Tripod gait generator. Pure functions, no ROS.

Given where a foot rests when standing and how the body wants to move, returns
where that foot should be right now. Each leg is independent; the coordination
comes from the half-cycle phase offset between the two tripods.
"""
import math

from . import robot_config as cfg


def foot_offset(neutral, vel, yaw_rate, phase, group, cycle=cfg.CYCLE_TIME, lift=cfg.STEP_HEIGHT):
    """Displacement from the neutral foot position, body frame (x, y, z).

    neutral  - foot rest position relative to the body centre
    vel      - commanded body velocity (vx forward, vy left), m/s
    yaw_rate - commanded turn rate, rad/s, positive counter-clockwise
    phase    - gait cycle position, wrapping 0..1
    group    - 0 or 1, the leg's tripod
    """
    # A planted foot must move backward as fast as the body moves forward.
    # The spin term is omega x r for a rotation about +Z.
    nx, ny, _nz = neutral
    fvx = -(vel[0] - yaw_rate * ny)
    fvy = -(vel[1] + yaw_rate * nx)
    # Stance is half the cycle and the stroke is centred, so amplitude is a quarter cycle of travel.
    ax, ay = fvx * cycle * 0.25, fvy * cycle * 0.25

    local = (phase + (0.5 if group == 1 else 0.0)) % 1.0
    if local < 0.5:
        u = local / 0.5                      # stance: on the ground, constant velocity
        return (-ax + 2.0 * ax * u, -ay + 2.0 * ay * u, 0.0)
    u = (local - 0.5) / 0.5                  # swing: back to the front, lifted
    s = u * u * (3.0 - 2.0 * u)              # smoothstep, gentle lift-off and touchdown
    return (ax - 2.0 * ax * s, ay - 2.0 * ay * s, lift * math.sin(math.pi * u))


def stance_progress(phase, group):
    """0..1 through stance, or -1 during swing. Contact checks should ignore the
    first and last stretch of stance: the foot is arriving or leaving."""
    local = (phase + (0.5 if group == 1 else 0.0)) % 1.0
    return local / 0.5 if local < 0.5 else -1.0


def is_stance(phase, group):
    return stance_progress(phase, group) >= 0.0
