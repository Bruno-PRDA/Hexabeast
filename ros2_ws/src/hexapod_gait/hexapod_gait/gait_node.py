"""Tripod gait as a ROS 2 node.

Subscribes to /cmd_vel (geometry_msgs/Twist: linear.x forward, linear.y left,
angular.z counter-clockwise) and publishes joint positions for the
ros2_control JointGroupPositionController on /leg_controller/commands, in the
joint order of robot_config.JOINT_NAMES.

The leg_ik and gait functions are the robot's brain and will run unchanged on
the ESP32; this node is only the plumbing around them.
"""
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from . import gait, leg_ik
from . import robot_config as cfg


@dataclass
class Leg:
    name: str
    mount: tuple
    yaw: float      # radians, rest direction in the body frame
    group: int      # tripod
    neutral: tuple  # foot position when standing, body frame


def _move_toward(value, target, step):
    if abs(target - value) <= step:
        return target
    return value + math.copysign(step, target - value)


class GaitNode(Node):
    def __init__(self):
        super().__init__("hexapod_gait")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("command_topic", "/leg_controller/commands")
        self.declare_parameter("startup_time", 1.0)   # ease from the boot pose into the stand
        self.declare_parameter("blend_rate", 4.0)     # how fast the legs settle when the command stops
        rate = float(self.get_parameter("rate_hz").value)
        topic = str(self.get_parameter("command_topic").value)
        self.startup_time = float(self.get_parameter("startup_time").value)
        self.blend_rate = float(self.get_parameter("blend_rate").value)
        self.dt = 1.0 / rate

        self.center = [math.radians(c) for c in cfg.JOINT_CENTER_DEG]
        self.legs = []
        for name, mount, yaw_deg, group in cfg.LEGS:
            yaw = math.radians(yaw_deg)
            neutral = (mount[0] + math.cos(yaw) * cfg.REACH,
                       mount[1] + math.sin(yaw) * cfg.REACH,
                       mount[2] - cfg.STAND_HEIGHT)
            self.legs.append(Leg(name, tuple(mount), yaw, group, neutral))

        self.cmd_vel = (0.0, 0.0)
        self.cmd_yaw = 0.0
        self.phase = 0.0
        self.blend = 0.0
        self.startup = 0.0
        # Slew-limited servo angles (centre-relative), what the real servos would be told.
        self.servo = [0.0] * (3 * len(self.legs))

        self.pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f"driving {len(self.servo)} joints on {topic} at {rate:.0f} Hz")

    def _on_cmd_vel(self, msg: Twist):
        self.cmd_vel = (msg.linear.x, msg.linear.y)
        self.cmd_yaw = msg.angular.z

    def _tick(self):
        dt = self.dt
        self.startup = min(1.0, self.startup + dt / self.startup_time)

        moving = math.hypot(*self.cmd_vel) > 1e-3 or abs(self.cmd_yaw) > 1e-3
        self.blend = _move_toward(self.blend, 1.0 if moving else 0.0, self.blend_rate * dt)
        if self.blend > 1e-3:
            self.phase = (self.phase + dt / cfg.CYCLE_TIME) % 1.0

        slew = cfg.SERVO_SPEED * dt
        out = []
        for i, leg in enumerate(self.legs):
            ox, oy, oz = gait.foot_offset(leg.neutral, self.cmd_vel, self.cmd_yaw, self.phase, leg.group)
            bx = leg.neutral[0] + ox * self.blend
            by = leg.neutral[1] + oy * self.blend
            bz = leg.neutral[2] + oz * self.blend
            # Body frame -> leg frame: subtract the mount, undo the rest yaw.
            dx, dy, dz = bx - leg.mount[0], by - leg.mount[1], bz - leg.mount[2]
            c, s = math.cos(leg.yaw), math.sin(leg.yaw)
            angles = leg_ik.solve(c * dx + s * dy, -s * dx + c * dy, dz)
            for j in range(3):
                k = 3 * i + j
                want = (angles[j] - self.center[j]) * self.startup
                self.servo[k] = _move_toward(self.servo[k], want, slew)
                # ros2_control wants URDF joint angles, i.e. the IK's angles.
                out.append(self.servo[k] + self.center[j])

        msg = Float64MultiArray()
        msg.data = out
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
