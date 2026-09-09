"""Hexapod gait: geometry, inverse kinematics and tripod gait as pure functions,
plus the ROS 2 node that wraps them. rclpy is imported only in gait_node so the
maths can be used from plain Python (tools/gen_urdf.py does)."""
