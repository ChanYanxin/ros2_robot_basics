#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TargetJointPublisher(Node):
    """Publish a desired single-joint trajectory as sensor_msgs/JointState."""

    def __init__(self) -> None:
        super().__init__("target_joint_publisher")

        self.declare_parameter("joint_name", "joint1")
        self.declare_parameter("trajectory_type", "sine")
        self.declare_parameter("amplitude", 1.0)
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("frequency_hz", 0.10)
        self.declare_parameter("step_value", 1.0)
        self.declare_parameter("publish_rate_hz", 100.0)

        self.joint_name = str(self.get_parameter("joint_name").value)
        self.trajectory_type = str(self.get_parameter("trajectory_type").value).lower()
        self.amplitude = float(self.get_parameter("amplitude").value)
        self.offset = float(self.get_parameter("offset").value)
        self.frequency_hz = float(self.get_parameter("frequency_hz").value)
        self.step_value = float(self.get_parameter("step_value").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.publisher = self.create_publisher(JointState, "/target_joint_state", 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_target)

        self.get_logger().info(
            f"Publishing {self.trajectory_type} target for {self.joint_name} "
            f"at {publish_rate_hz:.1f} Hz"
        )

    def publish_target(self) -> None:
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        if self.trajectory_type == "step":
            q_des = self.step_value
            qd_des = 0.0
        else:
            omega = 2.0 * math.pi * self.frequency_hz
            q_des = self.offset + self.amplitude * math.sin(omega * t)
            qd_des = self.amplitude * omega * math.cos(omega * t)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.joint_name]
        msg.position = [q_des]
        msg.velocity = [qd_des]
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
