#!/usr/bin/env python3
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class JointPDSimulator(Node):
    """
    Simulate one revolute joint driven by a PD controller.

    Plant:
        I * q_ddot + b * q_dot = tau

    Controller:
        tau = Kp * (q_des - q) + Kd * (qdot_des - qdot)
    """

    def __init__(self) -> None:
        super().__init__("joint_pd_simulator")

        self.declare_parameter("joint_name", "joint1")
        self.declare_parameter("kp", 25.0)
        self.declare_parameter("kd", 7.0)
        self.declare_parameter("inertia", 1.0)
        self.declare_parameter("damping", 0.8)
        self.declare_parameter("simulation_rate_hz", 200.0)
        self.declare_parameter("effort_limit", 20.0)
        self.declare_parameter("initial_position", 0.0)
        self.declare_parameter("initial_velocity", 0.0)

        self.joint_name = str(self.get_parameter("joint_name").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.inertia = float(self.get_parameter("inertia").value)
        self.damping = float(self.get_parameter("damping").value)
        simulation_rate_hz = float(self.get_parameter("simulation_rate_hz").value)
        self.effort_limit = float(self.get_parameter("effort_limit").value)

        self.q = float(self.get_parameter("initial_position").value)
        self.q_dot = float(self.get_parameter("initial_velocity").value)
        self.q_des: Optional[float] = None
        self.q_dot_des = 0.0

        self.target_sub = self.create_subscription(
            JointState, "/target_joint_state", self.target_callback, 10
        )
        self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.effort_pub = self.create_publisher(Float64, "/control_effort", 10)
        self.error_pub = self.create_publisher(Float64, "/tracking_error", 10)

        self.dt = 1.0 / simulation_rate_hz
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f"PD simulator started: Kp={self.kp:.2f}, Kd={self.kd:.2f}, "
            f"I={self.inertia:.2f}, b={self.damping:.2f}, "
            f"rate={simulation_rate_hz:.1f} Hz"
        )

    def target_callback(self, msg: JointState) -> None:
        if not msg.position:
            return

        if msg.name and self.joint_name in msg.name:
            index = msg.name.index(self.joint_name)
        else:
            index = 0

        if index >= len(msg.position):
            return

        self.q_des = float(msg.position[index])
        self.q_dot_des = (
            float(msg.velocity[index]) if index < len(msg.velocity) else 0.0
        )

    def update(self) -> None:
        if self.q_des is None:
            return

        position_error = self.q_des - self.q
        velocity_error = self.q_dot_des - self.q_dot

        tau = self.kp * position_error + self.kd * velocity_error
        tau = max(-self.effort_limit, min(self.effort_limit, tau))

        q_ddot = (tau - self.damping * self.q_dot) / self.inertia

        self.q_dot += q_ddot * self.dt
        self.q += self.q_dot * self.dt

        now = self.get_clock().now()

        state = JointState()
        state.header.stamp = now.to_msg()
        state.name = [self.joint_name]
        state.position = [self.q]
        state.velocity = [self.q_dot]
        state.effort = [tau]
        self.state_pub.publish(state)

        effort_msg = Float64()
        effort_msg.data = tau
        self.effort_pub.publish(effort_msg)

        error_msg = Float64()
        error_msg.data = position_error
        self.error_pub.publish(error_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointPDSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
