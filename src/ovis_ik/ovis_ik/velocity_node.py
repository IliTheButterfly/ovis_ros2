import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from roboticstoolbox import DHRobot, RevoluteDH
from sensor_msgs.msg import JointState


class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_node')

        self.twist_sub = self.create_subscription(
            Twist,
            '/mouse_twist',
            self.twist_callback,
            10,
        )
        self.joint_sub = self.create_subscription(
            JointState,
            '/robot_joint_states',
            self.joint_callback,
            10,
        )
        self.vel_pub = self.create_publisher(JointState, '/robot_joint_velocities', 10)

        # Joint naming and axis ordering are aligned with ovis_description URDF.
        # Joints: ovis_joint_1..6
        # Axes :  z, y, -y, -x, y, -x
        self.joint_names = [
            'ovis_joint_1',
            'ovis_joint_2',
            'ovis_joint_3',
            'ovis_joint_4',
            'ovis_joint_5',
            'ovis_joint_6',
        ]

        # DH model tuned with URDF-scale distances (meters).
        self.robot = DHRobot(
            [
                RevoluteDH(d=0.10001, a=0.0, alpha=np.pi / 2),
                RevoluteDH(d=0.0, a=-0.30039, alpha=np.pi),
                RevoluteDH(d=0.0, a=0.0, alpha=np.pi / 2, offset=np.pi / 2),
                RevoluteDH(d=0.33480, a=0.0, alpha=-np.pi / 2),
                RevoluteDH(d=0.0, a=0.0, alpha=np.pi / 2),
                RevoluteDH(d=0.07000, a=0.0, alpha=0.0),
            ],
            name='ovis',
        )

        self.q = np.zeros(6)
        self.latest_twist = np.zeros(6)

    def joint_callback(self, msg: JointState):
        if not msg.position:
            return

        index = {name: i for i, name in enumerate(msg.name)}
        try:
            self.q = np.array([msg.position[index[name]] for name in self.joint_names])
        except KeyError:
            # Fallback if names are not available in expected order.
            if len(msg.position) >= 6:
                self.q = np.array(msg.position[:6])

    def twist_callback(self, msg: Twist):
        self.latest_twist = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ])
        self.compute_and_publish()

    def compute_and_publish(self):
        jacobian = self.robot.jacob0(self.q)
        q_dot = np.linalg.pinv(jacobian) @ self.latest_twist

        if np.any(np.abs(q_dot) > 2.0):
            self.get_logger().warn('Joint velocity exceeded limit; command dropped.')
            return

        out_msg = JointState()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.name = self.joint_names
        out_msg.velocity = list(q_dot)
        self.vel_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
