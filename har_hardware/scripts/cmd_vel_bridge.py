#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class CmdVelToSerialNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial')

        # =====================================================
        # ROBOT KINEMATICS PARAMETERS
        # =====================================================
        self.declare_parameter('wheel_radius', 0.033)   # meters
        self.declare_parameter('wheel_base', 0.185)    # meters

        # Encoder ticks per wheel revolution.
        #
        # IMPORTANT:
        # Set this to the number of encoder counts you actually
        # get for ONE COMPLETE WHEEL REVOLUTION.
        #
        # If your encoder is quadrature and your Arduino counts
        # 4 edges per encoder pulse, use the resulting COUNTS/REV.
        self.declare_parameter('ticks_per_rev', 2340.0)

        self.wheel_radius = self.get_parameter(
            'wheel_radius'
        ).value

        self.wheel_base = self.get_parameter(
            'wheel_base'
        ).value

        self.ticks_per_rev = self.get_parameter(
            'ticks_per_rev'
        ).value

        # Subscribers and Publishers
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            String,
            '/serial_in',
            10
        )

        self.get_logger().info(
            f"cmd_vel_to_serial started | "
            f"wheel_radius={self.wheel_radius} m | "
            f"wheel_base={self.wheel_base} m | "
            f"ticks_per_rev={self.ticks_per_rev}"
        )

    def cmd_vel_callback(self, msg):
        # =====================================================
        # 1. Extract desired robot velocity
        # =====================================================
        v = msg.linear.x       # m/s
        omega = msg.angular.z  # rad/s

        # =====================================================
        # 2. Differential-drive wheel velocities
        # =====================================================
        v_left = v - (omega * self.wheel_base / 2.0)
        v_right = v + (omega * self.wheel_base / 2.0)

        # =====================================================
        # 3. Convert wheel linear velocity to angular velocity
        # =====================================================
        rad_s_left = v_left / self.wheel_radius
        rad_s_right = v_right / self.wheel_radius

        # =====================================================
        # 4. Convert rad/s -> encoder ticks/sec
        #
        # ticks/sec = rad/s * ticks/rev / (2*pi)
        # =====================================================
        ticks_s_left = (
            rad_s_left * self.ticks_per_rev / (2.0 * math.pi)
        )

        ticks_s_right = (
            rad_s_right * self.ticks_per_rev / (2.0 * math.pi)
        )

        # =====================================================
        # 5. Format serial command
        #
        # Example:
        # T:123.45,456.78
        # =====================================================
        serial_msg = String()

        serial_msg.data = (
            f"T:{ticks_s_left:.2f},{ticks_s_right:.2f}\n"
        )

        # Publish to /serial_in
        self.publisher.publish(serial_msg)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelToSerialNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()