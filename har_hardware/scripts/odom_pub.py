#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

# --- Robot Physical Parameters ---
WHEEL_SEPARATION = 0.185  # meters

# SEPARATE WHEEL RADII for calibration (Default: 65mm / 2)
# Tweak these independently to make RViz match the real world.
WHEEL_RADIUS_L = 0.033
WHEEL_RADIUS_R = 0.033

# IMPORTANT: Change this to match your motor's encoder resolution!
TICKS_PER_REV = 2340.0

# Pre-calculate distance traveled per single encoder tick for each wheel
METERS_PER_TICK_L = (2.0 * math.pi * WHEEL_RADIUS_L) / TICKS_PER_REV
METERS_PER_TICK_R = (2.0 * math.pi * WHEEL_RADIUS_R) / TICKS_PER_REV
RAD_PER_TICK = (2.0 * math.pi) / TICKS_PER_REV

# Joint names (must match URDF)
LEFT_JOINT_NAME = 'j1'
RIGHT_JOINT_NAME = 'j2'


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.sub = self.create_subscription(String, '/serial_out', self.serial_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Robot Pose State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel Joint State (rad)
        self.left_pos = 0.0
        self.right_pos = 0.0

        self.last_time = self.get_clock().now()

        self.get_logger().info("Odometry node started. Publishing to /odom, /tf and /joint_states")

    def serial_callback(self, msg):
        data = msg.data.strip()
        if not data.startswith("V:"):
            return

        try:
            parts = data[2:].split(',')
            if len(parts) != 2:
                return
            ticks_s_left = float(parts[0])
            ticks_s_right = float(parts[1])
        except ValueError:
            return

        current_time = self.get_clock().now()

        # Calculate time difference in seconds
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0.0:
            # Convert ticks/s to m/s independently for each wheel
            v_left = ticks_s_left * METERS_PER_TICK_L
            v_right = ticks_s_right * METERS_PER_TICK_R

            # Differential drive kinematics
            v = (v_right + v_left) / 2.0
            omega = (v_right - v_left) / WHEEL_SEPARATION

            # Odometry integration
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
            self.theta += omega * dt

            # Normalize theta to [-pi, pi]
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

            # Calculate quaternion from yaw (theta)
            q_x = 0.0
            q_y = 0.0
            q_z = math.sin(self.theta / 2.0)
            q_w = math.cos(self.theta / 2.0)

            timestamp = current_time.to_msg()

            # 1. Publish TF transform (odom -> base_footprint)
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q_x
            t.transform.rotation.y = q_y
            t.transform.rotation.z = q_z
            t.transform.rotation.w = q_w
            self.tf_broadcaster.sendTransform(t)

            # 2. Publish nav_msgs/Odometry
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'

            # Set the position
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation.x = q_x
            odom_msg.pose.pose.orientation.y = q_y
            odom_msg.pose.pose.orientation.z = q_z
            odom_msg.pose.pose.orientation.w = q_w

            # Set the velocity
            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = omega

            self.odom_pub.publish(odom_msg)

            # 3. Publish sensor_msgs/JointState (wheel joints)
            w_left = ticks_s_left * RAD_PER_TICK    # rad/s
            w_right = ticks_s_right * RAD_PER_TICK  # rad/s
            self.left_pos += w_left * dt
            self.right_pos += w_right * dt

            js = JointState()
            js.header.stamp = timestamp
            js.name = [LEFT_JOINT_NAME, RIGHT_JOINT_NAME]
            js.position = [self.left_pos, self.right_pos]
            js.velocity = [w_left, w_right]
            self.joint_pub.publish(js)

        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()