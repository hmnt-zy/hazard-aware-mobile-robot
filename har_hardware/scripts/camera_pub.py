#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('camera_index', '/dev/camera')
        self.declare_parameter(
            'calibration_file',
            '/home/madhan/Downloads/hz_calibration.npz'
        )

        camera_index = self.get_parameter(
            'camera_index'
        ).value

        calibration_file = self.get_parameter(
            'calibration_file'
        ).value

        # -----------------------------
        # Load calibration
        # -----------------------------
        try:
            calibration = np.load(calibration_file)

            self.camera_matrix = calibration['camera_matrix']
            self.dist_coeffs = calibration['dist_coeffs']

            self.get_logger().info(
                f'Loaded calibration from: {calibration_file}'
            )

            self.get_logger().info(
                f'Camera matrix:\n{self.camera_matrix}'
            )

            self.get_logger().info(
                f'Distortion coefficients:\n{self.dist_coeffs}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to load calibration: {e}'
            )
            raise

        # -----------------------------
        # Open camera
        # -----------------------------
        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera {camera_index}'
            )
            raise RuntimeError(
                f'Cannot open camera {camera_index}'
            )

        # Optional: set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.width = int(
            self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        )

        self.height = int(
            self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )

        self.get_logger().info(
            f'Camera resolution: {self.width}x{self.height}'
        )

        # -----------------------------
        # ROS publishers
        # -----------------------------
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            10
        )

        # self.bridge = CvBridge()

        # -----------------------------
        # CameraInfo setup
        # -----------------------------
        self.camera_info = CameraInfo()

        self.camera_info.header.frame_id = 'camera_link'

        self.camera_info.width = self.width
        self.camera_info.height = self.height

        # 3x3 intrinsic camera matrix
        self.camera_info.k = [
            float(self.camera_matrix[0, 0]),
            float(self.camera_matrix[0, 1]),
            float(self.camera_matrix[0, 2]),

            float(self.camera_matrix[1, 0]),
            float(self.camera_matrix[1, 1]),
            float(self.camera_matrix[1, 2]),

            float(self.camera_matrix[2, 0]),
            float(self.camera_matrix[2, 1]),
            float(self.camera_matrix[2, 2])
        ]

        # Distortion coefficients
        self.camera_info.d = [
            float(x) for x in self.dist_coeffs.flatten()
        ]

        self.camera_info.distortion_model = 'plumb_bob'

        # Rectification matrix
        self.camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        # Projection matrix
        self.camera_info.p = [
            float(self.camera_matrix[0, 0]),
            float(self.camera_matrix[0, 1]),
            float(self.camera_matrix[0, 2]),
            0.0,

            float(self.camera_matrix[1, 0]),
            float(self.camera_matrix[1, 1]),
            float(self.camera_matrix[1, 2]),
            0.0,

            float(self.camera_matrix[2, 0]),
            float(self.camera_matrix[2, 1]),
            float(self.camera_matrix[2, 2]),
            0.0
        ]

        # -----------------------------
        # Timer
        # -----------------------------
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_frame
        )

        self.get_logger().info(
            'Publishing:'
        )
        self.get_logger().info(
            '  /camera/image_raw'
        )
        self.get_logger().info(
            '  /camera/camera_info'
        )

    def publish_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning(
                'Failed to capture frame'
            )
            return

        # IMPORTANT:
        # Do NOT undistort the image here.
        #
        # image_raw should contain the original camera image.
        # CameraInfo contains the calibration parameters.

        timestamp = self.get_clock().now().to_msg()

        # -----------------------------
        # Publish Image
        # -----------------------------
        image_msg = Image()

        image_msg.header.stamp = timestamp
        image_msg.header.frame_id = 'camera_link'

        image_msg.height = frame.shape[0]
        image_msg.width = frame.shape[1]

        image_msg.encoding = 'bgr8'
        image_msg.is_bigendian = False
        image_msg.step = frame.shape[1] * 3

        image_msg.data = frame.tobytes()

        self.image_pub.publish(image_msg)


        # -----------------------------
        # Publish CameraInfo
        # -----------------------------
        self.camera_info.header.stamp = timestamp
        self.camera_info.header.frame_id = 'camera_optical_frame'

        self.camera_info_pub.publish(
            self.camera_info
        )

    def destroy_node(self):

        if self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
