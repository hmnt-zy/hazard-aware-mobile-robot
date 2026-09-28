#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, LaserScan, Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from ament_index_python import get_package_share_directory

import cv2
import numpy as np
import yaml
import json
import math
import os

class TemporalBoxFilter:
    """Tracks and smoothly interpolates bounding boxes across frames to eliminate jitter."""
    def __init__(self, alpha=0.35, min_hits=2, max_misses=3):
        self.alpha = alpha
        self.min_hits = min_hits
        self.max_misses = max_misses
        self.tracks = []

    @staticmethod
    def compute_iou(boxA, boxB):
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[0] + boxA[2], boxB[0] + boxB[2])
        yB = min(boxA[1] + boxA[3], boxB[1] + boxB[3])

        inter_w = max(0, xB - xA)
        inter_h = max(0, yB - yA)
        inter_area = inter_w * inter_h

        union_area = (boxA[2] * boxA[3]) + (boxB[2] * boxB[3]) - inter_area
        return inter_area / union_area if union_area > 0 else 0.0

    def update(self, detected_boxes):
        matched = set()
        active = []
        for track in self.tracks:
            best_iou = 0.0
            best_idx = -1
            for i, det in enumerate(detected_boxes):
                if i in matched:
                    continue
                iou = self.compute_iou(track["box"], det)
                if iou > best_iou:
                    best_iou = iou
                    best_idx = i
            if best_iou > 0.20 and best_idx != -1:
                det = detected_boxes[best_idx]
                track["box"] = [self.alpha * det[j] + (1.0 - self.alpha) * track["box"][j] for j in range(4)]
                track["hits"] = min(track["hits"] + 1, 15)
                track["misses"] = 0
                matched.add(best_idx)
                active.append(track)
            else:
                track["misses"] += 1
                if track["misses"] <= self.max_misses:
                    active.append(track)

        for i, det in enumerate(detected_boxes):
            if i not in matched:
                active.append({"box": [float(v) for v in det], "hits": 1, "misses": 0})

        self.tracks = active
        return [[int(round(v)) for v in t["box"]] for t in self.tracks if t["hits"] >= self.min_hits and t["misses"] == 0]


class HazardLaserScanNode(Node):
    def __init__(self):
        super().__init__('hazard_laserscan_node')



        pkg_share_dir = get_package_share_directory('har_perception')

        calib_file_dir = os.path.join(pkg_share_dir,'config','calibration-data.yaml')
        config_file_dir = os.path.join(pkg_share_dir,'config','hazard_vision_config.json')

        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('calib_yaml_path', calib_file_dir)
        self.declare_parameter('config_file', config_file_dir)
        self.declare_parameter('enable_gui', False)

        # Extrinsics & Frames
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('camera_optical_frame_id', 'camera_optical_frame')
        self.declare_parameter('camera_x_offset', 0.0)
        self.declare_parameter('camera_y_offset', 0.0)
        self.declare_parameter('camera_height', 0.17)
        self.declare_parameter('camera_pitch_deg', 15.0)

        # Floor Transition Filters
        self.declare_parameter('max_aspect_ratio', 6.0)
        self.declare_parameter('min_pixel_height', 15)

        # LaserScan output
        self.declare_parameter('scan_topic', '/hazard/scan')
        self.declare_parameter('scan_frame_id', 'base_footprint')
        self.declare_parameter('scan_angle_min_deg', -90.0)
        self.declare_parameter('scan_angle_max_deg', 90.0)
        self.declare_parameter('scan_angle_increment_deg', 1.0)
        self.declare_parameter('scan_range_min', 0.05)
        self.declare_parameter('scan_range_max', 5.0)
        self.declare_parameter('scan_min_v_ratio', 0.5)

        self.image_topic = self.get_parameter('image_topic').value
        self.calib_path = self.get_parameter('calib_yaml_path').value
        self.config_path = self.get_parameter('config_file').value
        self.enable_gui = self.get_parameter('enable_gui').value

        self.base_frame = self.get_parameter('base_frame_id').value
        self.camera_frame = self.get_parameter('camera_frame_id').value
        self.optical_frame = self.get_parameter('camera_optical_frame_id').value
        self.cam_x = float(self.get_parameter('camera_x_offset').value)
        self.cam_y = float(self.get_parameter('camera_y_offset').value)
        self.cam_h = float(self.get_parameter('camera_height').value)
        self.cam_pitch = float(self.get_parameter('camera_pitch_deg').value)

        self.max_aspect_ratio = float(self.get_parameter('max_aspect_ratio').value)
        self.min_pixel_height = int(self.get_parameter('min_pixel_height').value)
        

        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_frame = self.get_parameter('scan_frame_id').value
        self.scan_amin = math.radians(float(self.get_parameter('scan_angle_min_deg').value))
        self.scan_amax = math.radians(float(self.get_parameter('scan_angle_max_deg').value))
        self.scan_inc = math.radians(float(self.get_parameter('scan_angle_increment_deg').value))
        self.scan_rmin = float(self.get_parameter('scan_range_min').value)
        self.scan_rmax = float(self.get_parameter('scan_range_max').value)
        self.scan_n = int(round((self.scan_amax - self.scan_amin) / self.scan_inc)) + 1

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.VOLATILE)

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, qos)
        # self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.img_w, self.img_h, self.K, self.dist, self.camera_info_template = self.load_calibration(self.calib_path)
        self.cfg = self.load_hazard_config(self.config_path)

        self.scan_min_v = float(self.get_parameter('scan_min_v_ratio').value) * self.img_h  # after load_calibration

        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.K, self.dist, None, self.K, (self.img_w, self.img_h), cv2.CV_32FC1)

        sensor_canvas = np.full((self.img_h, self.img_w), 255, dtype=np.uint8)
        warped_mask = cv2.remap(sensor_canvas, self.map1, self.map2, interpolation=cv2.INTER_NEAREST)
        border_k = cv2.getStructuringElement(cv2.MORPH_RECT, (31, 31))
        self.valid_lens_mask = cv2.erode(warped_mask, border_k)

        fy, cy = self.K[1, 1], self.K[1, 2]
        theta_rad = math.radians(self.cam_pitch)
        v_horizon = int(cy - fy * math.tan(theta_rad))
        self.safe_horizon_y = max(0, min(self.img_h - 1, v_horizon + 20))
        self.roi_valid_mask = self.valid_lens_mask[self.safe_horizon_y:self.img_h, 0:self.img_w]

        self.tracker = TemporalBoxFilter(alpha=0.35, min_hits=2, max_misses=3)

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info(f"LaserScan Node Ready! Subscribed to {self.image_topic}, publishing {self.scan_topic}")

    def load_calibration(self, yaml_path):
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Calibration file '{yaml_path}' not found.")
        with open(yaml_path, "r") as f:
            calib = yaml.safe_load(f)

        w, h = int(calib["image_width"]), int(calib["image_height"])
        K = np.array(calib["camera_matrix"]["data"], dtype=np.float64).reshape((3, 3))
        dist = np.array(calib["distortion_coefficients"]["data"], dtype=np.float64)

        ci = CameraInfo()
        ci.width, ci.height = w, h
        ci.distortion_model = calib.get("distortion_model", "plumb_bob")
        ci.d = [float(x) for x in dist.flatten()]
        ci.k = [float(x) for x in K.flatten()]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        P = np.zeros((3, 4), dtype=np.float64)
        P[0:3, 0:3] = K
        ci.p = [float(x) for x in P.flatten()]

        return w, h, K, dist, ci

    def load_hazard_config(self, json_path):
        cfg = {"actual_blur_kernel": 9, "canny_min": 40, "canny_max": 120,
               "glare_cutoff": 215, "glare_dilate": 7, "actual_close_kernel": 5, "min_area": 400}
        if os.path.exists(json_path):
            try:
                with open(json_path, "r") as f:
                    data = json.load(f)
                k_blur = data.get("actual_blur_kernel", (2 * data.get("blur_slider", 4)) + 1)
                cfg["actual_blur_kernel"] = max(3, k_blur | 1)
                cfg["canny_min"] = data.get("canny_min", 40)
                cfg["canny_max"] = data.get("canny_max", 120)
                cfg["glare_cutoff"] = data.get("glare_cutoff", 215)
                cfg["glare_dilate"] = data.get("glare_dilate", 7)
                k_close = data.get("actual_close_kernel", (2 * data.get("close_slider", 2)) + 1)
                cfg["actual_close_kernel"] = max(1, k_close | 1)
                cfg["min_area"] = data.get("min_area", data.get("min_area_div100", 4) * 100)
            except Exception as e:
                pass
        return cfg

    def publish_transforms(self, stamp):
        t_base_cam = TransformStamped()
        t_base_cam.header.stamp = stamp
        t_base_cam.header.frame_id = self.base_frame
        t_base_cam.child_frame_id = self.camera_frame
        t_base_cam.transform.translation.x = self.cam_x
        t_base_cam.transform.translation.y = self.cam_y
        t_base_cam.transform.translation.z = self.cam_h

        pitch_rad = math.radians(self.cam_pitch)
        t_base_cam.transform.rotation.y = math.sin(pitch_rad / 2.0)
        t_base_cam.transform.rotation.w = math.cos(pitch_rad / 2.0)

        t_cam_opt = TransformStamped()
        t_cam_opt.header.stamp = stamp
        t_cam_opt.header.frame_id = self.camera_frame
        t_cam_opt.child_frame_id = self.optical_frame
        t_cam_opt.transform.rotation.x = -0.5
        t_cam_opt.transform.rotation.y = 0.5
        t_cam_opt.transform.rotation.z = -0.5
        t_cam_opt.transform.rotation.w = 0.5

        self.tf_broadcaster.sendTransform([t_base_cam, t_cam_opt])

    def pixel_to_metric_ground(self, u, v):
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        theta = math.radians(self.cam_pitch)

        xn = (u - cx) / fx
        yn = (v - cy) / fy
        denominator = yn * math.cos(theta) + math.sin(theta)
        if denominator <= 0.01:
            return None, None

        t = self.cam_h / denominator
        forward_z = t * math.cos(theta) - (t * yn) * math.sin(theta)
        lateral_x = t * xn
        return lateral_x, forward_z

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image from {self.image_topic}: {e}')

    def add_obstacle_to_scan(self, ranges, map_x, map_y, metric_w):
        """Fill scan bins covered by a frontal obstacle face (line at x = map_x, width metric_w)."""
        half = metric_w / 2.0
        a1 = math.atan2(map_y - half, map_x)
        a2 = math.atan2(map_y + half, map_x)
        a_lo, a_hi = min(a1, a2), max(a1, a2)

        i_lo = int(math.ceil((a_lo - self.scan_amin) / self.scan_inc))
        i_hi = int(math.floor((a_hi - self.scan_amin) / self.scan_inc))

        if i_lo > i_hi:
            # Narrower than one bin: mark the single nearest bin at the center
            i = int(round((math.atan2(map_y, map_x) - self.scan_amin) / self.scan_inc))
            if 0 <= i < self.scan_n:
                r = math.hypot(map_x, map_y)
                if self.scan_rmin <= r <= self.scan_rmax:
                    ranges[i] = min(ranges[i], r)
            return

        for i in range(max(i_lo, 0), min(i_hi, self.scan_n - 1) + 1):
            a = self.scan_amin + i * self.scan_inc
            c = math.cos(a)
            if c < 1e-3:
                continue
            r = map_x / c
            if self.scan_rmin <= r <= self.scan_rmax:
                ranges[i] = min(ranges[i], r)

    def process_frame(self):
        if self.latest_frame is None:
            return

        raw_frame = self.latest_frame
        if raw_frame.shape[1] != self.img_w or raw_frame.shape[0] != self.img_h:
            self.get_logger().warn(
                f'Incoming image is {raw_frame.shape[1]}x{raw_frame.shape[0]}, '
                f'but calibration expects {self.img_w}x{self.img_h}. '
                'Resizing image; use matching calibration dimensions for best results.',
                throttle_duration_sec=5.0
            )
            raw_frame = cv2.resize(raw_frame, (self.img_w, self.img_h))

        now = self.get_clock().now().to_msg()
        self.camera_info_template.header.stamp = now
        self.camera_info_template.header.frame_id = self.optical_frame
        #self.camera_info_pub.publish(self.camera_info_template)
        # self.publish_transforms(now)

        frame = cv2.remap(raw_frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        roi = frame[self.safe_horizon_y:self.img_h, 0:self.img_w]
        if roi.size == 0: return

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred_roi = cv2.medianBlur(gray_roi, self.cfg["actual_blur_kernel"])

        edges = cv2.Canny(blurred_roi, self.cfg["canny_min"], self.cfg["canny_max"])
        edges[:5, :] = 0
        edges_cleaned = cv2.bitwise_and(edges, edges, mask=self.roi_valid_mask)

        _, glare_mask = cv2.threshold(gray_roi, self.cfg["glare_cutoff"], 255, cv2.THRESH_BINARY)
        if self.cfg["glare_dilate"] > 0:
            glare_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * self.cfg["glare_dilate"] + 1, 2 * self.cfg["glare_dilate"] + 1))
            glare_mask = cv2.dilate(glare_mask, glare_k)
        edges_cleaned[glare_mask > 0] = 0

        close_k = cv2.getStructuringElement(cv2.MORPH_RECT, (self.cfg["actual_close_kernel"], self.cfg["actual_close_kernel"]))
        closed = cv2.morphologyEx(edges_cleaned, cv2.MORPH_CLOSE, close_k)
        dilated = cv2.dilate(closed, close_k, iterations=1)

        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        raw_boxes, confidences = [], []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= self.cfg["min_area"]:
                bx, by, bw, bh = cv2.boundingRect(cnt)

                # --- FLOOR TRANSITION FILTER ---
                if bh < self.min_pixel_height or (bw / float(bh)) > self.max_aspect_ratio:
                    continue

                raw_boxes.append([int(bx), int(by), int(bw), int(bh)])
                confidences.append(float(area))

        nms_boxes = []
        if len(raw_boxes) > 0:
            indices = cv2.dnn.NMSBoxes(raw_boxes, confidences, float(self.cfg["min_area"]), 0.35)
            if len(indices) > 0:
                for idx in np.array(indices).flatten():
                    nms_boxes.append(raw_boxes[idx])

        stabilized_boxes = self.tracker.update(nms_boxes)

        # LaserScan ranges (inf = no obstacle)
        ranges = [float('inf')] * self.scan_n

        for (bx, by, bw, bh) in stabilized_boxes:
            full_frame_x = bx
            full_frame_y = by + self.safe_horizon_y
            contact_u = full_frame_x + (bw // 2)
            contact_v = full_frame_y + bh

            # Only publish obstacles whose ground contact is below half image height
            if contact_v < self.img_h / 2:
                continue

            lat_x, fwd_z = self.pixel_to_metric_ground(contact_u, contact_v)

            if lat_x is not None and fwd_z is not None:
                # Estimate width of the object on the ground
                metric_w = max(0.08, (bw * fwd_z) / self.K[0, 0])

                # ROS REP 103 Convention (base_link frame: x forward, y left, z up)
                map_x, map_y = fwd_z, -lat_x

                self.add_obstacle_to_scan(ranges, map_x, map_y, metric_w)

                if self.enable_gui:
                    cv2.rectangle(frame, (full_frame_x, full_frame_y), (full_frame_x + bw, full_frame_y + bh), (0, 0, 255), 2)
                    cv2.putText(frame, f"X:{map_x:.2f}m Y:{map_y:.2f}m",
                                (full_frame_x, max(20, full_frame_y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

        # Publish the LaserScan
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = self.scan_frame
        scan.angle_min = float(self.scan_amin)
        scan.angle_max = float(self.scan_amin + (self.scan_n - 1) * self.scan_inc)
        scan.angle_increment = float(self.scan_inc)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 30.0
        scan.range_min = float(self.scan_rmin)
        scan.range_max = float(self.scan_rmax)
        scan.ranges = ranges
        self.scan_pub.publish(scan)

        if self.enable_gui:
            cv2.imshow("Camera Tracking", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HazardLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
