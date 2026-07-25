#!/usr/bin/env python3
"""
Feature 2 - Vision Assisted Navigation.

Subscribes to the raw camera feed, runs three lightweight OpenCV detectors
per frame (red obstacle cone, docking ArUco marker, stop ArUco marker), and
turns detections into navigation-affecting actions:

  - Red cone      -> publish a Nav2 speed limit (slows the robot down).
  - Dock marker   -> publish a docking-trigger flag (consumed by Feature 3).
  - Stop marker   -> publish to the SAME /goal_queue/pause topic Feature 1
                     already listens on, pausing the active mission.
"""

import json
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Empty
from nav2_msgs.msg import SpeedLimit


class VisionAssistNode(Node):
    """Camera-driven navigation assistance: cone slow-down, ArUco docking
    and stop triggers."""

    def __init__(self):
        super().__init__('vision_assist_node')

        # --------------------------------------------------------------
        # Parameters - Topics
        # --------------------------------------------------------------
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('vision_events_topic', 'vision/events')
        self.declare_parameter('speed_limit_topic', 'speed_limit')
        self.declare_parameter('pause_topic', 'goal_queue/pause')
        self.declare_parameter('dock_trigger_topic', 'vision/dock_trigger')

        self._camera_topic = self.get_parameter('camera_topic').value
        self._vision_events_topic = self.get_parameter('vision_events_topic').value
        self._speed_limit_topic = self.get_parameter('speed_limit_topic').value
        self._pause_topic = self.get_parameter('pause_topic').value
        self._dock_trigger_topic = self.get_parameter('dock_trigger_topic').value

        # --------------------------------------------------------------
        # Parameters - Velocity Adaptation
        # --------------------------------------------------------------
        self.declare_parameter('normal_speed', 0.5)
        self.declare_parameter('slow_speed', 0.15)
        self.declare_parameter('cone_lost_timeout_sec', 1.0)

        self._normal_speed = float(self.get_parameter('normal_speed').value)
        self._slow_speed = float(self.get_parameter('slow_speed').value)
        self._cone_lost_timeout_sec = float(
            self.get_parameter('cone_lost_timeout_sec').value)

        # --------------------------------------------------------------
        # Parameters - Performance (Frame Skip/Downscale)
        # --------------------------------------------------------------
        self.declare_parameter('processing_scale', 0.5)
        self.declare_parameter('process_every_n_frames', 2)

        self._processing_scale = float(self.get_parameter('processing_scale').value)
        self._process_every_n_frames = max(
            1, int(self.get_parameter('process_every_n_frames').value))

        # --------------------------------------------------------------
        # Parameters - Red Cone Detection (HSV Thresholds)
        # --------------------------------------------------------------
        self.declare_parameter('red_hue_low1', 0)
        self.declare_parameter('red_hue_high1', 10)
        self.declare_parameter('red_hue_low2', 170)
        self.declare_parameter('red_hue_high2', 180)
        self.declare_parameter('red_sat_min', 120)
        self.declare_parameter('red_val_min', 70)
        self.declare_parameter('red_cone_min_area_px', 800)
        self.declare_parameter('red_cone_confidence_scale', 3000.0)

        self._red_hue_low1 = int(self.get_parameter('red_hue_low1').value)
        self._red_hue_high1 = int(self.get_parameter('red_hue_high1').value)
        self._red_hue_low2 = int(self.get_parameter('red_hue_low2').value)
        self._red_hue_high2 = int(self.get_parameter('red_hue_high2').value)
        self._red_sat_min = int(self.get_parameter('red_sat_min').value)
        self._red_val_min = int(self.get_parameter('red_val_min').value)
        self._red_cone_min_area_px = float(
            self.get_parameter('red_cone_min_area_px').value)
        self._red_cone_confidence_scale = float(
            self.get_parameter('red_cone_confidence_scale').value)

        # --------------------------------------------------------------
        # Parameters - ArUco Markers
        # --------------------------------------------------------------
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('docking_marker_id', 0)
        self.declare_parameter('stop_marker_id', 1)
        self.declare_parameter('dock_event_cooldown_sec', 5.0)
        self.declare_parameter('stop_event_cooldown_sec', 3.0)

        self._aruco_dictionary_name = self.get_parameter('aruco_dictionary').value
        self._docking_marker_id = int(self.get_parameter('docking_marker_id').value)
        self._stop_marker_id = int(self.get_parameter('stop_marker_id').value)
        self._dock_event_cooldown_sec = float(
            self.get_parameter('dock_event_cooldown_sec').value)
        self._stop_event_cooldown_sec = float(
            self.get_parameter('stop_event_cooldown_sec').value)

        # --------------------------------------------------------------
        # Internal State (All Guarded by self._lock)
        # --------------------------------------------------------------
        self._lock = threading.RLock()
        self._bridge = CvBridge()
        self._frame_counter = 0
        self._last_cone_seen_time = 0.0
        self._current_speed_limit = self._normal_speed
        self._last_dock_event_time = 0.0
        self._last_stop_event_time = 0.0
        self._aruco_mode, self._aruco_detector = self._build_aruco_detector(
            self._aruco_dictionary_name)

        # --------------------------------------------------------------
        # Publishers/Subscribers
        # --------------------------------------------------------------
        self._events_pub = self.create_publisher(String, self._vision_events_topic, 10)
        self._speed_limit_pub = self.create_publisher(
            SpeedLimit, self._speed_limit_topic, 10)
        self._pause_pub = self.create_publisher(Empty, self._pause_topic, 10)
        self._dock_trigger_pub = self.create_publisher(
            Bool, self._dock_trigger_topic, 10)

        self.create_subscription(
            Image, self._camera_topic, self._image_callback, qos_profile_sensor_data)

        # Dynamic Parameter Tuning: Speeds, Thresholds, and Cooldowns can all be Changed at Runtime via `ros2 param set`.
        self.add_on_set_parameters_callback(self._on_parameters_set)

        self.get_logger().info(
            f"vision_assist_node started. camera='{self._camera_topic}', "
            f"normal_speed={self._normal_speed}, slow_speed={self._slow_speed}, "
            f"docking_marker_id={self._docking_marker_id}, "
            f"stop_marker_id={self._stop_marker_id}, "
            f"aruco_dictionary='{self._aruco_dictionary_name}' "
            f"(API mode: {self._aruco_mode}).")

    # ====================================================================
    # ArUco API Compatibility Shim
    # ====================================================================
    def _build_aruco_detector(self, dictionary_name: str):
        """Build an ArUco detector, handling both the OpenCV >= 4.7
        ArucoDetector API and the older (Ubuntu 22.04 apt python3-opencv,
        typically ~4.5.x) function-based API.

        Returns a ('new', ArucoDetector) tuple on modern OpenCV, or an
        ('old', (dictionary, parameters)) tuple on older OpenCV.
        """
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError(
                "cv2.aruco is not available. Install a build of OpenCV with "
                "the contrib 'aruco' module, e.g.: "
                "pip install opencv-contrib-python --break-system-packages")

        dict_id = getattr(cv2.aruco, dictionary_name, None)
        if dict_id is None:
            raise ValueError(
                f"Unknown ArUco dictionary '{dictionary_name}'. Expected "
                f"something like 'DICT_4X4_50', 'DICT_5X5_100', etc.")

        if hasattr(cv2.aruco, 'ArucoDetector'):
            dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            return 'new', detector

        dictionary = cv2.aruco.Dictionary_get(dict_id)
        parameters = cv2.aruco.DetectorParameters_create()
        return 'old', (dictionary, parameters)

    def _detect_markers(self, gray_image: np.ndarray):
        if self._aruco_mode == 'new':
            corners, ids, _ = self._aruco_detector.detectMarkers(gray_image)
        else:
            dictionary, parameters = self._aruco_detector
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray_image, dictionary, parameters=parameters)
        return corners, ids

    # ====================================================================
    # Dynamic Parameter Tuning
    # ====================================================================
    def _on_parameters_set(self, params) -> SetParametersResult:
        # validate first, then apply, so a bad value never partially updates internal state.
        pending = {}
        for p in params:
            if p.name == 'normal_speed':
                if p.value <= 0.0:
                    return SetParametersResult(
                        success=False, reason='normal_speed must be > 0')
                pending['normal_speed'] = p.value
            elif p.name == 'slow_speed':
                if p.value <= 0.0:
                    return SetParametersResult(
                        success=False, reason='slow_speed must be > 0')
                pending['slow_speed'] = p.value
            elif p.name == 'cone_lost_timeout_sec':
                if p.value < 0.0:
                    return SetParametersResult(
                        success=False, reason='cone_lost_timeout_sec must be >= 0')
                pending['cone_lost_timeout_sec'] = p.value
            elif p.name == 'red_cone_min_area_px':
                pending['red_cone_min_area_px'] = float(p.value)
            elif p.name == 'red_cone_confidence_scale':
                pending['red_cone_confidence_scale'] = float(p.value)

        with self._lock:
            if 'normal_speed' in pending:
                self._normal_speed = float(pending['normal_speed'])
            if 'slow_speed' in pending:
                self._slow_speed = float(pending['slow_speed'])
            if 'cone_lost_timeout_sec' in pending:
                self._cone_lost_timeout_sec = float(pending['cone_lost_timeout_sec'])
            if 'red_cone_min_area_px' in pending:
                self._red_cone_min_area_px = pending['red_cone_min_area_px']
            if 'red_cone_confidence_scale' in pending:
                self._red_cone_confidence_scale = pending['red_cone_confidence_scale']

        return SetParametersResult(success=True)

    # ====================================================================
    # Main Image Callback
    # ====================================================================
    def _image_callback(self, msg: Image):
        try:
            self._frame_counter += 1
            if (self._frame_counter % self._process_every_n_frames) != 0:
                return  # Skip this frame to save CPU.

            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except CvBridgeError as exc:
                self.get_logger().error(f"cv_bridge conversion failed: {exc}")
                return

            if self._processing_scale != 1.0:
                frame = cv2.resize(
                    frame, None,
                    fx=self._processing_scale, fy=self._processing_scale,
                    interpolation=cv2.INTER_LINEAR)

            self._process_red_cone(frame)
            self._process_aruco_markers(frame)

        except Exception as exc:  # noqa: BLE001 - callback must never raise
            self.get_logger().error(f"Unhandled exception in image callback: {exc}")

    # ====================================================================
    # Red Cone Detection - Speed Limiting
    # ====================================================================
    def _process_red_cone(self, frame: np.ndarray):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower1 = np.array([self._red_hue_low1, self._red_sat_min, self._red_val_min])
        upper1 = np.array([self._red_hue_high1, 255, 255])
        lower2 = np.array([self._red_hue_low2, self._red_sat_min, self._red_val_min])
        upper2 = np.array([self._red_hue_high2, 255, 255])

        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        # clean up sensor noise before contour extraction.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        now = time.monotonic()
        largest_area = max((cv2.contourArea(c) for c in contours), default=0.0)

        with self._lock:
            if largest_area >= self._red_cone_min_area_px:
                self._last_cone_seen_time = now
                confidence = min(1.0, largest_area / self._red_cone_confidence_scale)
                self._publish_event('red_cone', confidence)
                new_limit = self._slow_speed
            elif (now - self._last_cone_seen_time) > self._cone_lost_timeout_sec:
                new_limit = self._normal_speed
            else:
                new_limit = self._current_speed_limit  # still in the timeout window

            if new_limit != self._current_speed_limit:
                self._current_speed_limit = new_limit
                self._publish_speed_limit(new_limit)

    def _publish_speed_limit(self, speed_mps: float):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = speed_mps
        self._speed_limit_pub.publish(msg)
        self.get_logger().info(f"Speed limit set to {speed_mps:.2f} m/s.")

    # ====================================================================
    # ArUco Marker Detection - Docking Trigger/Mission Pause
    # ====================================================================
    def _process_aruco_markers(self, frame: np.ndarray):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            corners, ids = self._detect_markers(gray)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"ArUco detection failed: {exc}")
            return

        if ids is None:
            return

        now = time.monotonic()
        detected_ids = ids.flatten().tolist()

        with self._lock:
            if self._docking_marker_id in detected_ids:
                if (now - self._last_dock_event_time) > self._dock_event_cooldown_sec:
                    self._last_dock_event_time = now
                    self._publish_event('dock_marker', 1.0)
                    dock_msg = Bool()
                    dock_msg.data = True
                    self._dock_trigger_pub.publish(dock_msg)
                    self.get_logger().info(
                        f"Docking marker (id={self._docking_marker_id}) detected; "
                        f"published dock trigger on '{self._dock_trigger_topic}'.")

            if self._stop_marker_id in detected_ids:
                if (now - self._last_stop_event_time) > self._stop_event_cooldown_sec:
                    self._last_stop_event_time = now
                    self._publish_event('stop_marker', 1.0)
                    self._pause_pub.publish(Empty())
                    self.get_logger().warn(
                        f"Stop marker (id={self._stop_marker_id}) detected; "
                        f"published pause on '{self._pause_topic}'.")

    # ====================================================================
    # Vision Event Publishing
    # ====================================================================
    def _publish_event(self, event_type: str, confidence: float):
        payload = {
            'event_type': event_type,
            'confidence': round(float(confidence), 3),
            'timestamp_sec': self.get_clock().now().nanoseconds / 1e9,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._events_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionAssistNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - top-level safety net
        if node is not None:
            node.get_logger().fatal(f"Unhandled exception in vision_assist_node: {exc}")
        else:
            print(f"Unhandled exception during vision_assist_node startup: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()