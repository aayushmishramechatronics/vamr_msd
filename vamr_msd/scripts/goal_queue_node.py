#!/usr/bin/env python3

import json
import math
import os
import threading
from collections import deque
from dataclasses import dataclass, field
from functools import partial
from typing import Optional

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Empty, String


@dataclass
class GoalItem:
    """a single queued navigation goal plus its bookkeeping state."""

    goal_id: str
    pose: PoseStamped
    source: str
    retries_left: int
    next_attempt_time: Optional[Time] = field(default=None)


class GoalQueueNode(Node):
    """sequential Nav2 goal queue manager with pause/resume/cancel support."""

    # internal mission state machine, defined as class attributes (not
    # module-level globals) per the "no global variables" requirement.
    STATE_IDLE = 'IDLE'
    STATE_RUNNING = 'RUNNING'
    STATE_PAUSED = 'PAUSED'
    STATE_CANCELLED = 'CANCELLED'
    STATE_COMPLETED = 'COMPLETED'

    def __init__(self):
        super().__init__('goal_queue_node')

        # ------------------------------------------------------------------
        # parameters - every topic name, timing value, and behavioural knob
        # is a parameter so nothing here is hardcoded, per the architecture
        # requirements.
        # ------------------------------------------------------------------
        self.declare_parameter('action_server_name', 'navigate_to_pose')
        self.declare_parameter('rviz_goal_topic', 'goal_pose')
        self.declare_parameter('add_goal_topic', 'goal_queue/add')
        self.declare_parameter('pause_topic', 'goal_queue/pause')
        self.declare_parameter('resume_topic', 'goal_queue/resume')
        self.declare_parameter('cancel_topic', 'goal_queue/cancel')
        self.declare_parameter('status_topic', 'goal_queue/status')
        self.declare_parameter('current_goal_topic', 'current_goal')
        self.declare_parameter('mission_complete_topic', 'mission_complete')
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_delay_sec', 2.0)
        self.declare_parameter('status_publish_rate_hz', 1.0)
        self.declare_parameter('control_loop_rate_hz', 2.0)
        self.declare_parameter('default_frame_id', 'map')
        self.declare_parameter('mission_file', '')

        self._action_server_name = self.get_parameter('action_server_name').value
        self._rviz_goal_topic = self.get_parameter('rviz_goal_topic').value
        self._add_goal_topic = self.get_parameter('add_goal_topic').value
        self._pause_topic = self.get_parameter('pause_topic').value
        self._resume_topic = self.get_parameter('resume_topic').value
        self._cancel_topic = self.get_parameter('cancel_topic').value
        self._status_topic = self.get_parameter('status_topic').value
        self._current_goal_topic = self.get_parameter('current_goal_topic').value
        self._mission_complete_topic = self.get_parameter('mission_complete_topic').value
        self._max_retries = int(self.get_parameter('max_retries').value)
        self._retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)
        self._status_publish_rate_hz = float(self.get_parameter('status_publish_rate_hz').value)
        self._control_loop_rate_hz = float(self.get_parameter('control_loop_rate_hz').value)
        self._default_frame_id = self.get_parameter('default_frame_id').value
        self._mission_file = self.get_parameter('mission_file').value

        # ------------------------------------------------------------------
        # Internal Mutable State, Guarded by self._lock Everywhere it is
        # Touched.
        # ------------------------------------------------------------------
        self._lock = threading.RLock()
        self._queue = deque()
        self._state = self.STATE_IDLE
        self._current_goal_item = None
        self._current_goal_handle = None
        self._goal_counter = 0
        self._goals_completed = 0
        self._goals_failed = 0
        self._mission_started = False

        # ------------------------------------------------------------------
        # Subscriptions.
        # ------------------------------------------------------------------
        self.create_subscription(
            PoseStamped, self._rviz_goal_topic, self._rviz_goal_callback, 10)
        self.create_subscription(
            PoseStamped, self._add_goal_topic, self._add_goal_callback, 10)
        self.create_subscription(
            Empty, self._pause_topic, self._pause_callback, 10)
        self.create_subscription(
            Empty, self._resume_topic, self._resume_callback, 10)
        self.create_subscription(
            Empty, self._cancel_topic, self._cancel_callback, 10)

        # ------------------------------------------------------------------
        # Publishers.
        # ------------------------------------------------------------------
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._current_goal_pub = self.create_publisher(
            PoseStamped, self._current_goal_topic, 10)
        self._mission_complete_pub = self.create_publisher(
            Bool, self._mission_complete_topic, 10)

        # ------------------------------------------------------------------
        # Nav2 Action Client.
        # ------------------------------------------------------------------
        self._action_client = ActionClient(
            self, NavigateToPose, self._action_server_name)

        # ------------------------------------------------------------------
        # timers: one slow timer publishing status, one faster "control
        # loop" timer that dispatches the next queued goal when idle, using
        # two timers instead of ad-hoc per-retry timers keeps the whole
        # scheduling model in one predictable place.
        # ------------------------------------------------------------------
        status_period = 1.0 / max(self._status_publish_rate_hz, 0.01)
        control_period = 1.0 / max(self._control_loop_rate_hz, 0.01)
        self.create_timer(status_period, self._publish_status_callback)
        self.create_timer(control_period, self._control_loop_callback)

        self.get_logger().info(
            f"goal_queue_node started. Listening for goals on "
            f"'{self._rviz_goal_topic}' (RViz), '{self._add_goal_topic}' (topic); "
            f"Nav2 action server = '{self._action_server_name}'; "
            f"max_retries={self._max_retries}, retry_delay={self._retry_delay_sec}s."
        )

        if self._mission_file:
            self._load_mission_file(self._mission_file)

    # ========================================================================
    # Mission file loading
    # ========================================================================
    def _load_mission_file(self, path: str):
        """load a list of goals from a YAML mission file and enqueue them.

        Expected Format:

            goals:
              - x: 1.0
                y: 2.0
                yaw: 0.0        # radians, optional, default 0.0
                z: 0.0          # optional, default 0.0
                frame_id: map   # optional, defaults to 'default_frame_id' param
        """
        try:
            if not os.path.isfile(path):
                self.get_logger().error(f"mission file not found: '{path}'")
                return

            with open(path, 'r') as f:
                data = yaml.safe_load(f)

            goals = (data or {}).get('goals', [])
            if not goals:
                self.get_logger().warn(f"Mission file '{path}' contained no goals.")
                return

            with self._lock:
                for entry in goals:
                    pose = self._pose_from_dict(entry)
                    self._enqueue_goal_locked(pose, source='mission_file')

            self.get_logger().info(
                f"loaded {len(goals)} goal(s) from mission file '{path}'.")

        except (yaml.YAMLError, KeyError, ValueError, TypeError, OSError) as exc:
            self.get_logger().error(f"failed to load mission file '{path}': {exc}")

    def _pose_from_dict(self, entry: dict) -> PoseStamped:
        """build a PoseStamped from a mission-file goal entry."""
        pose = PoseStamped()
        pose.header.frame_id = entry.get('frame_id', self._default_frame_id)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(entry['x'])
        pose.pose.position.y = float(entry['y'])
        pose.pose.position.z = float(entry.get('z', 0.0))
        yaw = float(entry.get('yaw', 0.0))
        qz, qw = self._yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    @staticmethod
    def _yaw_to_quaternion(yaw: float):
        """convert a yaw angle (radians) to a (z, w) quaternion pair.

        x and y are assumed zero, which is correct for any pose that is
        purely a rotation about the vertical axis (true for a ground robot).
        """
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    # ========================================================================
    # Subscription Callbacks
    # ========================================================================
    def _rviz_goal_callback(self, msg: PoseStamped):
        try:
            with self._lock:
                self._enqueue_goal_locked(msg, source='rviz')
        except Exception as exc:  # noqa: BLE001 - callbacks must never raise
            self.get_logger().error(f"error handling RViz goal: {exc}")

    def _add_goal_callback(self, msg: PoseStamped):
        try:
            with self._lock:
                self._enqueue_goal_locked(msg, source='topic')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"error handling /goal_queue/add message: {exc}")

    def _pause_callback(self, _msg: Empty):
        try:
            with self._lock:
                if self._state != self.STATE_RUNNING:
                    self.get_logger().info(
                        f"pause requested but mission state is "
                        f"'{self._state}'; ignoring.")
                    return
                self._state = self.STATE_PAUSED
                if self._current_goal_handle is not None:
                    self.get_logger().info(
                        'cancelling active Nav2 goal due to pause request.')
                    cancel_future = self._current_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self._cancel_done_callback)
                self.get_logger().warn('Mission paused.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"error handling pause request: {exc}")

    def _resume_callback(self, _msg: Empty):
        try:
            with self._lock:
                if self._state != self.STATE_PAUSED:
                    self.get_logger().info(
                        f"resume requested but mission state is "
                        f"'{self._state}'; ignoring.")
                    return
                self._state = self.STATE_RUNNING
                self.get_logger().warn('Mission resumed.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"error handling resume request: {exc}")

    def _cancel_callback(self, _msg: Empty):
        try:
            with self._lock:
                self._queue.clear()
                if self._current_goal_handle is not None:
                    self.get_logger().info(
                        'cancelling active Nav2 goal due to cancel request.')
                    cancel_future = self._current_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self._cancel_done_callback)
                self._current_goal_item = None
                self._current_goal_handle = None
                self._state = self.STATE_CANCELLED
                self._publish_mission_complete(False)
                self.get_logger().warn('Mission cancelled; queue cleared.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"error handling cancel request: {exc}")

    def _cancel_done_callback(self, future):
        try:
            future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"error while cancelling Nav2 goal: {exc}")

    # ========================================================================
    # Queue Management (all _locked methods assume self._lock is held)
    # ========================================================================
    def _enqueue_goal_locked(self, pose: PoseStamped, source: str):
        self._goal_counter += 1
        goal_id = f"goal_{self._goal_counter}"
        item = GoalItem(
            goal_id=goal_id,
            pose=pose,
            source=source,
            retries_left=self._max_retries,
        )
        self._queue.append(item)
        self._mission_started = True

        if self._state in (self.STATE_IDLE, self.STATE_COMPLETED, self.STATE_CANCELLED):
            self._state = self.STATE_RUNNING

        self.get_logger().info(
            f"enqueued {goal_id} from '{source}' "
            f"(queue length now {len(self._queue)}).")

    def _handle_goal_failure_locked(self, item: GoalItem):
        if item.retries_left > 0:
            item.retries_left -= 1
            item.next_attempt_time = (
                self.get_clock().now() + Duration(seconds=self._retry_delay_sec))
            self._queue.appendleft(item)
            self.get_logger().warn(
                f"goal {item.goal_id} failed; retrying in "
                f"{self._retry_delay_sec:.1f}s ({item.retries_left} retries left).")
        else:
            self._goals_failed += 1
            self.get_logger().error(
                f"goal {item.goal_id} failed permanently after exhausting "
                f"retries; skipping and continuing mission.")

    # ========================================================================
    # Control Loop: dispatches the next goal when the node is idle and able.
    # ========================================================================
    def _control_loop_callback(self):
        with self._lock:
            if self._state != self.STATE_RUNNING:
                return

            if self._current_goal_item is not None:
                return  # a goal is already in flight; wait for its result.

            if not self._queue:
                if self._mission_started:
                    self._state = self.STATE_COMPLETED
                    self.get_logger().info(
                        f"Mission complete. "
                        f"{self._goals_completed} succeeded, "
                        f"{self._goals_failed} failed/skipped.")
                    self._publish_mission_complete(True)
                return

            next_item = self._queue[0]
            now = self.get_clock().now()
            if next_item.next_attempt_time is not None and now < next_item.next_attempt_time:
                return  # Still waiting out a retry delay.

            if not self._action_client.server_is_ready():
                self.get_logger().warn(
                    f"Nav2 action server '{self._action_server_name}' not "
                    f"available yet; will keep retrying.",
                    throttle_duration_sec=5.0)
                return

            self._queue.popleft()
            self._dispatch_goal_locked(next_item)

    def _dispatch_goal_locked(self, item: GoalItem):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = item.pose
        self._current_goal_item = item

        self.get_logger().info(
            f"Dispatching {item.goal_id} (source={item.source}) to Nav2.")

        send_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=partial(self._feedback_callback, item=item),
        )
        send_future.add_done_callback(partial(self._goal_response_callback, item=item))
        self._current_goal_pub.publish(item.pose)

    # ========================================================================
    # Nav2 Action Client Callbacks
    # ========================================================================
    def _feedback_callback(self, feedback_msg, item: GoalItem):
        try:
            distance = feedback_msg.feedback.distance_remaining
            self.get_logger().info(
                f"{item.goal_id}: {distance:.2f} m remaining.",
                throttle_duration_sec=5.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error processing Nav2 feedback: {exc}")

    def _goal_response_callback(self, future, item: GoalItem):
        try:
            with self._lock:
                if self._state == self.STATE_CANCELLED:
                    # The mission was cancelled while this goal was in
                    # flight; the queue has already been cleared, so do
                    # not touch it again.
                    self.get_logger().info(
                        f"Ignoring goal-response for {item.goal_id}; "
                        f"mission was cancelled.")
                    return

                goal_handle = future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn(
                        f"Goal {item.goal_id} was rejected by the action server.")
                    self._current_goal_item = None
                    self._current_goal_handle = None
                    self._handle_goal_failure_locked(item)
                    return

                self._current_goal_handle = goal_handle

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(partial(self._result_callback, item=item))

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Exception while processing goal response for "
                f"{item.goal_id}: {exc}")
            with self._lock:
                self._current_goal_item = None
                self._current_goal_handle = None
                self._handle_goal_failure_locked(item)

    def _result_callback(self, future, item: GoalItem):
        try:
            with self._lock:
                self._current_goal_handle = None

                if self._state == self.STATE_CANCELLED:
                    self.get_logger().info(
                        f"Ignoring result for {item.goal_id}; "
                        f"mission was cancelled.")
                    return

                try:
                    result_wrapper = future.result()
                    status = result_wrapper.status
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f"Exception fetching result for {item.goal_id}: {exc}")
                    self._current_goal_item = None
                    self._handle_goal_failure_locked(item)
                    return

                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f"Goal {item.goal_id} succeeded.")
                    self._current_goal_item = None
                    self._goals_completed += 1

                elif status == GoalStatus.STATUS_CANCELED and self._state == self.STATE_PAUSED:
                    # cancelled because of a pause request, not a real
                    # failure: requeue at the front with retries untouched
                    # so it resumes exactly where it left off.
                    self.get_logger().info(
                        f"Goal {item.goal_id} cancelled for pause; "
                        f"will resume on /goal_queue/resume.")
                    item.next_attempt_time = None
                    self._queue.appendleft(item)
                    self._current_goal_item = None

                else:
                    self.get_logger().warn(
                        f"Goal {item.goal_id} ended with Nav2 status "
                        f"{status}.")
                    self._current_goal_item = None
                    self._handle_goal_failure_locked(item)

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Unhandled exception in result callback for "
                f"{item.goal_id}: {exc}")

    # ========================================================================
    # Status / Mission-Complete Publishing
    # ========================================================================
    def _publish_status_callback(self):
        try:
            with self._lock:
                status_dict = {
                    'state': self._state,
                    'queue_length': len(self._queue),
                    'current_goal_id': (
                        self._current_goal_item.goal_id
                        if self._current_goal_item else None),
                    'goals_completed': self._goals_completed,
                    'goals_failed': self._goals_failed,
                }
            msg = String()
            msg.data = json.dumps(status_dict)
            self._status_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error publishing goal_queue status: {exc}")

    def _publish_mission_complete(self, success: bool):
        msg = Bool()
        msg.data = success
        self._mission_complete_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GoalQueueNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - top-level safety net
        if node is not None:
            node.get_logger().fatal(f"Unhandled exception in goal_queue_node: {exc}")
        else:
            print(f"Unhandled exception during goal_queue_node startup: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()