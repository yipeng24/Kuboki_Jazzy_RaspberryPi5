#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Deque, Optional
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException

# tf2_geometry_msgs 注册 PoseStamped 的 transform 支持（必须 import）
import tf2_geometry_msgs  # noqa: F401


def yaw_to_quat(yaw: float):
    """Return (x,y,z,w) quaternion for yaw about Z."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class WaypointNavClient(Node):
    """
    Subscribe:
      - /llm_waypoint (PoseStamped in base_link or any frame)
      - /llm_waypoints (Path) optional: sequence navigation

    Action:
      - /navigate_to_pose (nav2_msgs/action/NavigateToPose)
    """

    def __init__(self):
        super().__init__("llm_waypoint_nav_client")

        # ----------------------------
        # Parameters (你不喜欢命令行传参，这里全用参数/默认值)
        # ----------------------------
        self.declare_parameter("single_waypoint_topic", "/llm_waypoint")
        self.declare_parameter("path_topic", "/llm_waypoints")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("preferred_global_frame", "map")  # map or odom
        self.declare_parameter("fallback_global_frame", "odom")
        self.declare_parameter("tf_timeout_sec", 0.4)
        self.declare_parameter("cancel_on_new_goal", True)
        self.declare_parameter("queue_mode_for_path", True)  # Path 是否按序执行

        self.single_topic = self.get_parameter("single_waypoint_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.action_name = self.get_parameter("nav2_action_name").value
        self.global_frame = self.get_parameter("preferred_global_frame").value
        self.fallback_frame = self.get_parameter("fallback_global_frame").value
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.cancel_on_new_goal = bool(self.get_parameter("cancel_on_new_goal").value)
        self.queue_mode = bool(self.get_parameter("queue_mode_for_path").value)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, self.action_name)
        self._active_goal_handle: Optional[rclpy.action.client.ClientGoalHandle] = None

        # Waypoint queue (for Path)
        self._queue: Deque[PoseStamped] = deque()
        self._navigating = False

        # Subscriptions
        self.create_subscription(PoseStamped, self.single_topic, self._on_single_waypoint, 10)
        self.create_subscription(Path, self.path_topic, self._on_path, 10)

        self.get_logger().info(
            f"WaypointNavClient ready.\n"
            f"  single_topic: {self.single_topic}\n"
            f"  path_topic:   {self.path_topic}\n"
            f"  action:       {self.action_name}\n"
            f"  global_frame: {self.global_frame} (fallback {self.fallback_frame})"
        )

    # ---------- Callbacks ----------
    def _on_single_waypoint(self, msg: PoseStamped):
        """Receive one waypoint and navigate immediately."""
        if self.cancel_on_new_goal and self._active_goal_handle is not None:
            self.get_logger().warn("New waypoint received: cancelling current goal.")
            self._cancel_current_goal()

        self._queue.clear()
        self._navigating = False

        goal_pose = self._transform_to_global(msg)
        if goal_pose is None:
            return

        self._send_goal(goal_pose)

    def _on_path(self, msg: Path):
        """Receive a sequence of waypoints."""
        if not msg.poses:
            self.get_logger().warn("Received empty Path.")
            return

        if self.cancel_on_new_goal and self._active_goal_handle is not None:
            self.get_logger().warn("New Path received: cancelling current goal.")
            self._cancel_current_goal()

        self._queue.clear()
        for p in msg.poses:
            gp = self._transform_to_global(p)
            if gp is not None:
                self._queue.append(gp)

        if not self._queue:
            self.get_logger().error("No valid poses after TF transform.")
            return

        if self.queue_mode:
            if not self._navigating:
                self._navigate_next_in_queue()
        else:
            # 非队列模式：只取第一个
            self._send_goal(self._queue[0])

    # ---------- Core ----------
    def _transform_to_global(self, pose_in: PoseStamped) -> Optional[PoseStamped]:
        """Transform PoseStamped to global_frame (fallback to fallback_frame)."""
        if not pose_in.header.frame_id:
            pose_in.header.frame_id = "base_link"

        # 尝试 global_frame
        for target_frame in [self.global_frame, self.fallback_frame]:
            try:
                # tf_buffer.transform 会用 header.stamp；如果 stamp=0 代表 latest
                out = self.tf_buffer.transform(
                    pose_in, target_frame, timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
                )
                out.header.frame_id = target_frame
                return out
            except TransformException as ex:
                self.get_logger().warn(
                    f"TF failed {pose_in.header.frame_id} -> {target_frame}: {ex}"
                )

        self.get_logger().error(
            f"Cannot transform waypoint from {pose_in.header.frame_id} to {self.global_frame}/{self.fallback_frame}"
        )
        return None

    def _send_goal(self, goal_pose_global: PoseStamped):
        """Send NavigateToPose goal."""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not available: /navigate_to_pose")
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose_global

        self.get_logger().info(
            f"Sending goal in frame={goal.pose.header.frame_id}: "
            f"x={goal.pose.pose.position.x:.3f}, y={goal.pose.pose.position.y:.3f}"
        )

        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
            self._navigating = False
            return

        self.get_logger().info("Goal accepted.")
        self._active_goal_handle = goal_handle
        self._navigating = True

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        # fb.current_pose, fb.distance_remaining 等字段按版本不同可能略有差异
        if hasattr(fb, "distance_remaining"):
            self.get_logger().info(f"Distance remaining: {fb.distance_remaining:.3f}")

    def _on_result(self, future):
        res = future.result().result
        status = future.result().status

        self.get_logger().info(f"Nav2 finished with status={status}, result={res}")
        self._active_goal_handle = None

        if self.queue_mode and self._queue:
            # 当前点完成后，继续下一个
            self._navigate_next_in_queue()
        else:
            self._navigating = False

    def _navigate_next_in_queue(self):
        if not self._queue:
            self.get_logger().info("Waypoint queue done.")
            self._navigating = False
            return

        next_pose = self._queue.popleft()
        self._send_goal(next_pose)

    def _cancel_current_goal(self):
        if self._active_goal_handle is None:
            return
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda f: self.get_logger().info("Cancel request sent."))
        self._active_goal_handle = None
        self._navigating = False


def main():
    rclpy.init()
    node = WaypointNavClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
