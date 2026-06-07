from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from iii_drone_interfaces.action import CustomOperation
from iii_drone_interfaces.msg import Target


@dataclass(frozen=True)
class OperationResult:
    accepted: bool
    success: bool
    message: str = ""
    feedback_count: int = 0


class OperationsClient:
    """Client wrapper for the generic CustomOperation ROS action."""

    def __init__(self, node: Node, namespace: str = "/mission/custom_operation"):
        self._node = node
        self._namespace = namespace.rstrip("/")
        self._run_operation = ActionClient(self._node, CustomOperation, f"{self._namespace}/run_operation")
        self._active_goal = None

    def wait_for_servers(self, timeout_sec: Optional[float] = None) -> bool:
        return self._run_operation.wait_for_server(timeout_sec=timeout_sec)

    def build_goal(self, operation: str, arguments: dict[str, Any]) -> CustomOperation.Goal:
        return self.build_raw_goal(operation, json.dumps(arguments, sort_keys=True))

    def build_raw_goal(self, operation: str, arguments_json: str, request_id: str = "") -> CustomOperation.Goal:
        goal = CustomOperation.Goal()
        goal.operation = operation
        goal.arguments_json = arguments_json
        goal.request_id = request_id
        return goal

    def _send_goal(
        self,
        *,
        action_name: str,
        arguments: dict[str, Any],
        timeout_sec: Optional[float] = None,
    ) -> OperationResult:
        send_timeout_sec = timeout_sec if timeout_sec is not None else 10.0

        if not self._run_operation.wait_for_server(timeout_sec=send_timeout_sec):
            return OperationResult(False, False, f"{action_name} action server unavailable")

        feedback_count = 0

        def on_feedback(_feedback_msg: Any) -> None:
            nonlocal feedback_count
            feedback_count += 1

        send_future = self._run_operation.send_goal_async(
            self.build_goal(action_name, arguments),
            feedback_callback=on_feedback,
        )
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=send_timeout_sec)

        if not send_future.done():
            return OperationResult(False, False, f"timed out sending {action_name} goal")

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return OperationResult(False, False, f"{action_name} goal rejected")

        self._active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)
        if not result_future.done():
            self._active_goal = None
            return OperationResult(True, False, f"{action_name} result unavailable before shutdown", feedback_count)

        wrapped_result = result_future.result()
        result = wrapped_result.result
        self._active_goal = None
        success = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED and bool(getattr(result, "success", True))
        message = "" if success else str(getattr(result, "error", ""))
        return OperationResult(True, bool(success), message=message, feedback_count=feedback_count)

    def fly_to_position(
        self,
        *,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        blend_to_next: bool = False,
        timeout_sec: Optional[float] = None,
    ) -> OperationResult:
        return self._send_goal(
            action_name="fly_to_position",
            arguments={
                "frame_id": frame_id,
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw": float(yaw),
                "blend_to_next": bool(blend_to_next),
            },
            timeout_sec=timeout_sec,
        )

    def cable_aware_fly_to_position(
        self,
        *,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        timeout_sec: Optional[float] = None,
    ) -> OperationResult:
        return self._send_goal(
            action_name="cable_aware_fly_to_position",
            arguments={"frame_id": frame_id, "x": float(x), "y": float(y), "z": float(z), "yaw": float(yaw)},
            timeout_sec=timeout_sec,
        )

    def fly_to_object(self, *, target: Target | dict[str, Any], timeout_sec: Optional[float] = None) -> OperationResult:
        return self._send_goal(
            action_name="fly_to_object",
            arguments=self._target_arguments(target),
            timeout_sec=timeout_sec,
        )

    def cable_landing(self, *, target_cable_id: int, timeout_sec: Optional[float] = None) -> OperationResult:
        return self._send_goal(
            action_name="cable_landing",
            arguments={"target_cable_id": int(target_cable_id)},
            timeout_sec=timeout_sec,
        )

    def cable_takeoff(
        self,
        *,
        target_cable_id: int,
        target_cable_distance: float,
        timeout_sec: Optional[float] = None,
    ) -> OperationResult:
        return self._send_goal(
            action_name="cable_takeoff",
            arguments={"target_cable_id": int(target_cable_id), "target_cable_distance": float(target_cable_distance)},
            timeout_sec=timeout_sec,
        )

    def hover(
        self,
        *,
        duration_s: float,
        sustain_duration_s: float = 0.0,
        sustain_action: bool = False,
        timeout_sec: Optional[float] = None,
    ) -> OperationResult:
        return self._send_goal(
            action_name="hover",
            arguments={
                "duration_s": float(duration_s),
                "sustain_duration_s": float(sustain_duration_s),
                "sustain_action": bool(sustain_action),
            },
            timeout_sec=timeout_sec,
        )

    def hover_by_object(
        self,
        *,
        target: Target | dict[str, Any],
        duration_s: float,
        sustain_action: bool = False,
        timeout_sec: Optional[float] = None,
    ) -> OperationResult:
        arguments = self._target_arguments(target)
        arguments["duration_s"] = float(duration_s)
        arguments["sustain_action"] = bool(sustain_action)
        return self._send_goal(action_name="hover_by_object", arguments=arguments, timeout_sec=timeout_sec)

    def hover_on_cable(
        self,
        *,
        target_cable_id: int,
        target_z_velocity: float = 0.0,
        target_yaw_rate: float = 0.0,
        duration_s: float,
        sustain_action: bool = False,
        timeout_sec: Optional[float] = None,
    ) -> OperationResult:
        return self._send_goal(
            action_name="hover_on_cable",
            arguments={
                "target_cable_id": int(target_cable_id),
                "target_z_velocity": float(target_z_velocity),
                "target_yaw_rate": float(target_yaw_rate),
                "duration_s": float(duration_s),
                "sustain_action": bool(sustain_action),
            },
            timeout_sec=timeout_sec,
        )

    def cancel_active(self, timeout_sec: Optional[float] = None) -> bool:
        if self._active_goal is None:
            return False

        cancel_future = self._active_goal.cancel_goal_async()
        rclpy.spin_until_future_complete(self._node, cancel_future, timeout_sec=timeout_sec)

        if cancel_future.done():
            self._active_goal = None
            return True

        return False

    @staticmethod
    def _target_arguments(target: Target | dict[str, Any]) -> dict[str, Any]:
        if isinstance(target, dict):
            transform = target.get("target_transform", {}) or {}
            translation = transform.get("translation", {}) or {}
            rotation = transform.get("rotation", {}) or {}
            return {
                "target_type": int(target.get("target_type", Target.TARGET_TYPE_CABLE)),
                "target_id": int(target["target_id"]),
                "reference_frame_id": target.get("reference_frame_id", "world"),
                "target_transform_translation_x": float(translation.get("x", 0.0)),
                "target_transform_translation_y": float(translation.get("y", 0.0)),
                "target_transform_translation_z": float(translation.get("z", 0.0)),
                "target_transform_rotation_x": float(rotation.get("x", 0.0)),
                "target_transform_rotation_y": float(rotation.get("y", 0.0)),
                "target_transform_rotation_z": float(rotation.get("z", 0.0)),
                "target_transform_rotation_w": float(rotation.get("w", 1.0)),
            }
        return {
            "target_type": int(target.target_type),
            "target_id": int(target.target_id),
            "reference_frame_id": target.reference_frame_id,
            "target_transform_translation_x": float(target.target_transform.translation.x),
            "target_transform_translation_y": float(target.target_transform.translation.y),
            "target_transform_translation_z": float(target.target_transform.translation.z),
            "target_transform_rotation_x": float(target.target_transform.rotation.x),
            "target_transform_rotation_y": float(target.target_transform.rotation.y),
            "target_transform_rotation_z": float(target.target_transform.rotation.z),
            "target_transform_rotation_w": float(target.target_transform.rotation.w),
        }
