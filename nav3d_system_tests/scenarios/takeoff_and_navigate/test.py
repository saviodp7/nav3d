#!/usr/bin/env python3

import json
import math
import os
import sys
import time
import traceback
from dataclasses import asdict, dataclass
from typing import Callable, Optional

import rclpy
from action_msgs.msg import GoalStatus
from nav3d_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float


class TakeoffAndNavigateTest(Node):
    def __init__(self) -> None:
        super().__init__("takeoff_and_navigate_test")

        self.declare_parameter("report_file", "/tmp/nav3d_takeoff_and_navigate.json")
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 1.5)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("readiness_timeout_s", 90.0)
        self.declare_parameter("takeoff_timeout_s", 60.0)
        self.declare_parameter("navigation_timeout_s", 90.0)
        self.declare_parameter("minimum_takeoff_altitude", 0.8)
        self.declare_parameter("takeoff_vertical_speed_tolerance", 0.25)
        self.declare_parameter("takeoff_stability_duration_s", 1.0)
        self.declare_parameter("position_tolerance", 0.3)
        self.declare_parameter("yaw_tolerance", 0.3)

        self.report_file = self.get_parameter("report_file").value
        self.goal = PoseSample(
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
            float(self.get_parameter("goal_yaw").value),
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_odom: Optional[Odometry] = None
        self.latest_vehicle_status: Optional[VehicleStatus] = None
        self.odom_count = 0
        self.setpoint_count = 0
        self.odom_first_wall: Optional[float] = None
        self.odom_last_wall: Optional[float] = None
        self.setpoint_first_wall: Optional[float] = None
        self.setpoint_last_wall: Optional[float] = None
        self.odom_max_gap_s = 0.0
        self.setpoint_max_gap_s = 0.0
        self.failsafe_seen_during_navigation = False
        self.navigation_started = False
        self.last_feedback_distance: Optional[float] = None
        self.takeoff_diagnostics = {}

        self.create_subscription(Odometry, "/odom", self._odom_callback, sensor_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self._vehicle_status_callback, sensor_qos)
        self.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._setpoint_callback, sensor_qos)

        self.lifecycle_client = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.report = {
            "schema_version": 1,
            "scenario": "takeoff_and_navigate",
            "goal": asdict(self.goal),
            "passed": False,
            "failure": None,
            "checks": {},
            "metrics": {},
        }

    @staticmethod
    def _yaw_from_odometry(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angular_distance(a: float, b: float) -> float:
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def _pose_sample(self) -> Optional[PoseSample]:
        if self.latest_odom is None:
            return None
        position = self.latest_odom.pose.pose.position
        return PoseSample(position.x, position.y, position.z, self._yaw_from_odometry(self.latest_odom))

    def _odom_callback(self, msg: Odometry) -> None:
        now = time.monotonic()
        if self.odom_last_wall is not None:
            self.odom_max_gap_s = max(self.odom_max_gap_s, now - self.odom_last_wall)
        self.odom_first_wall = self.odom_first_wall or now
        self.odom_last_wall = now
        self.odom_count += 1
        self.latest_odom = msg

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.latest_vehicle_status = msg
        if self.navigation_started and msg.failsafe:
            self.failsafe_seen_during_navigation = True

    def _setpoint_callback(self, _: TrajectorySetpoint) -> None:
        now = time.monotonic()
        if self.setpoint_last_wall is not None:
            self.setpoint_max_gap_s = max(self.setpoint_max_gap_s, now - self.setpoint_last_wall)
        self.setpoint_first_wall = self.setpoint_first_wall or now
        self.setpoint_last_wall = now
        self.setpoint_count += 1

    def _spin_until(self, predicate: Callable[[], bool], timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError(f"Timeout while waiting for {description} ({timeout_s:.1f}s)")

    def _wait_for_lifecycle(self, timeout_s: float) -> None:
        self._spin_until(self.lifecycle_client.service_is_ready, timeout_s, "lifecycle manager service")
        response_future = self.lifecycle_client.call_async(Trigger.Request())
        self._spin_until(response_future.done, timeout_s, "lifecycle manager response")
        response = response_future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"Nav3D lifecycle manager is not active: {message}")

    def _wait_for_takeoff(self, timeout_s: float) -> None:
        minimum_altitude = float(self.get_parameter("minimum_takeoff_altitude").value)
        speed_tolerance = float(self.get_parameter("takeoff_vertical_speed_tolerance").value)
        stability_duration = float(self.get_parameter("takeoff_stability_duration_s").value)
        stable_since: Optional[float] = None
        last_diagnostic_log = 0.0

        def takeoff_is_stable() -> bool:
            nonlocal stable_since, last_diagnostic_log
            if self.latest_odom is None or self.latest_vehicle_status is None:
                stable_since = None
                return False

            status = self.latest_vehicle_status
            altitude = self.latest_odom.pose.pose.position.z
            vertical_speed = abs(self.latest_odom.twist.twist.linear.z)
            ready = (
                status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and not status.failsafe
                and altitude >= minimum_altitude
                and vertical_speed <= speed_tolerance
            )

            self.takeoff_diagnostics = {
                "altitude_m": altitude,
                "vertical_speed_mps": vertical_speed,
                "arming_state": int(status.arming_state),
                "navigation_state": int(status.nav_state),
                "failsafe": bool(status.failsafe),
                "conditions": {
                    "armed": status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                    "offboard": status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                    "not_failsafe": not status.failsafe,
                    "minimum_altitude": altitude >= minimum_altitude,
                    "vertical_speed_stable": vertical_speed <= speed_tolerance,
                },
            }

            now = time.monotonic()
            if now - last_diagnostic_log >= 5.0:
                last_diagnostic_log = now
                self.get_logger().info(
                    "Takeoff state: "
                    f"armed={self.takeoff_diagnostics['conditions']['armed']}, "
                    f"offboard={self.takeoff_diagnostics['conditions']['offboard']}, "
                    f"failsafe={status.failsafe}, altitude={altitude:.3f}m, "
                    f"vertical_speed={vertical_speed:.3f}m/s"
                )

            if not ready:
                stable_since = None
                return False
            if stable_since is None:
                stable_since = time.monotonic()
            return time.monotonic() - stable_since >= stability_duration

        self._spin_until(takeoff_is_stable, timeout_s, "stable offboard takeoff")

    def _feedback_callback(self, feedback_msg) -> None:
        self.last_feedback_distance = float(feedback_msg.feedback.distance_remaining)

    def _message_rate(self, count: int, first: Optional[float], last: Optional[float]) -> Optional[float]:
        if count < 2 or first is None or last is None or last <= first:
            return None
        return (count - 1) / (last - first)

    def _write_report(self) -> None:
        directory = os.path.dirname(os.path.abspath(self.report_file))
        os.makedirs(directory, exist_ok=True)
        with open(self.report_file, "w", encoding="utf-8") as stream:
            json.dump(self.report, stream, indent=2, sort_keys=True)
            stream.write("\n")

    def run(self) -> bool:
        readiness_timeout = float(self.get_parameter("readiness_timeout_s").value)
        takeoff_timeout = float(self.get_parameter("takeoff_timeout_s").value)
        navigation_timeout = float(self.get_parameter("navigation_timeout_s").value)
        position_tolerance = float(self.get_parameter("position_tolerance").value)
        yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)

        scenario_start = time.monotonic()
        try:
            self.get_logger().info("Waiting for Nav3D lifecycle nodes...")
            self._wait_for_lifecycle(readiness_timeout)
            self.report["checks"]["lifecycle_active"] = True

            self.get_logger().info("Waiting for odometry, PX4 status and outgoing setpoints...")
            self._spin_until(
                lambda: self.latest_odom is not None
                and self.latest_vehicle_status is not None
                and self.setpoint_count > 0,
                readiness_timeout,
                "odometry, PX4 status and trajectory setpoints",
            )
            self.report["checks"]["runtime_topics_ready"] = True

            self.get_logger().info("Waiting for stable autonomous takeoff...")
            takeoff_start = time.monotonic()
            self._wait_for_takeoff(takeoff_timeout)
            self.report["metrics"]["takeoff_wait_s"] = time.monotonic() - takeoff_start
            takeoff_pose = self._pose_sample()
            self.report["takeoff_pose"] = asdict(takeoff_pose) if takeoff_pose else None
            self.report["checks"]["takeoff_complete"] = True

            self._spin_until(self.navigate_client.server_is_ready, readiness_timeout, "navigate_to_pose action server")

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.pose.position.x = self.goal.x
            goal_msg.pose.pose.position.y = self.goal.y
            goal_msg.pose.pose.position.z = self.goal.z
            goal_msg.pose.pose.orientation.w = math.cos(self.goal.yaw / 2.0)
            goal_msg.pose.pose.orientation.z = math.sin(self.goal.yaw / 2.0)
            goal_msg.behavior_tree = ""

            self.get_logger().info(
                f"Sending navigation goal x={self.goal.x:.2f}, y={self.goal.y:.2f}, z={self.goal.z:.2f}"
            )
            send_future = self.navigate_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
            self._spin_until(send_future.done, readiness_timeout, "goal acceptance")
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("NavigateToPose goal was rejected")
            self.report["checks"]["goal_accepted"] = True

            self.navigation_started = True
            navigation_start = time.monotonic()
            result_future = goal_handle.get_result_async()
            self._spin_until(result_future.done, navigation_timeout, "navigation result")
            self.navigation_started = False
            navigation_duration = time.monotonic() - navigation_start

            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError("NavigateToPose returned no result")

            result = wrapped_result.result
            self.report["action"] = {
                "status": int(wrapped_result.status),
                "error_code": int(result.error_code),
                "error_msg": result.error_msg,
            }
            self.report["metrics"]["navigation_duration_s"] = navigation_duration
            self.report["metrics"]["last_feedback_distance_m"] = self.last_feedback_distance

            if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
                raise RuntimeError(
                    f"Navigation failed with status={wrapped_result.status}, "
                    f"error_code={result.error_code}: {result.error_msg}"
                )
            self.report["checks"]["navigation_succeeded"] = True

            self._spin_until(lambda: self.latest_odom is not None, 2.0, "final odometry")
            final_pose = self._pose_sample()
            if final_pose is None:
                raise RuntimeError("Final odometry is unavailable")

            position_error = math.sqrt(
                (final_pose.x - self.goal.x) ** 2
                + (final_pose.y - self.goal.y) ** 2
                + (final_pose.z - self.goal.z) ** 2
            )
            yaw_error = abs(self._angular_distance(final_pose.yaw, self.goal.yaw))

            self.report["final_pose"] = asdict(final_pose)
            self.report["metrics"]["final_position_error_m"] = position_error
            self.report["metrics"]["final_yaw_error_rad"] = yaw_error

            if position_error > position_tolerance:
                raise RuntimeError(f"Final position error {position_error:.3f}m exceeds {position_tolerance:.3f}m")
            if yaw_error > yaw_tolerance:
                raise RuntimeError(f"Final yaw error {yaw_error:.3f}rad exceeds {yaw_tolerance:.3f}rad")
            if self.failsafe_seen_during_navigation:
                raise RuntimeError("PX4 entered failsafe during navigation")

            self.report["checks"]["final_pose_within_tolerance"] = True
            self.report["checks"]["no_px4_failsafe_during_navigation"] = True
            self.report["passed"] = True
            self.get_logger().info("SYSTEM TEST PASSED")
            return True
        except Exception as exc:  # noqa: BLE001 - the report must capture every system-test failure
            self.navigation_started = False
            self.report["failure"] = {
                "type": type(exc).__name__,
                "message": str(exc),
                "traceback": traceback.format_exc(),
            }
            self.get_logger().error(f"SYSTEM TEST FAILED: {exc}")
            return False
        finally:
            current_pose = self._pose_sample()
            self.report["last_observed_pose"] = asdict(current_pose) if current_pose else None
            if self.latest_vehicle_status is not None:
                self.report["last_vehicle_status"] = {
                    "arming_state": int(self.latest_vehicle_status.arming_state),
                    "navigation_state": int(self.latest_vehicle_status.nav_state),
                    "failsafe": bool(self.latest_vehicle_status.failsafe),
                }
            self.report["takeoff_diagnostics"] = self.takeoff_diagnostics
            self.report["metrics"]["scenario_wall_duration_s"] = time.monotonic() - scenario_start
            self.report["metrics"]["odom_messages"] = self.odom_count
            self.report["metrics"]["odom_rate_hz"] = self._message_rate(
                self.odom_count, self.odom_first_wall, self.odom_last_wall
            )
            self.report["metrics"]["odom_max_gap_s"] = self.odom_max_gap_s
            self.report["metrics"]["trajectory_setpoint_messages"] = self.setpoint_count
            self.report["metrics"]["trajectory_setpoint_rate_hz"] = self._message_rate(
                self.setpoint_count, self.setpoint_first_wall, self.setpoint_last_wall
            )
            self.report["metrics"]["trajectory_setpoint_max_gap_s"] = self.setpoint_max_gap_s
            self._write_report()
            self.get_logger().info(f"Report written to {self.report_file}")


def main() -> int:
    rclpy.init(args=sys.argv)
    node = TakeoffAndNavigateTest()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
