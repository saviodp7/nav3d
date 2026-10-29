#!/usr/bin/env python3

"""Verify the automatic takeoff"""

import json
import math
import os
import sys
import time
import traceback
from collections.abc import Callable
from dataclasses import asdict, dataclass

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float


class TakeoffTest(Node):
    def __init__(self) -> None:
        super().__init__("takeoff_test")

        self.declare_parameter("report_file", "/tmp/nav3d_takeoff_test.json")
        self.declare_parameter("readiness_timeout_s", 90.0)
        self.declare_parameter("takeoff_timeout_s", 60.0)
        self.declare_parameter("minimum_takeoff_climb_m", 0.8)
        self.declare_parameter("takeoff_vertical_speed_tolerance", 0.25)
        self.declare_parameter("maximum_horizontal_drift_m", 0.5)
        self.declare_parameter("maximum_yaw_error_rad", 0.35)
        self.declare_parameter("takeoff_stability_duration_s", 1.0)
        self.declare_parameter("post_takeoff_observation_s", 3.0)

        self.report_file = str(self.get_parameter("report_file").value)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_odom: Odometry | None = None
        self.latest_vehicle_status: VehicleStatus | None = None
        self.initial_pose: PoseSample | None = None
        self.max_horizontal_drift_m = 0.0
        self.max_yaw_error_rad = 0.0
        self.odom_count = 0
        self.setpoint_count = 0
        self.odom_first_wall: float | None = None
        self.odom_last_wall: float | None = None
        self.setpoint_first_wall: float | None = None
        self.setpoint_last_wall: float | None = None
        self.odom_max_gap_s = 0.0
        self.setpoint_max_gap_s = 0.0
        self.failsafe_seen_after_arm = False
        self.takeoff_diagnostics = {}

        self.create_subscription(Odometry, "/odom", self._odom_callback, sensor_qos)
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._vehicle_status_callback,
            sensor_qos,
        )
        self.create_subscription(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self._setpoint_callback,
            sensor_qos,
        )
        self.lifecycle_client = self.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active"
        )

        self.report = {
            "schema_version": 2,
            "scenario": "takeoff_test",
            "world": "default",
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

    def _pose_sample(self) -> PoseSample | None:
        if self.latest_odom is None:
            return None
        position = self.latest_odom.pose.pose.position
        return PoseSample(
            position.x,
            position.y,
            position.z,
            self._yaw_from_odometry(self.latest_odom),
        )

    def _capture_initial_pose_if_disarmed(self) -> None:
        if (
            self.initial_pose is None
            and self.latest_vehicle_status is not None
            and self.latest_vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_DISARMED
        ):
            self.initial_pose = self._pose_sample()
            if self.initial_pose is not None:
                self.get_logger().info(f"Initial disarmed pose: {self.initial_pose}")

    def _pose_errors(self, pose: PoseSample) -> tuple[float, float]:
        assert self.initial_pose is not None
        horizontal_drift = math.hypot(
            pose.x - self.initial_pose.x, pose.y - self.initial_pose.y
        )
        yaw_delta = pose.yaw - self.initial_pose.yaw
        yaw_error = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
        return horizontal_drift, yaw_error

    def _odom_callback(self, msg: Odometry) -> None:
        now = time.monotonic()
        if self.odom_last_wall is not None:
            self.odom_max_gap_s = max(self.odom_max_gap_s, now - self.odom_last_wall)
        self.odom_first_wall = self.odom_first_wall or now
        self.odom_last_wall = now
        self.odom_count += 1
        self.latest_odom = msg
        self._capture_initial_pose_if_disarmed()
        if (
            self.initial_pose is not None
            and self.latest_vehicle_status is not None
            and self.latest_vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):
            pose = self._pose_sample()
            assert pose is not None
            horizontal_drift, yaw_error = self._pose_errors(pose)
            self.max_horizontal_drift_m = max(
                self.max_horizontal_drift_m, horizontal_drift
            )
            self.max_yaw_error_rad = max(self.max_yaw_error_rad, yaw_error)

    def _vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.latest_vehicle_status = msg
        self._capture_initial_pose_if_disarmed()
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED and msg.failsafe:
            self.failsafe_seen_after_arm = True

    def _setpoint_callback(self, _: TrajectorySetpoint) -> None:
        now = time.monotonic()
        if self.setpoint_last_wall is not None:
            self.setpoint_max_gap_s = max(
                self.setpoint_max_gap_s, now - self.setpoint_last_wall
            )
        self.setpoint_first_wall = self.setpoint_first_wall or now
        self.setpoint_last_wall = now
        self.setpoint_count += 1

    def _spin_until(
        self, predicate: Callable[[], bool], timeout_s: float, description: str
    ) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise TimeoutError(
            f"Timeout while waiting for {description} ({timeout_s:.1f}s)"
        )

    def _wait_for_lifecycle(self, timeout_s: float) -> None:
        self._spin_until(
            self.lifecycle_client.service_is_ready,
            timeout_s,
            "lifecycle manager service",
        )
        response_future = self.lifecycle_client.call_async(Trigger.Request())
        self._spin_until(response_future.done, timeout_s, "lifecycle manager response")
        response = response_future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"Nav3D lifecycle manager is not active: {message}")

    def _takeoff_conditions(self) -> bool:
        pose = self._pose_sample()
        if (
            pose is None
            or self.initial_pose is None
            or self.latest_vehicle_status is None
        ):
            return False

        status = self.latest_vehicle_status
        climb = pose.z - self.initial_pose.z
        horizontal_drift, yaw_error = self._pose_errors(pose)
        vertical_speed = abs(self.latest_odom.twist.twist.linear.z)
        minimum_climb = float(self.get_parameter("minimum_takeoff_climb_m").value)
        speed_tolerance = float(
            self.get_parameter("takeoff_vertical_speed_tolerance").value
        )
        maximum_horizontal_drift = float(
            self.get_parameter("maximum_horizontal_drift_m").value
        )
        maximum_yaw_error = float(self.get_parameter("maximum_yaw_error_rad").value)
        conditions = {
            "armed": status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            "offboard": status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            "not_failsafe": not status.failsafe and not self.failsafe_seen_after_arm,
            "minimum_relative_altitude": climb >= minimum_climb,
            "vertical_speed_stable": vertical_speed <= speed_tolerance,
            "horizontal_position_held": horizontal_drift <= maximum_horizontal_drift,
            "yaw_held": yaw_error <= maximum_yaw_error,
        }
        self.takeoff_diagnostics = {
            "altitude_m": pose.z,
            "relative_altitude_m": climb,
            "horizontal_drift_m": horizontal_drift,
            "yaw_error_rad": yaw_error,
            "vertical_speed_mps": vertical_speed,
            "arming_state": int(status.arming_state),
            "navigation_state": int(status.nav_state),
            "failsafe": bool(status.failsafe),
            "conditions": conditions,
        }
        return all(conditions.values())

    def _wait_for_takeoff(self, timeout_s: float) -> None:
        stability_duration = float(
            self.get_parameter("takeoff_stability_duration_s").value
        )
        stable_since: float | None = None
        last_diagnostic_log = 0.0

        def takeoff_is_stable() -> bool:
            nonlocal stable_since, last_diagnostic_log
            ready = self._takeoff_conditions()
            now = time.monotonic()
            if self.takeoff_diagnostics and now - last_diagnostic_log >= 5.0:
                last_diagnostic_log = now
                self.get_logger().info(
                    f"Takeoff state: {self.takeoff_diagnostics['conditions']}, "
                    f"climb={self.takeoff_diagnostics['relative_altitude_m']:.3f}m, "
                    f"vertical_speed={self.takeoff_diagnostics['vertical_speed_mps']:.3f}m/s"
                )
            if not ready:
                stable_since = None
                return False
            if stable_since is None:
                stable_since = now
            return now - stable_since >= stability_duration

        self._spin_until(takeoff_is_stable, timeout_s, "stable automatic takeoff")

    @staticmethod
    def _message_rate(
        count: int, first: float | None, last: float | None
    ) -> float | None:
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
        observation_duration = float(
            self.get_parameter("post_takeoff_observation_s").value
        )
        scenario_start = time.monotonic()
        try:
            self.get_logger().info("Waiting for Nav3D lifecycle nodes...")
            self._wait_for_lifecycle(readiness_timeout)
            self.report["checks"]["lifecycle_active"] = True

            self.get_logger().info(
                "Waiting for initial disarmed pose, PX4 status and outgoing setpoints..."
            )
            self._spin_until(
                lambda: self.initial_pose is not None
                and self.latest_odom is not None
                and self.latest_vehicle_status is not None
                and self.setpoint_count > 0,
                readiness_timeout,
                "initial disarmed pose, PX4 status and trajectory setpoints",
            )
            self.report["checks"]["runtime_topics_ready"] = True
            self.report["initial_pose"] = asdict(self.initial_pose)

            self.get_logger().info("Waiting for stable autonomous takeoff...")
            takeoff_start = time.monotonic()
            self._wait_for_takeoff(takeoff_timeout)
            self.report["metrics"]["takeoff_wait_s"] = time.monotonic() - takeoff_start
            takeoff_pose = self._pose_sample()
            self.report["takeoff_pose"] = asdict(takeoff_pose) if takeoff_pose else None
            if takeoff_pose is not None:
                self.report["metrics"]["takeoff_climb_m"] = (
                    takeoff_pose.z - self.initial_pose.z
                )
            self.report["checks"]["takeoff_complete"] = True

            # Observe a short hover; this test never creates a navigation action client.
            observation_start = time.monotonic()
            odom_count_at_start = self.odom_count
            setpoint_count_at_start = self.setpoint_count
            while (
                rclpy.ok()
                and time.monotonic() - observation_start < observation_duration
            ):
                rclpy.spin_once(self, timeout_sec=0.1)
                if not self._takeoff_conditions():
                    raise RuntimeError(
                        "Automatic takeoff conditions were lost during hover"
                    )
            if not rclpy.ok():
                raise RuntimeError("ROS shut down during post-takeoff hover")
            if (
                self.odom_count == odom_count_at_start
                or self.setpoint_count == setpoint_count_at_start
            ):
                raise RuntimeError(
                    "Odometry or trajectory setpoints stopped during hover"
                )
            if self.max_horizontal_drift_m > float(
                self.get_parameter("maximum_horizontal_drift_m").value
            ):
                raise RuntimeError(
                    f"Horizontal drift exceeded limit: {self.max_horizontal_drift_m:.3f}m"
                )
            if self.max_yaw_error_rad > float(
                self.get_parameter("maximum_yaw_error_rad").value
            ):
                raise RuntimeError(
                    f"Yaw error exceeded limit: {self.max_yaw_error_rad:.3f}rad"
                )
            self.report["checks"]["post_takeoff_hover_stable"] = True
            self.report["metrics"]["post_takeoff_observation_s"] = (
                time.monotonic() - observation_start
            )
            self.report["passed"] = True
            self.get_logger().info(
                "SYSTEM TEST PASSED: automatic takeoff in default world"
            )
            return True
        except Exception as exc:
            self.report["failure"] = {
                "type": type(exc).__name__,
                "message": str(exc),
                "traceback": traceback.format_exc(),
            }
            self.get_logger().error(f"SYSTEM TEST FAILED: {exc}")
            return False
        finally:
            current_pose = self._pose_sample()
            self.report["last_observed_pose"] = (
                asdict(current_pose) if current_pose else None
            )
            if self.latest_vehicle_status is not None:
                self.report["last_vehicle_status"] = {
                    "arming_state": int(self.latest_vehicle_status.arming_state),
                    "navigation_state": int(self.latest_vehicle_status.nav_state),
                    "failsafe": bool(self.latest_vehicle_status.failsafe),
                }
            self.report["takeoff_diagnostics"] = self.takeoff_diagnostics
            self.report["metrics"][
                "max_horizontal_drift_m"
            ] = self.max_horizontal_drift_m
            self.report["metrics"]["max_yaw_error_rad"] = self.max_yaw_error_rad
            self.report["metrics"]["scenario_wall_duration_s"] = (
                time.monotonic() - scenario_start
            )
            self.report["metrics"]["odom_messages"] = self.odom_count
            self.report["metrics"]["odom_rate_hz"] = self._message_rate(
                self.odom_count, self.odom_first_wall, self.odom_last_wall
            )
            self.report["metrics"]["odom_max_gap_s"] = self.odom_max_gap_s
            self.report["metrics"]["trajectory_setpoint_messages"] = self.setpoint_count
            self.report["metrics"]["trajectory_setpoint_rate_hz"] = self._message_rate(
                self.setpoint_count, self.setpoint_first_wall, self.setpoint_last_wall
            )
            self.report["metrics"][
                "trajectory_setpoint_max_gap_s"
            ] = self.setpoint_max_gap_s
            self._write_report()
            self.get_logger().info(f"Report written to {self.report_file}")


def main() -> int:
    rclpy.init(args=sys.argv)
    node = TakeoffTest()
    try:
        passed = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
