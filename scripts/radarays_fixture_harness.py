#!/usr/bin/env python3

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


FIXTURE_NAMES = {
    "static_cpu": "static_cpu_fixture.json",
    "dynamic_cpu": "dynamic_cpu_fixture.json",
    "dynamic_multi_cpu": "dynamic_multi_cpu_fixture.json",
    "egomotion_gpu": "egomotion_gpu_fixture.json",
}

WORLD_NAMES = {
    "static_cpu": "gz_static_radar_cpu.sdf",
    "dynamic_cpu": "gz_dynamic_radar_cpu.sdf",
    "dynamic_multi_cpu": "gz_dynamic_radar_cpu_multi.sdf",
    # gz_egomotion_radar_gpu.sdf already existed on disk (Tier1 #1, the
    # moving-SENSOR scenario -- distinct from dynamic_cpu's moving-target
    # scenario) but was never wired into colcon test at all, meaning this
    # package's entire GPU/OptiX path had zero automated coverage. See
    # README.md for the real numbers this was verified against.
    "egomotion_gpu": "gz_egomotion_radar_gpu.sdf",
}

TOPICS = {
    "scan": "radar/scan",
    "points": "radar/points",
    "image": "radar/image",
}


def share_root() -> Path:
    return Path(get_package_share_directory("radarays_gazebo_plugins"))


def fixture_path(world: str) -> Path:
    return share_root() / "testdata" / "radarays_harmonic" / FIXTURE_NAMES[world]


def world_path(world: str) -> Path:
    return share_root() / "worlds" / WORLD_NAMES[world]


def libexec_root() -> Path:
    return Path(get_package_share_directory("radarays_gazebo_plugins")).parents[1] / "lib" / "radarays_gazebo_plugins"


def default_output_path(world: str) -> Path:
    return Path("/tmp") / f"radarays_harmonic_{world}_capture.json"


def default_log_path(world: str) -> Path:
    return Path("/tmp") / f"radarays_harmonic_{world}_gz.log"


def ensure_ros_log_dir() -> None:
    log_dir = Path("/tmp") / "radarays_gazebo_plugins_ros_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(log_dir))


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def stamp_to_float(msg) -> Optional[float]:
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
        return None
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def finite_values(values: List[float], range_min: float, range_max: float) -> List[float]:
    return [value for value in values if math.isfinite(value) and range_min <= value <= range_max]


def rate_from_stamps(stamps: List[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    deltas = [b - a for a, b in zip(stamps[:-1], stamps[1:]) if b > a]
    if not deltas:
        return 0.0
    return 1.0 / (sum(deltas) / len(deltas))


def summarize_scan(msg: LaserScan) -> Dict[str, object]:
    ranges = list(msg.ranges)
    finite = finite_values(ranges, msg.range_min, msg.range_max)
    center_index = len(ranges) // 2
    window_radius = 5
    window_start = max(0, center_index - window_radius)
    window_stop = min(len(ranges), center_index + window_radius + 1)
    center_window = ranges[window_start:window_stop]
    center_window_finite = finite_values(center_window, msg.range_min, msg.range_max)

    latest = {
        "finite_count": len(finite),
        "finite_ratio": (len(finite) / len(ranges)) if ranges else 0.0,
        "min_finite_range": min(finite) if finite else None,
        "max_finite_range": max(finite) if finite else None,
        "median_finite_range": statistics.median(finite) if finite else None,
        "center_index": center_index,
        "center_value": ranges[center_index] if ranges else None,
        "center_window": center_window,
        "center_window_finite_count": len(center_window_finite),
    }

    return {
        "sensor_config": {
            "frame_id": msg.header.frame_id,
            "beam_count": len(ranges),
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "scan_time": msg.scan_time,
            "time_increment": msg.time_increment,
        },
        "latest": latest,
        "probe": {
            "center_index": center_index,
            "center_value": latest["center_value"],
            "center_window": center_window,
        },
    }


def summarize_cloud(msg: PointCloud2) -> Dict[str, object]:
    finite_points: List[List[float]] = []
    nan_points = 0

    for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            finite_points.append([x, y, z])
        else:
            nan_points += 1

    bbox = None
    sample_points = []
    if finite_points:
        xs = [p[0] for p in finite_points]
        ys = [p[1] for p in finite_points]
        zs = [p[2] for p in finite_points]
        bbox = {
            "min": [min(xs), min(ys), min(zs)],
            "max": [max(xs), max(ys), max(zs)],
            "midpoint": [
                (min(xs) + max(xs)) / 2.0,
                (min(ys) + max(ys)) / 2.0,
                (min(zs) + max(zs)) / 2.0,
            ],
        }
        sample_points = finite_points[:5]

    return {
        "sensor_config": {
            "frame_id": msg.header.frame_id,
            "height": msg.height,
            "width": msg.width,
            "point_step": msg.point_step,
            "row_step": msg.row_step,
        },
        "latest": {
            "finite_point_count": len(finite_points),
            "nan_point_count": nan_points,
            "bbox": bbox,
            "sample_points": sample_points,
        },
        "probe": {
            "finite_point_count": len(finite_points),
            "bbox_midpoint_x": bbox["midpoint"][0] if bbox else None,
            "bbox_midpoint_y": bbox["midpoint"][1] if bbox else None,
        },
    }


def summarize_image(msg: Image) -> Dict[str, object]:
    values = list(msg.data)
    nonzero_values = [value for value in values if value != 0]
    max_value = max(values) if values else 0
    nonzero_count = len(nonzero_values)
    nonzero_ratio = (nonzero_count / len(values)) if values else 0.0

    center_column = msg.width // 2 if msg.width else 0
    center_column_values = []
    if msg.width > 0 and msg.height > 0:
        step = msg.step if msg.step > 0 else msg.width
        center_column_values = [
            values[row * step + center_column]
            for row in range(msg.height)
            if (row * step + center_column) < len(values)
        ]

    # Strongest return within a row window (not the column-wide argmax: with
    # multi-reflection recording on, a legitimate secondary bounce can
    # reasonably tie or beat the primary echo elsewhere in the column, so
    # pinning a single global peak position is not a robust invariant).
    # What should hold is that *near the known primary target range* there
    # is a strong return well above the ambient noise floor -- that is what
    # actually distinguishes "real signal" from "any nonzero image", which
    # nonzero/max alone would not catch.
    center_column_peak_row = None
    center_column_peak_value = 0
    if center_column_values:
        center_column_peak_row = max(range(len(center_column_values)), key=lambda r: center_column_values[r])
        center_column_peak_value = center_column_values[center_column_peak_row]

    return {
        "sensor_config": {
            "frame_id": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
            "step": msg.step,
        },
        "latest": {
            "nonzero_count": nonzero_count,
            "nonzero_ratio": nonzero_ratio,
            "max_value": max_value,
            "min_value": min(values) if values else 0,
            "center_column_nonzero_count": sum(1 for value in center_column_values if value != 0),
            "center_column_max_value": max(center_column_values) if center_column_values else 0,
            "center_column_peak_row": center_column_peak_row,
            "center_column_peak_value": center_column_peak_value,
            "center_column_values": center_column_values,
        },
        "probe": {
            "nonzero_count": nonzero_count,
            "max_value": max_value,
            "center_column_nonzero_count": sum(1 for value in center_column_values if value != 0),
            "center_column_peak_row": center_column_peak_row,
        },
    }


class TopicCollector(Node):
    def __init__(self) -> None:
        super().__init__("radarays_fixture_collector")
        self.scan_messages: List[Dict[str, object]] = []
        self.points_messages: List[Dict[str, object]] = []
        self.image_messages: List[Dict[str, object]] = []
        self.create_subscription(LaserScan, TOPICS["scan"], self._on_scan, 10)
        self.create_subscription(PointCloud2, TOPICS["points"], self._on_points, 10)
        self.create_subscription(Image, TOPICS["image"], self._on_image, qos_profile_sensor_data)

    def _on_scan(self, msg: LaserScan) -> None:
        summary = summarize_scan(msg)
        summary["stamp"] = stamp_to_float(msg)
        self.scan_messages.append(summary)

    def _on_points(self, msg: PointCloud2) -> None:
        summary = summarize_cloud(msg)
        summary["stamp"] = stamp_to_float(msg)
        self.points_messages.append(summary)

    def _on_image(self, msg: Image) -> None:
        summary = summarize_image(msg)
        summary["stamp"] = stamp_to_float(msg)
        self.image_messages.append(summary)


def finalize_capture(world: str, collector: TopicCollector, duration_sec: float, log_path: Path) -> Dict[str, object]:
    scan_stamps = [msg["stamp"] for msg in collector.scan_messages if msg["stamp"] is not None]
    point_stamps = [msg["stamp"] for msg in collector.points_messages if msg["stamp"] is not None]
    image_stamps = [msg["stamp"] for msg in collector.image_messages if msg["stamp"] is not None]

    latest_scan = collector.scan_messages[-1] if collector.scan_messages else None
    latest_points = collector.points_messages[-1] if collector.points_messages else None
    latest_image = collector.image_messages[-1] if collector.image_messages else None

    center_series = [
        msg["probe"]["center_value"] for msg in collector.scan_messages
        if msg["probe"]["center_value"] is not None and math.isfinite(msg["probe"]["center_value"])
    ]
    center_span = (max(center_series) - min(center_series)) if len(center_series) >= 2 else 0.0
    center_stddev = statistics.pstdev(center_series) if len(center_series) >= 2 else 0.0

    point_count_series = [int(msg["probe"]["finite_point_count"]) for msg in collector.points_messages]
    point_count_span = (max(point_count_series) - min(point_count_series)) if len(point_count_series) >= 2 else 0

    bbox_midpoint_x_series = [
        msg["probe"]["bbox_midpoint_x"] for msg in collector.points_messages
        if msg["probe"]["bbox_midpoint_x"] is not None
    ]
    bbox_midpoint_y_series = [
        msg["probe"]["bbox_midpoint_y"] for msg in collector.points_messages
        if msg["probe"]["bbox_midpoint_y"] is not None
    ]
    image_nonzero_series = [int(msg["probe"]["nonzero_count"]) for msg in collector.image_messages]
    image_nonzero_span = (max(image_nonzero_series) - min(image_nonzero_series)) if len(image_nonzero_series) >= 2 else 0
    image_max_series = [int(msg["probe"]["max_value"]) for msg in collector.image_messages]
    image_peak_row_series = [
        int(msg["probe"]["center_column_peak_row"]) for msg in collector.image_messages
        if msg["probe"]["center_column_peak_row"] is not None
    ]
    image_peak_row_span = (max(image_peak_row_series) - min(image_peak_row_series)) if len(image_peak_row_series) >= 2 else 0
    image_peak_row_stddev = statistics.pstdev(image_peak_row_series) if len(image_peak_row_series) >= 2 else 0.0

    result = {
        "world": world,
        "world_file": WORLD_NAMES[world],
        "topics": TOPICS,
        "capture": {
            "started_at": iso_now(),
            "duration_sec": duration_sec,
            "message_timeout_sec": 20.0,
            "gz_log_path": str(log_path),
        },
        "scan": {
            "message_count": len(collector.scan_messages),
            "rate_hz_estimate": rate_from_stamps(scan_stamps),
            "latest": latest_scan["latest"] if latest_scan else None,
            "sensor_config": latest_scan["sensor_config"] if latest_scan else None,
            "series": {
                "center_value": center_series,
                "center_value_span": center_span,
                "center_value_stddev": center_stddev,
                "finite_count": [msg["latest"]["finite_count"] for msg in collector.scan_messages],
            },
            "representative_sample": latest_scan["probe"] if latest_scan else None,
        },
        "points": {
            "message_count": len(collector.points_messages),
            "rate_hz_estimate": rate_from_stamps(point_stamps),
            "latest": latest_points["latest"] if latest_points else None,
            "sensor_config": latest_points["sensor_config"] if latest_points else None,
            "series": {
                "finite_point_count": point_count_series,
                "finite_point_count_span": point_count_span,
                "bbox_midpoint_x": bbox_midpoint_x_series,
                "bbox_midpoint_y": bbox_midpoint_y_series,
            },
            "representative_sample": latest_points["latest"]["sample_points"] if latest_points else None,
        },
        "image": {
            "message_count": len(collector.image_messages),
            "rate_hz_estimate": rate_from_stamps(image_stamps),
            "latest": latest_image["latest"] if latest_image else None,
            "sensor_config": latest_image["sensor_config"] if latest_image else None,
            "series": {
                "nonzero_count": image_nonzero_series,
                "nonzero_count_span": image_nonzero_span,
                "max_value": image_max_series,
                "center_column_peak_row": image_peak_row_series,
                "center_column_peak_row_span": image_peak_row_span,
                "center_column_peak_row_stddev": image_peak_row_stddev,
            },
            "representative_sample": latest_image["probe"] if latest_image else None,
        },
    }
    return result


def launch_world(world: str, log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")
    cmd = [
        "gz",
        "sim",
        "-s",
        "-r",
        "--headless-rendering",
        "-v",
        "1",
        str(world_path(world)),
    ]
    env = os.environ.copy()
    resource_root = str(world_path(world).parent)
    existing = env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"{resource_root}:{existing}" if existing else resource_root
    return subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        env=env,
    )


def stop_world(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def capture_world(world: str, output_path: Path, duration_sec: float, timeout_sec: float) -> int:
    log_path = default_log_path(world)
    process = launch_world(world, log_path)
    ensure_ros_log_dir()
    rclpy.init(args=None)
    collector = TopicCollector()

    try:
        first_deadline = time.monotonic() + timeout_sec
        while time.monotonic() < first_deadline:
            if process.poll() is not None:
                print(f"gz sim exited early with code {process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)
            if collector.scan_messages and collector.points_messages and collector.image_messages:
                break
        else:
            print(
                f"Timed out waiting for {TOPICS['scan']}, {TOPICS['points']}, and {TOPICS['image']}. See {log_path}",
                file=sys.stderr,
            )
            return 1

        capture_end = time.monotonic() + duration_sec
        while time.monotonic() < capture_end:
            if process.poll() is not None:
                print(f"gz sim exited during capture with code {process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)

        result = finalize_capture(world, collector, duration_sec, log_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Wrote capture for {world} to {output_path}")
        print(f"Gazebo log saved to {log_path}")
        return 0
    finally:
        try:
            collector.destroy_node()
        finally:
            rclpy.shutdown()
            stop_world(process)


def load_json(path: Path) -> Dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def ensure(condition: bool, message: str, failures: List[str]) -> None:
    if not condition:
        failures.append(message)


def compare_metric_range(actual: Optional[float], expected: Dict[str, float], label: str, failures: List[str]) -> None:
    if actual is None:
        failures.append(f"{label}: missing value")
        return
    minimum = expected.get("min")
    maximum = expected.get("max")
    if minimum is not None and actual < minimum:
        failures.append(f"{label}: {actual:.3f} < min {minimum:.3f}")
    if maximum is not None and actual > maximum:
        failures.append(f"{label}: {actual:.3f} > max {maximum:.3f}")


def compare_capture(world: str, result_path: Path) -> int:
    fixture = load_json(fixture_path(world))
    result = load_json(result_path)
    failures: List[str] = []

    ensure(result.get("world") == world, f"world mismatch: expected {world}, got {result.get('world')}", failures)

    scan = result.get("scan") or {}
    points = result.get("points") or {}
    image = result.get("image") or {}
    scan_latest = scan.get("latest") or {}
    point_latest = points.get("latest") or {}
    image_latest = image.get("latest") or {}
    expected_scan = fixture["expected"]["scan"]
    expected_points = fixture["expected"]["points"]
    expected_image = fixture["expected"]["image"]

    ensure(scan.get("message_count", 0) >= expected_scan["min_messages"],
           f"scan message_count {scan.get('message_count', 0)} < {expected_scan['min_messages']}",
           failures)
    ensure(points.get("message_count", 0) >= expected_points["min_messages"],
           f"points message_count {points.get('message_count', 0)} < {expected_points['min_messages']}",
           failures)
    ensure(image.get("message_count", 0) >= expected_image["min_messages"],
           f"image message_count {image.get('message_count', 0)} < {expected_image['min_messages']}",
           failures)
    ensure(scan.get("rate_hz_estimate", 0.0) >= expected_scan["min_rate_hz"],
           f"scan rate {scan.get('rate_hz_estimate', 0.0):.3f} < {expected_scan['min_rate_hz']:.3f}",
           failures)
    ensure(points.get("rate_hz_estimate", 0.0) >= expected_points["min_rate_hz"],
           f"points rate {points.get('rate_hz_estimate', 0.0):.3f} < {expected_points['min_rate_hz']:.3f}",
           failures)
    ensure(image.get("rate_hz_estimate", 0.0) >= expected_image["min_rate_hz"],
           f"image rate {image.get('rate_hz_estimate', 0.0):.3f} < {expected_image['min_rate_hz']:.3f}",
           failures)

    ensure(scan_latest.get("finite_count", 0) >= expected_scan["latest_finite_count_min"],
           f"scan finite_count {scan_latest.get('finite_count', 0)} < {expected_scan['latest_finite_count_min']}",
           failures)
    ensure(point_latest.get("finite_point_count", 0) >= expected_points["latest_finite_points_min"],
           f"points finite_point_count {point_latest.get('finite_point_count', 0)} < {expected_points['latest_finite_points_min']}",
           failures)
    ensure(image_latest.get("nonzero_count", 0) >= expected_image["latest_nonzero_count_min"],
           f"image nonzero_count {image_latest.get('nonzero_count', 0)} < {expected_image['latest_nonzero_count_min']}",
           failures)
    ensure(image_latest.get("max_value", 0) >= expected_image["latest_max_value_min"],
           f"image max_value {image_latest.get('max_value', 0)} < {expected_image['latest_max_value_min']}",
           failures)

    compare_metric_range(
        scan_latest.get("center_value"),
        expected_scan["center_value_range"],
        "static_cpu scan center_value",
        failures,
    )
    ensure(scan.get("series", {}).get("center_value_stddev", 0.0) <= expected_scan["center_value_stddev_max"],
           f"static_cpu scan center stddev {scan.get('series', {}).get('center_value_stddev', 0.0):.3f} > {expected_scan['center_value_stddev_max']:.3f}",
           failures)
    ensure(scan_latest.get("center_window_finite_count", 0) >= expected_scan["center_window_finite_min"],
           f"static_cpu center_window_finite_count {scan_latest.get('center_window_finite_count', 0)} < {expected_scan['center_window_finite_min']}",
           failures)

    bbox = point_latest.get("bbox")
    if not bbox:
        failures.append("static_cpu points bbox missing")
    else:
        compare_metric_range(bbox["midpoint"][0], expected_points["bbox_midpoint_x_range"], "static_cpu points bbox_midpoint_x", failures)
        ensure((bbox["max"][1] - bbox["min"][1]) >= expected_points["bbox_y_span_min"],
               f"static_cpu points bbox y span {(bbox['max'][1] - bbox['min'][1]):.3f} < {expected_points['bbox_y_span_min']:.3f}",
               failures)
    sensor_config = image.get("sensor_config") or {}
    ensure(sensor_config.get("width") == expected_image["width"],
           f"image width {sensor_config.get('width')} != {expected_image['width']}",
           failures)
    ensure(sensor_config.get("height") == expected_image["height"],
           f"image height {sensor_config.get('height')} != {expected_image['height']}",
           failures)
    ensure(image_latest.get("center_column_nonzero_count", 0) >= expected_image["center_column_nonzero_count_min"],
           f"image center_column_nonzero_count {image_latest.get('center_column_nonzero_count', 0)} < {expected_image['center_column_nonzero_count_min']}",
           failures)

    if "center_column_target_window" in expected_image:
        window = expected_image["center_column_target_window"]
        values = image_latest.get("center_column_values") or []
        window_values = values[window["row_min"]:window["row_max"] + 1]
        window_max = max(window_values) if window_values else 0
        ensure(window_max >= window["min_value"],
               f"image center_column max in rows [{window['row_min']}, {window['row_max']}] "
               f"is {window_max} < {window['min_value']} (no strong return near the known target range)",
               failures)

    # Dynamic-scenario checks: the static fixture wants stability (small
    # stddev/span); a moving-geometry fixture instead asserts the signal
    # actually changes over the capture window. Optional so the static
    # fixture doesn't need dummy values for a check that doesn't apply to it.
    if "center_value_span_min" in expected_scan:
        span = scan.get("series", {}).get("center_value_span", 0.0)
        ensure(span >= expected_scan["center_value_span_min"],
               f"scan center_value span {span:.3f} < {expected_scan['center_value_span_min']:.3f} (expected motion)",
               failures)
    if "center_value_stddev_min" in expected_scan:
        stddev = scan.get("series", {}).get("center_value_stddev", 0.0)
        ensure(stddev >= expected_scan["center_value_stddev_min"],
               f"scan center_value stddev {stddev:.3f} < {expected_scan['center_value_stddev_min']:.3f} (expected motion)",
               failures)
    if "bbox_midpoint_x_span_min" in expected_points:
        series = points.get("series", {}).get("bbox_midpoint_x", [])
        span = (max(series) - min(series)) if len(series) >= 2 else 0.0
        ensure(span >= expected_points["bbox_midpoint_x_span_min"],
               f"points bbox_midpoint_x span {span:.3f} < {expected_points['bbox_midpoint_x_span_min']:.3f} (expected motion)",
               failures)
    if "bbox_midpoint_y_span_min" in expected_points:
        # Distinct from x-span: a rotating (not translating) off-boresight
        # object mostly shows up here, since its own position barely moves
        # but its footprint/visible-point-set does as it yaws.
        series = points.get("series", {}).get("bbox_midpoint_y", [])
        span = (max(series) - min(series)) if len(series) >= 2 else 0.0
        ensure(span >= expected_points["bbox_midpoint_y_span_min"],
               f"points bbox_midpoint_y span {span:.3f} < {expected_points['bbox_midpoint_y_span_min']:.3f} (expected motion)",
               failures)
    if "center_column_peak_row_span_min" in expected_image:
        span = image.get("series", {}).get("center_column_peak_row_span", 0)
        ensure(span >= expected_image["center_column_peak_row_span_min"],
               f"image center_column_peak_row span {span} < {expected_image['center_column_peak_row_span_min']} (expected motion)",
               failures)
    if "nonzero_count_span_min" in expected_image:
        # Whole-image nonzero count changing captures effects a single
        # center-column check can't -- e.g. an off-boresight rotating
        # object changing how much of the scene it reflects.
        span = image.get("series", {}).get("nonzero_count_span", 0)
        ensure(span >= expected_image["nonzero_count_span_min"],
               f"image nonzero_count span {span} < {expected_image['nonzero_count_span_min']} (expected motion)",
               failures)

    if failures:
        print(f"Comparison against {fixture_path(world)} failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print(f"{world} capture matches expected fixture {fixture_path(world)}")
    return 0


def parse_args(argv: List[str]) -> argparse.Namespace:
    command_name = Path(argv[0]).name
    parser = argparse.ArgumentParser(prog=command_name)
    parser.add_argument("world", choices=sorted(FIXTURE_NAMES.keys()))
    parser.add_argument("--output", type=Path, help="Output path for capture JSON")
    parser.add_argument("--result", type=Path, help="Result JSON to compare")
    parser.add_argument("--duration", type=float, help="Capture duration in seconds")
    parser.add_argument("--timeout", type=float, default=20.0, help="Timeout waiting for the first scan and point cloud")
    return parser.parse_args(argv[1:])


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    command_name = Path(argv[0]).name

    if "capture" in command_name:
        output = args.output or default_output_path(args.world)
        fixture = load_json(fixture_path(args.world))
        duration = args.duration or fixture["capture_defaults"]["duration_sec"]
        return capture_world(args.world, output, float(duration), float(args.timeout))

    if "compare" in command_name:
        result = args.result or default_output_path(args.world)
        if not result.exists():
            print(f"Result file not found: {result}", file=sys.stderr)
            return 1
        return compare_capture(args.world, result)

    print(f"Unsupported entrypoint name: {command_name}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
