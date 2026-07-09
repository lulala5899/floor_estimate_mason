#!/usr/bin/env python3
"""
Paper-aligned offset calibration for two collocated sensors.

This script implements the preprocessing + offset estimation flow from paper
arXiv:2601.02184v1, Section "Preprocessing and Time Alignment" and
"Offset Model and Estimation":

1) Resample each sensor stream to a uniform 30 s grid by averaging.
2) Inner-join the grid timestamps across sensors.
3) Apply Eq.(4) jump-threshold filtering:
   |p_i(t_m)-p_i(t_{m-1})| <= 1 hPa and |T_i(t_m)-T_i(t_{m-1})| <= 1 C.
4) Compute Eq.(8)-(9) closed-form offsets:
   - per timestamp reference = mean across sensors
   - per sensor offset beta_i = time-average(sensor - reference)
5) Write runtime offsets as "offset = -beta" into ROS YAML.
"""

from __future__ import annotations

import argparse
import os
import statistics
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import DefaultDict, Dict, List, Tuple

import yaml

_ROS_IMPORT_ERROR: Exception | None = None
try:
    import rclpy
    from barometer_interfaces.msg import Barometer
    from rclpy.node import Node
except Exception as exc:  # pragma: no cover (depends on ROS env)
    rclpy = None  # type: ignore[assignment]
    Barometer = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    _ROS_IMPORT_ERROR = exc

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_YAML_PATH = REPO_ROOT / "ros_barometer-main" / "serial_to_ros2" / "config" / "esp32_serial_baro.yaml"
ROS_SETUP_HINT = (
    "source /opt/ros/humble/setup.bash && "
    "source <repo>/ros_barometer-main/install/setup.bash"
)


# Raw sample tuple: (unix_time_s, pressure_hpa, temperature_c)
RawSample = Tuple[float, float, float]


@dataclass
class GridSample:
    """Mean pressure/temperature on one resampled grid timestamp."""

    pressure_hpa: float
    temperature_c: float


@dataclass
class CliArgs:
    """Command-line options used by the calibration pipeline."""

    mobile_mac: str
    base_mac: str
    duration_s: float
    delta_s: float
    jump_pressure_hpa: float
    jump_temp_c: float
    min_valid_bins: int
    mobile_topic: str
    base_topic: str
    yaml_path: str


class DualBarometerCollector(Node):
    """
    Collect raw barometer messages from two ROS topics.

    The script assumes current project topology:
    - mobile topic: /barometer
    - base topic:   /base/barometer
    """

    def __init__(self, mobile_topic: str, base_topic: str) -> None:
        super().__init__("offset_calibration_eq89")
        self.mobile_raw: List[RawSample] = []
        self.base_raw: List[RawSample] = []
        self.create_subscription(Barometer, mobile_topic, self._cb_mobile, 50)
        self.create_subscription(Barometer, base_topic, self._cb_base, 50)

    @staticmethod
    def _to_raw(msg: Barometer) -> RawSample:
        ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        pressure_hpa = float(msg.pressure) / 100.0
        temperature_c = float(msg.temperature)
        return ts, pressure_hpa, temperature_c

    def _cb_mobile(self, msg: Barometer) -> None:
        self.mobile_raw.append(self._to_raw(msg))

    def _cb_base(self, msg: Barometer) -> None:
        self.base_raw.append(self._to_raw(msg))


def parse_args() -> CliArgs:
    """Parse CLI and environment variables."""

    parser = argparse.ArgumentParser(
        description="Paper Eq.(4),(8),(9) offset calibration for ROS barometer streams."
    )
    parser.add_argument(
        "--mobile-mac",
        default=os.getenv("MOBILE_MAC", ""),
        help="Mobile MAC in underscore-upper form, e.g. AA_BB_CC_DD_EE_FF",
    )
    parser.add_argument(
        "--base-mac",
        default=os.getenv("BASE_MAC", ""),
        help="Base MAC in underscore-upper form, e.g. 11_22_33_44_55_66",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=float(os.getenv("CALIB_DURATION", "1800")),
        dest="duration_s",
        help="Collection duration in seconds (default: 1800)",
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=30.0,
        dest="delta_s",
        help="Resampling grid interval in seconds (paper default: 30)",
    )
    parser.add_argument(
        "--jump-pressure",
        type=float,
        default=1.0,
        dest="jump_pressure_hpa",
        help="Eq.(4) pressure jump threshold in hPa (default: 1.0)",
    )
    parser.add_argument(
        "--jump-temp",
        type=float,
        default=1.0,
        dest="jump_temp_c",
        help="Eq.(4) temperature jump threshold in C (default: 1.0)",
    )
    parser.add_argument(
        "--min-valid-bins",
        type=int,
        default=10,
        help="Minimum number of bins after Eq.(4) filtering",
    )
    parser.add_argument(
        "--mobile-topic",
        default="/barometer",
        help="ROS topic for mobile sensor (default: /barometer)",
    )
    parser.add_argument(
        "--base-topic",
        default="/base/barometer",
        help="ROS topic for base sensor (default: /base/barometer)",
    )
    parser.add_argument(
        "--yaml-path",
        default=str(DEFAULT_YAML_PATH),
        help="Target ROS YAML file to write cali_offsets",
    )
    ns = parser.parse_args()

    if not ns.mobile_mac or not ns.base_mac:
        parser.error("Both --mobile-mac and --base-mac are required.")
    if ns.duration_s <= 0 or ns.delta_s <= 0:
        parser.error("--duration and --delta must be > 0.")
    if ns.min_valid_bins < 1:
        parser.error("--min-valid-bins must be >= 1.")

    return CliArgs(
        mobile_mac=ns.mobile_mac,
        base_mac=ns.base_mac,
        duration_s=ns.duration_s,
        delta_s=ns.delta_s,
        jump_pressure_hpa=ns.jump_pressure_hpa,
        jump_temp_c=ns.jump_temp_c,
        min_valid_bins=ns.min_valid_bins,
        mobile_topic=ns.mobile_topic,
        base_topic=ns.base_topic,
        yaml_path=ns.yaml_path,
    )


def ensure_ros_available() -> None:
    """Fail fast with a clear hint when ROS Python env is not ready."""

    if _ROS_IMPORT_ERROR is not None:
        raise RuntimeError(
            "ROS2 Python modules unavailable.\n"
            f"Please run: {ROS_SETUP_HINT}"
        ) from _ROS_IMPORT_ERROR


def resample_average(raw: List[RawSample], delta_s: float) -> Dict[int, GridSample]:
    """Resample one stream to a uniform grid by averaging values in each bin.30s 重采样"""

    buckets: DefaultDict[int, List[Tuple[float, float]]] = defaultdict(list)
    for ts, pressure_hpa, temp_c in raw:
        key = int(ts // delta_s)
        buckets[key].append((pressure_hpa, temp_c))

    out: Dict[int, GridSample] = {}
    for key, values in buckets.items():
        out[key] = GridSample(
            pressure_hpa=statistics.fmean(v[0] for v in values),
            temperature_c=statistics.fmean(v[1] for v in values),
        )
    return out


def inner_join_bins(
    mobile_grid: Dict[int, GridSample],
    base_grid: Dict[int, GridSample],
) -> List[Tuple[int, GridSample, GridSample]]:
    """Inner join resampled bins across sensors (paper 'aligned samples').两传感器 inner join 对齐"""

    common_keys = sorted(set(mobile_grid).intersection(base_grid))
    return [(k, mobile_grid[k], base_grid[k]) for k in common_keys]


def filter_by_eq4(
    joined: List[Tuple[int, GridSample, GridSample]],
    jump_pressure_hpa: float,
    jump_temp_c: float,
) -> List[Tuple[int, GridSample, GridSample]]:
    """
    Apply Eq.(4) jump filtering on aligned bins.
    Eq.(4) 跳变过滤
    Keep t_m only if BOTH sensors satisfy:
    - |p_i(t_m)-p_i(t_{m-1})| <= threshold
    - |T_i(t_m)-T_i(t_{m-1})| <= threshold
    """

    if len(joined) < 2:
        return []

    valid: List[Tuple[int, GridSample, GridSample]] = []
    # Compare each consecutive aligned pair: (t_{m-1}, t_m).
    for prev, cur in zip(joined, joined[1:]):
        _, m_prev, b_prev = prev
        key, m_cur, b_cur = cur

        mobile_ok = (
            abs(m_cur.pressure_hpa - m_prev.pressure_hpa) <= jump_pressure_hpa
            and abs(m_cur.temperature_c - m_prev.temperature_c) <= jump_temp_c
        )
        base_ok = (
            abs(b_cur.pressure_hpa - b_prev.pressure_hpa) <= jump_pressure_hpa
            and abs(b_cur.temperature_c - b_prev.temperature_c) <= jump_temp_c
        )

        if mobile_ok and base_ok:
            valid.append((key, m_cur, b_cur))

    return valid


def estimate_betas_eq89(
    valid: List[Tuple[int, GridSample, GridSample]],
) -> Tuple[float, float, float, float]:
    """
    Estimate sensor offsets beta using Eq.(8)-(9) for N=2 sensors.

    Returns:
    beta_mobile_p, beta_mobile_t, beta_base_p, beta_base_t
    """

    mobile_dev_p: List[float] = []
    mobile_dev_t: List[float] = []
    base_dev_p: List[float] = []
    base_dev_t: List[float] = []

    for _, mobile, base in valid:
        # Eq.(8)/(9): per-timestamp reference is mean across sensors.
        p_ref = 0.5 * (mobile.pressure_hpa + base.pressure_hpa)
        t_ref = 0.5 * (mobile.temperature_c + base.temperature_c)

        # Eq.(8)/(9): per-sensor deviation from reference.
        mobile_dev_p.append(mobile.pressure_hpa - p_ref)
        mobile_dev_t.append(mobile.temperature_c - t_ref)
        base_dev_p.append(base.pressure_hpa - p_ref)
        base_dev_t.append(base.temperature_c - t_ref)

    # Eq.(8)/(9): time-average deviation => beta_i.
    beta_mobile_p = statistics.fmean(mobile_dev_p)
    beta_mobile_t = statistics.fmean(mobile_dev_t)
    beta_base_p = statistics.fmean(base_dev_p)
    beta_base_t = statistics.fmean(base_dev_t)

    # Enforce gauge constraint Eq.(6): sum(beta_i)=0 (numerically robust).
    mean_beta_p = 0.5 * (beta_mobile_p + beta_base_p)
    mean_beta_t = 0.5 * (beta_mobile_t + beta_base_t)
    beta_mobile_p -= mean_beta_p
    beta_base_p -= mean_beta_p
    beta_mobile_t -= mean_beta_t
    beta_base_t -= mean_beta_t

    return beta_mobile_p, beta_mobile_t, beta_base_p, beta_base_t


def write_offsets_to_yaml(
    yaml_path: str,
    mobile_mac: str,
    base_mac: str,
    beta_mobile_p: float,
    beta_mobile_t: float,
    beta_base_p: float,
    beta_base_t: float,
) -> None:
    """
    Write runtime offsets into ROS YAML.

    Model in paper:      sensor = common + beta + noise
    Runtime correction:  corrected = raw + offset
    Therefore:           offset = -beta
    """

    with open(yaml_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    params = cfg.setdefault("esp32_serial_baro", {}).setdefault("ros__parameters", {})
    cali = params.setdefault("cali_offsets", {})

    cali[mobile_mac] = {
        "pressure_offset": round(-beta_mobile_p, 4),
        "temperature_offset": round(-beta_mobile_t, 4),
        "linear_factor": 1.0,
    }
    cali[base_mac] = {
        "pressure_offset": round(-beta_base_p, 4),
        "temperature_offset": round(-beta_base_t, 4),
        "linear_factor": 1.0,
    }

    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)


def main() -> int:
    """Run collection -> preprocessing -> Eq.(8)(9) estimation -> YAML write.流程主入口"""

    args = parse_args()
    # Robustness fix: check ROS env before touching rclpy APIs.
    ensure_ros_available()

    node = None
    ros_initialized = False
    try:
        rclpy.init()
        ros_initialized = True
        node = DualBarometerCollector(args.mobile_topic, args.base_topic)

        print(
            "Starting offset calibration collection: "
            f"duration={args.duration_s:.1f}s, delta={args.delta_s:.1f}s",
            flush=True,
        )
        print(
            f"Topics: mobile={args.mobile_topic}, base={args.base_topic}",
            flush=True,
        )
        print(f"YAML target: {args.yaml_path}", flush=True)

        start = time.time()
        progress_interval_s = 5.0
        next_progress = start
        while time.time() - start < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            if now >= next_progress:
                elapsed = now - start
                remaining = max(0.0, args.duration_s - elapsed)
                print(
                    "[collect] "
                    f"t+{elapsed:6.1f}s/{args.duration_s:.1f}s "
                    f"rem={remaining:6.1f}s "
                    f"mobile={len(node.mobile_raw)} "
                    f"base={len(node.base_raw)}",
                    flush=True,
                )
                next_progress = now + progress_interval_s

        print(
            "Collection finished: "
            f"mobile_raw={len(node.mobile_raw)}, base_raw={len(node.base_raw)}",
            flush=True,
        )

        mobile_grid = resample_average(node.mobile_raw, args.delta_s)
        base_grid = resample_average(node.base_raw, args.delta_s)
        joined = inner_join_bins(mobile_grid, base_grid)
        valid = filter_by_eq4(
            joined,
            jump_pressure_hpa=args.jump_pressure_hpa,
            jump_temp_c=args.jump_temp_c,
        )

        if len(valid) < args.min_valid_bins:
            raise RuntimeError(
                f"Too few valid bins after Eq.(4): {len(valid)} < {args.min_valid_bins}. "
                "Extend collection duration and keep sensors collocated and static."
            )

        beta_mobile_p, beta_mobile_t, beta_base_p, beta_base_t = estimate_betas_eq89(valid)
        write_offsets_to_yaml(
            yaml_path=args.yaml_path,
            mobile_mac=args.mobile_mac,
            base_mac=args.base_mac,
            beta_mobile_p=beta_mobile_p,
            beta_mobile_t=beta_mobile_t,
            beta_base_p=beta_base_p,
            beta_base_t=beta_base_t,
        )

        print("Offset calibration finished.")
        print(f"M (joined bins)       = {len(joined)}")
        print(f"M' (Eq.4 valid bins)  = {len(valid)}")
        print("Estimated beta (paper model):")
        print(f"  mobile beta_p={beta_mobile_p:+.6f} hPa, beta_T={beta_mobile_t:+.6f} C")
        print(f"  base   beta_p={beta_base_p:+.6f} hPa, beta_T={beta_base_t:+.6f} C")
        print("Written runtime offsets (offset = -beta):")
        for mac, off_p, off_t in (
            (args.mobile_mac, -beta_mobile_p, -beta_mobile_t),
            (args.base_mac, -beta_base_p, -beta_base_t),
        ):
            print(f"  {mac}: pressure_offset={off_p:+.6f}, temperature_offset={off_t:+.6f}")
        print(f"YAML: {args.yaml_path}")
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if ros_initialized:
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        # Keep CLI output clean for expected runtime-precondition failures.
        print(f"ERROR: {exc}")
        raise SystemExit(1)
