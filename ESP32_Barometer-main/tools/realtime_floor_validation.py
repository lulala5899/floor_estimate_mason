#!/usr/bin/env python3
"""
Realtime validation for differential barometer + floor indexing.

This script subscribes to:
- /barometer
- /base/barometer

Then it reports:
- paired timing quality
- dp/dT/dh statistics
- floor index distribution by edge-threshold rule:
    floor k iff dh >= k*H - margin (clipped to [0, floor_count-1])
- nearest-floor error statistics (for comparability):
    err = min_k |dh - Hk|

Default floor map:
- floor height = 3 m
- 5 floors => Hk = [0, 3, 6, 9, 12]
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from collections import Counter
from dataclasses import dataclass
from typing import List, Tuple

_ROS_IMPORT_ERROR: Exception | None = None
try:
    import rclpy
    from barometer_interfaces.msg import Barometer
    from rclpy.node import Node
except Exception as exc:  # pragma: no cover
    _ROS_IMPORT_ERROR = exc
    rclpy = None  # type: ignore[assignment]
    Barometer = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]


Sample = Tuple[float, float, float, float]  # (t, pressure_hpa, temp_c, altitude_m)


@dataclass
class Args:
    duration_s: float
    max_pair_dt_s: float
    live_interval_s: float
    summary_only: bool
    floor_height_m: float
    floor_count: int
    floor_switch_margin_m: float
    mobile_topic: str
    base_topic: str


class Collector(Node):
    """Collect two barometer streams for a fixed realtime window."""

    def __init__(self, mobile_topic: str, base_topic: str) -> None:
        super().__init__("realtime_floor_validation")
        self.mobile: List[Sample] = []
        self.base: List[Sample] = []
        self.create_subscription(Barometer, mobile_topic, self._cb_mobile, 50)
        self.create_subscription(Barometer, base_topic, self._cb_base, 20)

    @staticmethod
    def _to_sample(msg: Barometer) -> Sample:
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return t, float(msg.pressure) / 100.0, float(msg.temperature), float(msg.altitude)

    def _cb_mobile(self, msg: Barometer) -> None:
        self.mobile.append(self._to_sample(msg))

    def _cb_base(self, msg: Barometer) -> None:
        self.base.append(self._to_sample(msg))


def parse_args() -> Args:
    parser = argparse.ArgumentParser(description="Realtime floor-index validation")
    parser.add_argument("--duration", type=float, default=120.0, dest="duration_s")
    parser.add_argument("--max-pair-dt", type=float, default=0.35, dest="max_pair_dt_s")
    parser.add_argument("--live-interval", type=float, default=1.0, dest="live_interval_s")
    parser.add_argument("--summary-only", action="store_true", help="Disable live rolling print; only final summary.")
    parser.add_argument("--floor-height", type=float, default=3.0, dest="floor_height_m")
    parser.add_argument("--floor-count", type=int, default=5, dest="floor_count")
    parser.add_argument(
        "--floor-switch-margin",
        type=float,
        default=0.2,
        dest="floor_switch_margin_m",
        help=(
            "Switch threshold margin in meters. "
            "With floor_height=3 and margin=0.2, 0->1 switch is at dh>=2.8."
        ),
    )
    parser.add_argument("--mobile-topic", default="/barometer")
    parser.add_argument("--base-topic", default="/base/barometer")
    ns = parser.parse_args()
    if ns.duration_s <= 0:
        parser.error("--duration must be > 0")
    if ns.max_pair_dt_s <= 0:
        parser.error("--max-pair-dt must be > 0")
    if ns.live_interval_s <= 0:
        parser.error("--live-interval must be > 0")
    if ns.floor_height_m <= 0:
        parser.error("--floor-height must be > 0")
    if ns.floor_count < 2:
        parser.error("--floor-count must be >= 2")
    if ns.floor_switch_margin_m < 0:
        parser.error("--floor-switch-margin must be >= 0")
    if ns.floor_switch_margin_m >= ns.floor_height_m:
        parser.error("--floor-switch-margin must be < --floor-height")
    return Args(
        duration_s=ns.duration_s,
        max_pair_dt_s=ns.max_pair_dt_s,
        live_interval_s=ns.live_interval_s,
        summary_only=ns.summary_only,
        floor_height_m=ns.floor_height_m,
        floor_count=ns.floor_count,
        floor_switch_margin_m=ns.floor_switch_margin_m,
        mobile_topic=ns.mobile_topic,
        base_topic=ns.base_topic,
    )


def nearest_floor(dh: float, hk: List[float]) -> Tuple[int, float]:
    """Return nearest floor index and absolute height error for one dh sample."""
    idx = min(range(len(hk)), key=lambda i: abs(dh - hk[i]))
    err = abs(dh - hk[idx])
    return idx, err


def threshold_floor_index(
    dh: float,
    floor_height_m: float,
    floor_count: int,
    floor_switch_margin_m: float,
) -> int:
    """
    Return floor index using a floor-edge switching threshold.

    Example with floor_height=3.0 and margin=0.2:
    - dh < 2.8  => floor 0
    - dh >= 2.8 => floor 1
    """
    idx = math.floor((dh + floor_switch_margin_m) / floor_height_m)
    return max(0, min(floor_count - 1, idx))


def _p95_from_sorted(values: List[float]) -> float:
    """
    Compute p95 using the script's original index rule.

    Keeping this exact rule preserves existing behavior and historical outputs.
    """
    return values[int(0.95 * (len(values) - 1))]


def build_paired_metrics(
    mobile: List[Sample],
    base: List[Sample],
    max_pair_dt_s: float,
    hk: List[float],
    floor_height_m: float,
    floor_switch_margin_m: float,
) -> dict | None:
    """Pair mobile/base by nearest timestamp and compute validation metrics."""

    if not mobile or not base:
        return None

    # Two-pointer nearest-neighbor pairing:
    # For each base sample, advance mobile index while the next mobile sample
    # is closer in timestamp. This avoids O(N*M) matching.
    mobile_idx = 0
    dt_list: List[float] = []
    dp_list: List[float] = []
    dT_list: List[float] = []
    dh_list: List[float] = []
    floor_idx_list: List[int] = []
    floor_err_list: List[float] = []

    for tb, pb, Tb, hb in base:
        while (
            mobile_idx + 1 < len(mobile)
            and abs(mobile[mobile_idx + 1][0] - tb) <= abs(mobile[mobile_idx][0] - tb)
        ):
            mobile_idx += 1

        tm, pm, Tm, hm = mobile[mobile_idx]
        dt = abs(tm - tb)
        if dt > max_pair_dt_s:
            continue

        dp = pm - pb
        dT = Tm - Tb
        dh = hm - hb
        idx = threshold_floor_index(
            dh=dh,
            floor_height_m=floor_height_m,
            floor_count=len(hk),
            floor_switch_margin_m=floor_switch_margin_m,
        )
        _, err = nearest_floor(dh, hk)

        dt_list.append(dt)
        dp_list.append(dp)
        dT_list.append(dT)
        dh_list.append(dh)
        floor_idx_list.append(idx)
        floor_err_list.append(err)

    if not dh_list:
        return None

    # Keep percentile behavior identical to previous implementation.
    dt_sorted = sorted(dt_list)
    floor_err_sorted = sorted(floor_err_list)
    dt_p95 = _p95_from_sorted(dt_sorted)
    err_p95 = _p95_from_sorted(floor_err_sorted)
    floor_dist = dict(sorted(Counter(floor_idx_list).items()))
    dominant_floor = max(floor_dist, key=floor_dist.get)

    return {
        "pairs": len(dh_list),
        "dt_mean": statistics.fmean(dt_list),
        "dt_p95": dt_p95,
        "dt_max": max(dt_list),
        "dp_mean": statistics.fmean(dp_list),
        "dp_std": statistics.pstdev(dp_list),
        "dT_mean": statistics.fmean(dT_list),
        "dT_std": statistics.pstdev(dT_list),
        "dh_mean": statistics.fmean(dh_list),
        "dh_std": statistics.pstdev(dh_list),
        "floor_dist": floor_dist,
        "dominant_floor": dominant_floor,
        "err_mean": statistics.fmean(floor_err_list),
        "err_p95": err_p95,
        "err_max": max(floor_err_list),
    }


def snapshot_metrics(
    node: Collector,
    max_pair_dt_s: float,
    hk: List[float],
    floor_height_m: float,
    floor_switch_margin_m: float,
) -> dict | None:
    """
    Build one metrics snapshot from current collector buffers.

    Streams are sorted by timestamp before pairing so nearest-neighbor matching
    is deterministic regardless of callback arrival order.
    """
    mobile = sorted(node.mobile, key=lambda x: x[0])
    base = sorted(node.base, key=lambda x: x[0])
    return build_paired_metrics(
        mobile,
        base,
        max_pair_dt_s,
        hk,
        floor_height_m=floor_height_m,
        floor_switch_margin_m=floor_switch_margin_m,
    )


def main() -> int:
    args = parse_args()
    if _ROS_IMPORT_ERROR is not None:
        print("ERROR: ROS2 Python env not ready.", file=sys.stderr)
        print(
            "Please run: source /opt/ros/humble/setup.bash && "
            "source <repo>/ros_barometer-main/install/setup.bash",
            file=sys.stderr,
        )
        return 1

    hk = [i * args.floor_height_m for i in range(args.floor_count)]
    print("floor_map(Hk):", hk)
    print(
        "floor_switch_rule: "
        f"dh >= k*H - margin -> floor k; "
        f"H={args.floor_height_m}, margin={args.floor_switch_margin_m}"
    )

    rclpy.init()
    node = Collector(args.mobile_topic, args.base_topic)
    try:
        t0 = time.time()
        next_live = t0 + args.live_interval_s
        while time.time() - t0 < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.summary_only:
                continue

            now = time.time()
            if now < next_live:
                continue

            metrics = snapshot_metrics(
                node,
                args.max_pair_dt_s,
                hk,
                floor_height_m=args.floor_height_m,
                floor_switch_margin_m=args.floor_switch_margin_m,
            )
            elapsed = now - t0
            if metrics is None:
                print(f"[t+{elapsed:6.1f}s] waiting data...")
            else:
                print(
                    f"[t+{elapsed:6.1f}s] "
                    f"pairs={metrics['pairs']} "
                    f"dh_mean={metrics['dh_mean']:+.4f}m "
                    f"dh_std={metrics['dh_std']:.4f}m "
                    f"floor={metrics['dominant_floor']} "
                    f"err_mean={metrics['err_mean']:.4f}m",
                    flush=True,
                )
            next_live += args.live_interval_s

        metrics = snapshot_metrics(
            node,
            args.max_pair_dt_s,
            hk,
            floor_height_m=args.floor_height_m,
            floor_switch_margin_m=args.floor_switch_margin_m,
        )
        if metrics is None:
            print("ERROR: no aligned pairs after nearest-neighbor pairing")
            return 3

        print("pairs:", metrics["pairs"])
        print(
            "dt mean/p95/max:",
            round(metrics["dt_mean"], 4),
            round(metrics["dt_p95"], 4),
            round(metrics["dt_max"], 4),
        )
        print("dp mean/std (hPa):", round(metrics["dp_mean"], 4), round(metrics["dp_std"], 4))
        print("dT mean/std (C):", round(metrics["dT_mean"], 4), round(metrics["dT_std"], 4))
        print("dh mean/std (m):", round(metrics["dh_mean"], 4), round(metrics["dh_std"], 4))
        print("floor distribution:", metrics["floor_dist"])
        print(
            "nearest floor error mean/p95/max (m):",
            round(metrics["err_mean"], 4),
            round(metrics["err_p95"], 4),
            round(metrics["err_max"], 4),
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
