#!/usr/bin/env python3
"""Utilities for discretizing LiDAR observations into wall-following state bins."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class EncodedState:
    front_bin: str
    right_bin: str
    heading_bin: str
    front_min: float
    right_front: float
    right_rear: float
    right_min: float

    @property
    def key(self) -> str:
        return f"{self.front_bin}|{self.right_bin}|{self.heading_bin}"


class StateEncoder:
    def __init__(
        self,
        front_too_close: float = 0.55,
        right_too_close: float = 0.55,
        right_too_far: float = 0.95,
        heading_parallel_tolerance: float = 0.10,
    ):
        self.front_too_close = float(front_too_close)
        self.right_too_close = float(right_too_close)
        self.right_too_far = float(right_too_far)
        self.heading_parallel_tolerance = float(heading_parallel_tolerance)

    def encode(self, scan_msg) -> EncodedState:
        front_min = self._sector_min(scan_msg, -15.0, 15.0)
        right_front = self._sector_min(scan_msg, -70.0, -20.0)
        right_rear = self._sector_min(scan_msg, -110.0, -70.0)
        right_min = min(right_front, right_rear)

        front_bin = "too_close" if front_min < self.front_too_close else "safe"

        if right_min < self.right_too_close:
            right_bin = "too_close"
        elif right_min > self.right_too_far:
            right_bin = "too_far"
        else:
            right_bin = "good"

        heading_delta = right_front - right_rear
        if abs(heading_delta) <= self.heading_parallel_tolerance:
            heading_bin = "parallel"
        elif heading_delta < 0.0:
            heading_bin = "toward_wall"
        else:
            heading_bin = "away_from_wall"

        return EncodedState(
            front_bin=front_bin,
            right_bin=right_bin,
            heading_bin=heading_bin,
            front_min=front_min,
            right_front=right_front,
            right_rear=right_rear,
            right_min=right_min,
        )

    def _sector_min(self, scan_msg, start_deg: float, end_deg: float) -> float:
        if not scan_msg.ranges:
            return float(scan_msg.range_max)

        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)

        candidates = []
        angle = float(scan_msg.angle_min)
        for value in scan_msg.ranges:
            if self._angle_in_sector(angle, start_rad, end_rad):
                try:
                    numeric = float(value)
                except (TypeError, ValueError):
                    numeric = float("nan")
                if math.isfinite(numeric):
                    clipped = min(max(numeric, float(scan_msg.range_min)), float(scan_msg.range_max))
                    candidates.append(clipped)
            angle += float(scan_msg.angle_increment)

        if not candidates:
            return float(scan_msg.range_max)
        return min(candidates)

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _angle_in_sector(self, angle: float, sector_start: float, sector_end: float) -> bool:
        a = self._normalize_angle(angle)
        s = self._normalize_angle(sector_start)
        e = self._normalize_angle(sector_end)

        if s <= e:
            return s <= a <= e
        return a >= s or a <= e
