#!/usr/bin/env python3

"""Map loading and likelihood-field generation utilities."""

import heapq
import math
import os

import numpy as np
import yaml
from PIL import Image


class LikelihoodFieldMap:
    """Loads a ROS occupancy map and exposes a distance-field lookup table."""

    FREE = 0
    OCCUPIED = 1
    UNKNOWN = -1

    def __init__(
        self,
        map_yaml_path,
        occupancy_grid,
        resolution,
        origin,
        distance_field,
        image_path,
        meta,
    ):
        self.map_yaml_path = os.path.abspath(map_yaml_path)
        self.occupancy_grid = np.array(occupancy_grid, dtype=np.int8)
        self.resolution = float(resolution)
        self.origin = tuple(float(v) for v in origin[:3])
        self.distance_field = np.array(distance_field, dtype=np.float32)
        self.image_path = os.path.abspath(image_path)
        self.meta = dict(meta)
        self.height, self.width = self.occupancy_grid.shape
        self.max_distance = float(np.max(self.distance_field))

    @classmethod
    def load_or_create(cls, map_yaml_path, cache_path=None, regenerate=False):
        meta = cls._load_map_yaml(map_yaml_path)
        occupancy_grid = cls._load_occupancy_grid(meta)

        if cache_path and os.path.exists(cache_path) and not regenerate:
            cache = np.load(cache_path, allow_pickle=False)
            shape_matches = tuple(cache["occupancy_grid"].shape) == occupancy_grid.shape
            resolution_matches = math.isclose(
                float(cache["resolution"]), float(meta["resolution"])
            )
            origin_matches = np.allclose(cache["origin"], np.array(meta["origin"][:3]))
            if shape_matches and resolution_matches and origin_matches:
                return cls(
                    map_yaml_path=map_yaml_path,
                    occupancy_grid=cache["occupancy_grid"],
                    resolution=float(cache["resolution"]),
                    origin=cache["origin"],
                    distance_field=cache["distance_field"],
                    image_path=meta["image_path"],
                    meta=meta,
                )

        distance_field = cls._compute_distance_field(
            occupancy_grid, float(meta["resolution"])
        )
        instance = cls(
            map_yaml_path=map_yaml_path,
            occupancy_grid=occupancy_grid,
            resolution=float(meta["resolution"]),
            origin=meta["origin"],
            distance_field=distance_field,
            image_path=meta["image_path"],
            meta=meta,
        )
        if cache_path:
            instance.save_cache(cache_path)
        return instance

    @staticmethod
    def _load_map_yaml(map_yaml_path):
        with open(map_yaml_path, "r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle)

        image_path = meta["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

        return {
            "image_path": os.path.abspath(image_path),
            "resolution": float(meta["resolution"]),
            "origin": tuple(meta.get("origin", [0.0, 0.0, 0.0])),
            "negate": int(meta.get("negate", 0)),
            "occupied_thresh": float(meta.get("occupied_thresh", 0.65)),
            "free_thresh": float(meta.get("free_thresh", 0.196)),
        }

    @classmethod
    def _load_occupancy_grid(cls, meta):
        image = Image.open(meta["image_path"]).convert("L")
        pixels = np.asarray(image, dtype=np.uint8)

        if meta["negate"]:
            occupancy_prob = pixels.astype(np.float32) / 255.0
        else:
            occupancy_prob = (255.0 - pixels.astype(np.float32)) / 255.0

        occupancy = np.full(pixels.shape, cls.UNKNOWN, dtype=np.int8)
        occupancy[occupancy_prob > meta["occupied_thresh"]] = cls.OCCUPIED
        occupancy[occupancy_prob < meta["free_thresh"]] = cls.FREE
        return occupancy

    @classmethod
    def _compute_distance_field(cls, occupancy_grid, resolution):
        obstacle_mask = occupancy_grid != cls.FREE
        height, width = occupancy_grid.shape
        distances = np.full((height, width), np.inf, dtype=np.float64)
        queue = []

        for row, col in np.argwhere(obstacle_mask):
            distances[row, col] = 0.0
            heapq.heappush(queue, (0.0, int(row), int(col)))

        if not queue:
            fallback = math.hypot(height, width) * resolution
            return np.full((height, width), fallback, dtype=np.float32)

        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        )

        while queue:
            current_distance, row, col = heapq.heappop(queue)
            if current_distance > distances[row, col]:
                continue

            for delta_row, delta_col, step_cost in neighbors:
                next_row = row + delta_row
                next_col = col + delta_col
                if next_row < 0 or next_row >= height:
                    continue
                if next_col < 0 or next_col >= width:
                    continue

                next_distance = current_distance + step_cost
                if next_distance >= distances[next_row, next_col]:
                    continue

                distances[next_row, next_col] = next_distance
                heapq.heappush(queue, (next_distance, next_row, next_col))

        return (distances * resolution).astype(np.float32)

    def save_cache(self, output_path):
        self._ensure_parent_dir(output_path)
        np.savez_compressed(
            output_path,
            occupancy_grid=self.occupancy_grid,
            distance_field=self.distance_field,
            resolution=np.float32(self.resolution),
            origin=np.array(self.origin, dtype=np.float32),
        )

    def save_metric_map_png(self, output_path):
        self._ensure_parent_dir(output_path)
        image = np.full(self.occupancy_grid.shape, 127, dtype=np.uint8)
        image[self.occupancy_grid == self.FREE] = 255
        image[self.occupancy_grid == self.OCCUPIED] = 0
        Image.fromarray(image, mode="L").save(output_path)

    def save_likelihood_field_png(self, output_path):
        self._ensure_parent_dir(output_path)
        scale = max(self.max_distance, 1.0e-9)
        normalized = 1.0 - np.clip(self.distance_field / scale, 0.0, 1.0)
        image = np.round(255.0 * normalized).astype(np.uint8)
        Image.fromarray(image, mode="L").save(output_path)

    def world_to_map(self, x, y):
        map_x = int(math.floor((x - self.origin[0]) / self.resolution))
        map_y = int(math.floor((y - self.origin[1]) / self.resolution))

        if map_x < 0 or map_x >= self.width:
            return None
        if map_y < 0 or map_y >= self.height:
            return None

        row = self.height - 1 - map_y
        col = map_x
        return row, col

    def map_to_world(self, row, col):
        x = self.origin[0] + (col + 0.5) * self.resolution
        map_y = self.height - 1 - row
        y = self.origin[1] + (map_y + 0.5) * self.resolution
        return x, y

    def distance_at_world(self, x, y, default=None):
        cell = self.world_to_map(x, y)
        if cell is None:
            return self.max_distance if default is None else float(default)
        return float(self.distance_field[cell])

    def is_free_world(self, x, y):
        cell = self.world_to_map(x, y)
        if cell is None:
            return False
        return self.occupancy_grid[cell] == self.FREE

    @staticmethod
    def _ensure_parent_dir(path):
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
