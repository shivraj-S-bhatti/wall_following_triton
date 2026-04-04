#!/usr/bin/env python3

"""Generate a metric map PNG and likelihood-field PNG from a ROS map."""

import argparse
import os

from map_utils import LikelihoodFieldMap


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Generate the metric-map and likelihood-field artifacts used by "
            "Deliverable 1."
        )
    )
    parser.add_argument("--map-yaml", required=True, help="Path to the ROS map YAML file")
    parser.add_argument(
        "--metric-map-output",
        default="",
        help="Output PNG path for the metric map visualization",
    )
    parser.add_argument(
        "--likelihood-field-output",
        default="",
        help="Output PNG path for the likelihood-field visualization",
    )
    parser.add_argument(
        "--cache-output",
        default="",
        help="Output NPZ path for the cached numeric distance field",
    )
    parser.add_argument(
        "--regenerate",
        action="store_true",
        help="Ignore an existing cache and rebuild the field from the map image",
    )
    return parser.parse_args()


def default_output_path(map_yaml_path, suffix):
    stem, _ = os.path.splitext(map_yaml_path)
    return f"{stem}_{suffix}"


def main():
    args = parse_args()

    metric_map_output = args.metric_map_output or default_output_path(
        args.map_yaml, "metric_map.png"
    )
    likelihood_field_output = args.likelihood_field_output or default_output_path(
        args.map_yaml, "likelihood_field.png"
    )

    field_map = LikelihoodFieldMap.load_or_create(
        map_yaml_path=args.map_yaml,
        cache_path=args.cache_output or None,
        regenerate=args.regenerate,
    )
    field_map.save_metric_map_png(metric_map_output)
    field_map.save_likelihood_field_png(likelihood_field_output)

    print(f"map_yaml: {os.path.abspath(args.map_yaml)}")
    print(f"metric_map_png: {os.path.abspath(metric_map_output)}")
    print(f"likelihood_field_png: {os.path.abspath(likelihood_field_output)}")
    if args.cache_output:
        print(f"likelihood_field_cache: {os.path.abspath(args.cache_output)}")


if __name__ == "__main__":
    main()
