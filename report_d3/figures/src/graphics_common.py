#!/usr/bin/env python3
"""Shared styling and export helpers for D3 report figures."""

from __future__ import annotations

from pathlib import Path
import math
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
REPORT_ROOT = REPO_ROOT / "report_d3"
FIGURES_ROOT = REPORT_ROOT / "figures"
GENERATED_ROOT = FIGURES_ROOT / "generated"

# Match Triton diagram disks in state/action figures; wedge inner radius should equal this.
ROBOT_RADIUS = 0.12

# Cohesive cool-neutral technical palette (print-friendly, less pastel clash).
PALETTE = {
    "ink": "#1c2430",
    "muted": "#5c6570",
    "grid": "#cfd6de",
    "panel": "#f4f6f8",
    "blue": "#2563eb",
    "teal": "#0d9488",
    "slate": "#475569",
    "indigo": "#4338ca",
    "violet": "#6d28d9",
    "sage": "#15803d",
    "amber": "#b45309",
    "coral": "#c2410c",
    "green": "#0f766e",
    "red": "#b91c1c",
}


def configure_matplotlib() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.edgecolor": PALETTE["grid"],
            "axes.labelcolor": PALETTE["ink"],
            "text.color": PALETTE["ink"],
            "axes.titleweight": "bold",
            "axes.titlesize": 12,
        }
    )


def load_yaml(path: Path) -> dict:
    with path.open() as handle:
        return yaml.safe_load(handle)


def load_d2_params() -> dict:
    return load_yaml(REPO_ROOT / "config" / "d2_params.yaml")


def load_d2_actions() -> dict:
    return load_yaml(REPO_ROOT / "config" / "actions_d2.yaml")["actions"]


def save_figure(fig: plt.Figure, name: str) -> Path:
    GENERATED_ROOT.mkdir(parents=True, exist_ok=True)
    png_path = GENERATED_ROOT / f"{name}.png"
    fig.savefig(png_path, dpi=220, bbox_inches="tight")
    return png_path


def add_robot(ax, center=(0.0, 0.0), radius: float = ROBOT_RADIUS, color: str | None = None) -> None:
    robot = patches.Circle(center, radius=radius, facecolor="white", edgecolor=color or PALETTE["ink"], linewidth=1.5)
    ax.add_patch(robot)


def add_sector(
    ax,
    inner: float,
    outer: float,
    theta1: float,
    theta2: float,
    color: str,
    label: str | None = None,
    label_radius: float | None = None,
    alpha: float = 0.22,
    edge_alpha: float = 0.8,
) -> None:
    wedge = patches.Wedge(
        (0.0, 0.0),
        outer,
        theta1,
        theta2,
        width=outer - inner,
        facecolor=color,
        edgecolor=color,
        linewidth=1.2,
        alpha=alpha,
    )
    wedge.set_edgecolor(color)
    wedge.set_linewidth(1.2)
    wedge.set_alpha(alpha)
    ax.add_patch(wedge)
    if label:
        radius = label_radius if label_radius is not None else (inner + outer) / 2.0
        angle = (theta1 + theta2) / 2.0
        x = radius * math.cos(math.radians(angle))
        y = radius * math.sin(math.radians(angle))
        ax.text(x, y, label, ha="center", va="center", fontsize=8, color=PALETTE["ink"], weight="bold")


def add_threshold_arc(
    ax,
    radius: float,
    theta1: float,
    theta2: float,
    label: str,
    y_offset: float = 0.0,
    label_angle: float | None = None,
    *,
    label_radial_offset: float = 0.0,
    inner_connect: float | None = None,
    label_bbox: bool = True,
) -> None:
    arc = patches.Arc(
        (0.0, 0.0),
        2 * radius,
        2 * radius,
        theta1=theta1,
        theta2=theta2,
        linewidth=1.0,
        linestyle="--",
        color=PALETTE["muted"],
        alpha=0.75,
    )
    ax.add_patch(arc)
    if inner_connect is not None and inner_connect < radius - 1e-6:
        for theta in (theta1, theta2):
            t = math.radians(theta)
            ax.plot(
                [inner_connect * math.cos(t), radius * math.cos(t)],
                [inner_connect * math.sin(t), radius * math.sin(t)],
                linestyle="--",
                linewidth=1.0,
                color=PALETTE["muted"],
                alpha=0.75,
            )
    angle = label_angle if label_angle is not None else (theta1 + theta2) / 2.0
    r_label = radius + label_radial_offset
    x = r_label * math.cos(math.radians(angle))
    y = r_label * math.sin(math.radians(angle)) + y_offset
    bbox = None
    if label_bbox:
        bbox = {
            "boxstyle": "round,pad=0.12",
            "facecolor": "white",
            "edgecolor": PALETTE["grid"],
            "linewidth": 0.45,
            "alpha": 0.93,
        }
    ax.text(
        x,
        y,
        label,
        ha="center",
        va="center",
        fontsize=7.5,
        color=PALETTE["muted"],
        bbox=bbox,
    )


def add_card(
    ax,
    x: float,
    y: float,
    width: float,
    height: float,
    title: str,
    body: str,
    facecolor: str,
    title_size: float = 9.8,
    body_size: float = 8.2,
    wrap_width: int = 30,
) -> None:
    card = patches.FancyBboxPatch(
        (x, y),
        width,
        height,
        boxstyle="round,pad=0.012,rounding_size=0.02",
        linewidth=1.3,
        edgecolor=facecolor,
        facecolor=facecolor,
        alpha=0.12,
    )
    ax.add_patch(card)
    ax.text(
        0.07,
        0.91,
        title,
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=title_size,
        weight="bold",
        color=PALETTE["ink"],
    )
    ax.text(
        0.07,
        0.74,
        textwrap.fill(body, width=wrap_width),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=body_size,
        color=PALETTE["ink"],
    )


def add_column_arrow(ax, x1: float, y: float, x2: float) -> None:
    arrow = patches.FancyArrowPatch(
        (x1, y),
        (x2, y),
        arrowstyle="-|>",
        mutation_scale=12,
        linewidth=1.4,
        color=PALETTE["muted"],
        alpha=0.9,
    )
    ax.add_patch(arrow)


def format_action_value(linear_y: float, angular_z: float) -> str:
    return f"v_y={linear_y:.2f}, ω_z={angular_z:.2f}"
