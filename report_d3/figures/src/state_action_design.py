#!/usr/bin/env python3
"""Generate state/action design assets for the D3 report.

Outputs separate PDF/PNG files (no composite). Explanatory text belongs in LaTeX.
Run from this directory: MPLCONFIGDIR=/tmp/mpl python3 state_action_design.py
"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
from matplotlib import gridspec, patches

from graphics_common import (
    PALETTE,
    ROBOT_RADIUS,
    add_robot,
    add_sector,
    add_threshold_arc,
    configure_matplotlib,
    format_action_value,
    load_d2_actions,
    load_d2_params,
    save_figure,
)


def _style_axis(ax, title: str = "") -> None:
    if title:
        ax.set_title(title, loc="left", pad=8, fontsize=11, weight="bold")
    ax.set_aspect("equal")
    ax.axis("off")


def _coarse_heading_ray(ax, angle_deg: float, length: float = 1.52) -> None:
    th = math.radians(angle_deg)
    ax.plot(
        [ROBOT_RADIUS * math.cos(th), length * math.cos(th)],
        [ROBOT_RADIUS * math.sin(th), length * math.sin(th)],
        color=PALETTE["amber"],
        linewidth=1.4,
        linestyle="--",
    )


def _coarse_candidate_diagram(ax) -> None:
    """18-state radar only (no legend boxes)."""
    _style_axis(ax, "")
    add_robot(ax)

    add_sector(ax, ROBOT_RADIUS, 2.45, 72, 108, PALETTE["blue"], label="front")
    add_sector(ax, ROBOT_RADIUS, 2.05, 5, 70, PALETTE["teal"], label=None)
    ax.text(
        1.82 * math.cos(math.radians(36.0)),
        1.82 * math.sin(math.radians(36.0)),
        "right\n(RF vs RR)",
        fontsize=8,
        color=PALETTE["ink"],
        weight="bold",
        ha="center",
        va="center",
    )

    add_threshold_arc(
        ax,
        0.55,
        5,
        70,
        "too close",
        label_angle=43.0,
        label_radial_offset=0.08,
        inner_connect=ROBOT_RADIUS,
    )
    add_threshold_arc(
        ax,
        0.95,
        5,
        70,
        "good",
        label_angle=49.0,
        label_radial_offset=0.10,
        inner_connect=ROBOT_RADIUS,
    )
    add_threshold_arc(
        ax,
        1.45,
        5,
        70,
        "too far",
        label_angle=55.0,
        label_radial_offset=0.11,
        inner_connect=ROBOT_RADIUS,
    )

    _coarse_heading_ray(ax, 37.5)
    _coarse_heading_ray(ax, -45.0)

    ax.set_xlim(-2.35, 2.85)
    ax.set_ylim(-2.0, 2.65)


def _final_state_diagram(ax, params) -> None:
    """72-state radar only (no legend boxes)."""
    _style_axis(ax, "")
    add_robot(ax)

    fl_s = float(params["front_left_open_sector_start_deg"])
    fl_e = float(params["front_left_open_sector_end_deg"])
    fr_s = float(params["front_right_open_sector_start_deg"])
    fr_e = float(params["front_right_open_sector_end_deg"])
    fs_s = float(params["front_sector_start_deg"])
    fs_e = float(params["front_sector_end_deg"])
    rf_s = float(params["right_front_sector_start_deg"])
    rf_e = float(params["right_front_sector_end_deg"])
    rr_s = float(params["right_rear_sector_start_deg"])
    rr_e = float(params["right_rear_sector_end_deg"])

    sectors = [
        ("front", fs_s, fs_e, PALETTE["blue"], 2.2, None),
        ("FL open", fl_s, fl_e, PALETTE["violet"], 2.75, 2.38),
        ("FR open", fr_s, fr_e, PALETTE["sage"], 2.75, 2.42),
        ("right-front", rf_s, rf_e, PALETTE["teal"], 2.05, 1.72),
        ("right-rear", rr_s, rr_e, PALETTE["amber"], 2.08, 1.78),
    ]
    for label, theta1, theta2, color, radius, label_radius in sectors:
        lr = label_radius if label_radius is not None else (radius - 0.35)
        add_sector(ax, ROBOT_RADIUS, radius, theta1, theta2, color, label=label, label_radius=lr)

    add_threshold_arc(
        ax,
        float(params["right_too_close"]),
        -70,
        70,
        "0.55 m",
        label_angle=-30,
        inner_connect=ROBOT_RADIUS,
        label_radial_offset=0.11,
    )
    add_threshold_arc(
        ax,
        float(params["right_too_far"]),
        -70,
        70,
        "0.95 m",
        y_offset=0.02,
        label_angle=-17,
        inner_connect=ROBOT_RADIUS,
        label_radial_offset=0.12,
    )
    add_threshold_arc(
        ax,
        float(params["opening_distance_threshold"]),
        int(fr_s),
        int(fl_e),
        "1.20 m",
        y_offset=0.05,
        label_angle=122,
        inner_connect=ROBOT_RADIUS,
        label_radial_offset=0.13,
    )

    ax.set_xlim(-2.65, 2.95)
    ax.set_ylim(-2.45, 3.0)


def _draw_path(ax, points, color, lw: float = 1.75):
    xs, ys = zip(*points)
    ax.plot(xs, ys, color=color, linewidth=lw, alpha=0.92, solid_capstyle="round", zorder=2)
    ax.scatter(xs, ys, s=22, color=color, zorder=4, edgecolors=PALETTE["ink"], linewidths=0.45)


def _draw_mini_robot(ax, x: float, y: float, scale: float = 1.0) -> None:
    robot = patches.Circle((x, y), radius=0.055 * scale, facecolor="white", edgecolor=PALETTE["ink"], linewidth=1.9, zorder=5)
    ax.add_patch(robot)
    ax.arrow(
        x,
        y + 0.04 * scale,
        0.0,
        0.17 * scale,
        width=0.0058 * scale,
        head_width=0.040 * scale,
        head_length=0.036 * scale,
        color=PALETTE["blue"],
        length_includes_head=True,
        zorder=5,
    )


def _draw_old_guess(ax, cx: float, cy: float, scale: float, kind: str) -> None:
    wrong = PALETTE["coral"]
    x0 = cx + 0.13 * scale
    y0 = cy - 0.05 * scale
    x1 = x0
    y1 = cy + 0.25 * scale
    ax.plot([x0, x1], [y0, y1], color=wrong, linewidth=2.0, linestyle=(0, (4, 3)), alpha=0.75, zorder=3)
    ax.arrow(
        x1,
        y1 - 0.025 * scale,
        0.0,
        0.001,
        head_width=0.030 * scale,
        head_length=0.030 * scale,
        color=wrong,
        alpha=0.75,
        length_includes_head=True,
        zorder=3,
    )
    ax.text(x0 + 0.005 * scale, y0 - 0.03 * scale, "old guess", fontsize=6.8, color=wrong, weight="bold", ha="center")

    if kind == "blocked_front_left":
        ax.text(x0, cy + 0.31 * scale, "X", fontsize=13, color=wrong, weight="bold", ha="center", va="center")


def _draw_action_hint(ax, kind: str, cx: float, cy: float, scale: float) -> None:
    good = PALETTE["sage"]

    _draw_old_guess(ax, cx, cy, scale, kind)

    if kind == "corridor":
        ax.arrow(
            cx - 0.11 * scale,
            cy - 0.05 * scale,
            0.0,
            0.28 * scale,
            width=0.005 * scale,
            head_width=0.034 * scale,
            head_length=0.034 * scale,
            color=good,
            length_includes_head=True,
            zorder=4,
        )
        ax.text(cx - 0.11 * scale, cy + 0.31 * scale, "should go\nstraight", fontsize=7.3, color=good, weight="bold", ha="center")
    elif kind == "uturn_mouth":
        arrow = patches.FancyArrowPatch(
            (cx - 0.04 * scale, cy + 0.16 * scale),
            (cx + 0.21 * scale, cy + 0.02 * scale),
            connectionstyle="arc3,rad=-0.55",
            arrowstyle="-|>",
            mutation_scale=18,
            linewidth=2.8,
            color=good,
            zorder=4,
        )
        ax.add_patch(arrow)
        ax.text(cx + 0.00 * scale, cy + 0.33 * scale, "should turn right", fontsize=7.0, color=good, weight="bold", ha="center")
    elif kind == "blocked_front_left":
        arrow = patches.FancyArrowPatch(
            (cx - 0.01 * scale, cy + 0.16 * scale),
            (cx - 0.24 * scale, cy + 0.07 * scale),
            connectionstyle="arc3,rad=0.50",
            arrowstyle="-|>",
            mutation_scale=18,
            linewidth=2.8,
            color=good,
            zorder=4,
        )
        ax.add_patch(arrow)
        ax.text(cx - 0.23 * scale, cy + 0.30 * scale, "should turn left", fontsize=7.0, color=good, weight="bold", ha="center")


def _draw_mini_geometry(ax, bx: float, by: float, w: float, h: float, kind: str) -> None:
    """Three easy-to-read local layouts that the old model could confuse."""
    ink = PALETTE["ink"]
    open_col = "#dbeafe"
    wall_lw = 2.1

    def wx(px: float) -> float:
        return bx + px * w

    def hy(py: float) -> float:
        return by + py * h

    robot_x, robot_y = wx(0.54), hy(0.26)

    if kind == "corridor":
        xl, xr = wx(0.12), wx(0.88)
        yb, yt = hy(0.12), hy(0.88)
        ax.plot([xl, xl], [yb, yt], color=ink, linewidth=wall_lw)
        ax.plot([xr, xr], [yb, yt], color=ink, linewidth=wall_lw)

    elif kind == "uturn_mouth":
        xl, xr = wx(0.12), wx(0.82)
        yb, yt = hy(0.12), hy(0.88)
        y_lip = hy(0.42)
        ax.plot([xl, xl], [yb, yt], color=ink, linewidth=wall_lw)
        ax.plot([xr, xr], [yb, y_lip], color=ink, linewidth=wall_lw)
        ax.plot([xr, wx(0.95)], [y_lip, y_lip], color=ink, linewidth=wall_lw)
        ax.add_patch(
            patches.Rectangle((wx(0.82), hy(0.18)), wx(0.16) - wx(0.0), hy(0.48) - hy(0.18), facecolor=open_col, edgecolor="none", alpha=0.75, zorder=1)
        )

    elif kind == "blocked_front_left":
        xl, xr = wx(0.11), wx(0.88)
        yb, yt = hy(0.12), hy(0.88)
        y_front = hy(0.63)
        ax.plot([xr, xr], [yb, yt], color=ink, linewidth=wall_lw)
        ax.plot([xl, xl], [yb, hy(0.42)], color=ink, linewidth=wall_lw)
        ax.plot([wx(0.28), xr], [y_front, y_front], color=ink, linewidth=wall_lw)
        ax.add_patch(
            patches.Rectangle((wx(0.00), hy(0.46)), wx(0.15) - wx(0.00), hy(0.68) - hy(0.46), facecolor=open_col, edgecolor="none", alpha=0.75, zorder=1)
        )

    _draw_action_hint(ax, kind, robot_x, robot_y, w)
    _draw_mini_robot(ax, robot_x, robot_y, scale=w)


def _aliasing_diagram(ax) -> None:
    """Three panels for 18-state aliasing.

    Keep the asset *purely visual*: no titles, no explanatory text, no coarse-state badge.
    LaTeX provides panel labels and the caption.
    """
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")

    cards = [
        (0.02, "corridor"),
        (0.35, "uturn_mouth"),
        (0.68, "blocked_front_left"),
    ]
    for x, kind in cards:
        card = patches.FancyBboxPatch(
            (x, 0.10),
            0.28,
            0.80,
            boxstyle="round,pad=0.016,rounding_size=0.02",
            linewidth=1.1,
            edgecolor=PALETTE["grid"],
            facecolor=PALETTE["panel"],
        )
        ax.add_patch(card)
        # Larger geometry region, more whitespace, no overlap with any labels.
        _draw_mini_geometry(ax, x + 0.03, 0.20, 0.22, 0.58, kind)


def _draw_action_glyph(ax, name: str, linear_y: float, angular_z: float, color: str) -> None:
    ax.set_aspect("equal")
    ax.axis("off")
    add_robot(ax, radius=0.11)

    arc_radius = 0.62 if abs(angular_z) > 0.0 else 0.0
    upward = linear_y * 4.0

    if linear_y > 0.0:
        ax.arrow(
            0.0,
            0.12,
            0.0,
            upward,
            width=0.02,
            head_width=0.12,
            head_length=0.13,
            length_includes_head=True,
            color=color,
            alpha=0.9,
        )

    if angular_z > 0.0:
        arc = patches.Arc((0.0, 0.05), 2 * arc_radius, 2 * arc_radius, theta1=70, theta2=168, color=color, linewidth=2.2)
        ax.add_patch(arc)
        ax.arrow(-0.58, 0.29, -0.06, 0.1, head_width=0.1, head_length=0.08, color=color, linewidth=0, length_includes_head=True)
    elif angular_z < 0.0:
        arc = patches.Arc((0.0, 0.05), 2 * arc_radius, 2 * arc_radius, theta1=12, theta2=110, color=color, linewidth=2.2)
        ax.add_patch(arc)
        ax.arrow(0.58, 0.29, 0.06, 0.1, head_width=0.1, head_length=0.08, color=color, linewidth=0, length_includes_head=True)

    label_map = {
        "turn_left_hard": "turn left\nhard",
        "turn_left_soft": "turn left\nsoft",
        "straight": "straight",
        "turn_right_soft": "turn right\nsoft",
        "turn_right_hard": "turn right\nhard",
    }
    ax.text(0.0, -0.56, label_map[name], ha="center", va="top", fontsize=8.8, weight="bold")
    ax.text(0.0, -0.94, format_action_value(linear_y, angular_z), ha="center", va="top", fontsize=7.5, color=PALETTE["muted"])
    ax.set_xlim(-1.05, 1.05)
    ax.set_ylim(-1.08, 1.3)


def _figure_state_18() -> plt.Figure:
    fig, ax = plt.subplots(figsize=(5.6, 5.0), layout="constrained")
    _coarse_candidate_diagram(ax)
    return fig


def _figure_state_72(params) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(5.8, 5.2), layout="constrained")
    _final_state_diagram(ax, params)
    return fig


def _figure_aliasing() -> plt.Figure:
    # Taller to keep the three panels legible in print.
    fig, ax = plt.subplots(figsize=(9.8, 3.35), layout="constrained")
    _aliasing_diagram(ax)
    return fig


def _figure_actions(actions) -> plt.Figure:
    fig = plt.figure(figsize=(11.5, 2.05), layout="constrained")
    gs = gridspec.GridSpec(1, 5, figure=fig, wspace=0.15)
    order = [
        ("turn_left_hard", PALETTE["amber"]),
        ("turn_left_soft", PALETTE["sage"]),
        ("straight", PALETTE["blue"]),
        ("turn_right_soft", PALETTE["teal"]),
        ("turn_right_hard", PALETTE["slate"]),
    ]
    for idx, (name, color) in enumerate(order):
        ax = fig.add_subplot(gs[0, idx])
        _draw_action_glyph(ax, name, actions[name]["linear_y"], actions[name]["angular_z"], color)
    return fig


def main() -> None:
    configure_matplotlib()
    params = load_d2_params()
    actions = load_d2_actions()

    fig = _figure_state_18()
    save_figure(fig, "state_diagram_18state")
    plt.close(fig)

    fig = _figure_state_72(params)
    save_figure(fig, "state_diagram_72state")
    plt.close(fig)

    fig = _figure_aliasing()
    save_figure(fig, "state_aliasing_cartoons")
    plt.close(fig)

    fig = _figure_actions(actions)
    save_figure(fig, "action_set_d2")
    plt.close(fig)


if __name__ == "__main__":
    main()
