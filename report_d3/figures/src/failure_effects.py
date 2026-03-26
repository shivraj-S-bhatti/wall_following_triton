#!/usr/bin/env python3
"""Generate the failure-modes figure for the D3 report."""

from __future__ import annotations

import textwrap

import matplotlib.pyplot as plt
from matplotlib import patches

from graphics_common import PALETTE, configure_matplotlib, save_figure


def _prep_icon_axis(ax) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.set_aspect("equal")
    ax.axis("off")


def _icon_state_alias(ax) -> None:
    _prep_icon_axis(ax)
    ax.text(0.50, 0.79, "same old state", ha="center", va="center", fontsize=8.2, color=PALETTE["red"], weight="bold")
    for x in (0.25, 0.50, 0.75):
        ax.add_patch(patches.Circle((x, 0.40), 0.08, facecolor="white", edgecolor=PALETTE["ink"], linewidth=1.4))
    ax.text(0.25, 0.18, "hall", ha="center", va="center", fontsize=7.6, color=PALETTE["muted"])
    ax.text(0.50, 0.18, "U-turn", ha="center", va="center", fontsize=7.6, color=PALETTE["muted"])
    ax.text(0.75, 0.18, "wall", ha="center", va="center", fontsize=7.6, color=PALETTE["muted"])


def _icon_drift(ax) -> None:
    _prep_icon_axis(ax)
    ax.plot([0.68, 0.68], [0.18, 0.82], color=PALETTE["ink"], linewidth=1.4)
    ax.plot([0.90, 0.90], [0.18, 0.82], color=PALETTE["ink"], linewidth=1.4)
    ax.plot([0.68, 0.90], [0.18, 0.18], color=PALETTE["ink"], linewidth=1.4)
    xs = [0.10, 0.26, 0.42, 0.55, 0.63, 0.69]
    ys = [0.24, 0.24, 0.24, 0.31, 0.44, 0.72]
    ax.plot(xs, ys, color=PALETTE["coral"], linewidth=1.9)
    ax.scatter(xs, ys, s=14, color=PALETTE["coral"], zorder=3)


def _icon_sweep(ax) -> None:
    _prep_icon_axis(ax)
    ax.plot([0.18, 0.18], [0.18, 0.82], color=PALETTE["ink"], linewidth=1.4)
    theta = [0.16, 0.30, 0.44, 0.58, 0.72, 0.86, 1.00]
    xs = [0.18 + 0.54 * t for t in theta]
    ys = [0.20 + 0.55 * (1 - (t - 0.58) ** 2) for t in theta]
    ax.plot(xs, ys, color=PALETTE["coral"], linewidth=1.9)
    ax.scatter(xs, ys, s=14, color=PALETTE["coral"], zorder=3)


def _icon_pivot(ax) -> None:
    _prep_icon_axis(ax)
    cx, cy = 0.50, 0.50
    ax.add_patch(patches.Circle((cx, cy), radius=0.055, facecolor="white", edgecolor=PALETTE["ink"], linewidth=1.3))
    ax.add_patch(patches.Arc((cx, cy), 0.55, 0.48, theta1=120, theta2=250, linewidth=1.8, color=PALETTE["amber"]))
    ax.add_patch(patches.Arc((cx, cy), 0.55, 0.48, theta1=-70, theta2=60, linewidth=1.8, color=PALETTE["coral"]))
    ax.arrow(0.27, 0.60, -0.01, 0.0, head_width=0.04, head_length=0.04, color=PALETTE["amber"], length_includes_head=True)
    ax.arrow(0.73, 0.40, 0.01, 0.0, head_width=0.04, head_length=0.04, color=PALETTE["coral"], length_includes_head=True)


def _icon_spawn(ax) -> None:
    _prep_icon_axis(ax)
    target = (0.30, 0.70)
    actual = (0.72, 0.28)
    ax.scatter([target[0]], [target[1]], marker="x", s=45, linewidths=1.6, color=PALETTE["green"])
    ax.scatter([actual[0]], [actual[1]], marker="o", s=20, color=PALETTE["coral"])
    ax.annotate(
        "",
        xy=actual,
        xytext=target,
        arrowprops=dict(arrowstyle="-|>", color=PALETTE["muted"], lw=1.4, linestyle="--", mutation_scale=10),
    )
    ax.text(target[0], target[1] + 0.10, "requested", ha="center", va="bottom", fontsize=7.2, color=PALETTE["green"])
    ax.text(actual[0], actual[1] - 0.10, "actual", ha="center", va="top", fontsize=7.2, color=PALETTE["coral"])


ICONS = {
    "alias": _icon_state_alias,
    "drift": _icon_drift,
    "sweep": _icon_sweep,
    "pivot": _icon_pivot,
    "spawn": _icon_spawn,
}


def _gap_arrow(ax) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")
    ax.annotate(
        "",
        xy=(0.90, 0.50),
        xytext=(0.10, 0.50),
        arrowprops=dict(arrowstyle="-|>", color=PALETTE["muted"], lw=1.5, mutation_scale=11),
    )


def _draw_type(ax, label: str, color: str) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")
    pill = patches.FancyBboxPatch(
        (0.15, 0.35),
        0.70,
        0.30,
        boxstyle="round,pad=0.02,rounding_size=0.12",
        linewidth=1.4,
        edgecolor=color,
        facecolor=color,
        alpha=0.14,
    )
    ax.add_patch(pill)
    ax.text(0.50, 0.50, label, ha="center", va="center", fontsize=10.0, weight="bold", color=PALETTE["ink"])


def _draw_card(ax, title: str, body: str, tint: str, *, wrap_width: int = 38) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")
    card = patches.FancyBboxPatch(
        (0.04, 0.08),
        0.92,
        0.84,
        boxstyle="round,pad=0.012,rounding_size=0.03",
        linewidth=1.2,
        edgecolor=tint,
        facecolor=tint,
        alpha=0.12,
    )
    ax.add_patch(card)
    ax.text(0.08, 0.83, title, transform=ax.transAxes, ha="left", va="top", fontsize=9.6, weight="bold")
    ax.text(
        0.08,
        0.64,
        textwrap.fill(body, width=wrap_width),
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=8.15,
        color=PALETTE["ink"],
        linespacing=1.25,
    )


def main() -> None:
    configure_matplotlib()

    fig = plt.figure(figsize=(15.2, 10.2))
    gs = fig.add_gridspec(
        5,
        6,
        width_ratios=[1.0, 2.55, 0.14, 1.85, 0.14, 2.70],
        wspace=0.0,
        hspace=0.18,
    )

    rows = [
        (
            "STATE MODEL",
            PALETTE["blue"],
            "The old 18-state view hid left-open, right-open, and blocked-front geometry.",
            "Different places looked like the same simple hallway state.",
            "Add front-left-open and front-right-open bits to make the 72-state encoder.",
            "alias",
        ),
        (
            "REWARD MODEL",
            PALETTE["amber"],
            "Going straight still gave points after the robot lost the right wall.",
            "The robot skipped the U-turn and drifted into open space.",
            "Make straight lose points in too-far states.",
            "drift",
        ),
        (
            "REWARD MODEL",
            PALETTE["amber"],
            "Soft-right and hard-right turns looked almost equally good in U-turn recovery.",
            "SARSA made a giant right sweep instead of a clean turn around the mouth.",
            "Reward turn_right_soft more than turn_right_hard there.",
            "sweep",
        ),
        (
            "ACTION MODEL",
            PALETTE["teal"],
            "Hard turns had zero forward speed, so they could pivot in place forever.",
            "The robot got stuck in one spot without needing a stop action.",
            "Keep training actions, but add a SARSA-only test escape for that loop.",
            "pivot",
        ),
        (
            "TEST SETUP",
            PALETTE["violet"],
            "The test-start command could run before Triton existed in Gazebo.",
            "A good policy could start in the wrong place and look bad.",
            "Wait for the model first, then apply the requested start pose.",
            "spawn",
        ),
    ]

    first_row_axes = None
    for idx, (kind, kind_color, cause_body, effect_body, fix_body, icon) in enumerate(rows):
        ax_type = fig.add_subplot(gs[idx, 0])
        ax_cause = fig.add_subplot(gs[idx, 1])
        ax_gap1 = fig.add_subplot(gs[idx, 2])
        ax_effect = fig.add_subplot(gs[idx, 3])
        ax_gap2 = fig.add_subplot(gs[idx, 4])
        ax_fix = fig.add_subplot(gs[idx, 5])

        _draw_type(ax_type, kind, kind_color)
        _draw_card(ax_cause, "What was wrong", cause_body, PALETTE["amber"], wrap_width=39)
        _draw_card(ax_effect, "What the robot did", effect_body, PALETTE["coral"], wrap_width=24)
        _draw_card(ax_fix, "How we fixed it", fix_body, PALETTE["green"], wrap_width=37)

        icon_ax = ax_effect.inset_axes([0.60, 0.08, 0.28, 0.34])
        ICONS[icon](icon_ax)
        _gap_arrow(ax_gap1)
        _gap_arrow(ax_gap2)

        if idx == 0:
            first_row_axes = (ax_type, ax_cause, ax_effect, ax_fix)

    if first_row_axes is not None:
        headers = [
            ("Type", first_row_axes[0]),
            ("What was wrong", first_row_axes[1]),
            ("What the robot did", first_row_axes[2]),
            ("Final fix", first_row_axes[3]),
        ]
        for label, ax in headers:
            pos = ax.get_position()
            fig.text(pos.x0 + pos.width / 2.0, pos.y1 + 0.012, label, ha="center", va="bottom", fontsize=11.8, weight="bold")

    fig.suptitle(
        "Problem-structuring lessons: which kind of mistake caused which failure",
        x=0.5,
        y=0.99,
        fontsize=14,
        weight="bold",
    )

    save_figure(fig, "failure_effects")
    plt.close(fig)


if __name__ == "__main__":
    main()
