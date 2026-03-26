#!/usr/bin/env python3
"""Text-only lessons table for the D3 report."""

from __future__ import annotations

import textwrap

import matplotlib.pyplot as plt
from matplotlib import patches

from graphics_common import PALETTE, configure_matplotlib, save_figure


def _card(ax, title: str, body: str, edge: str, *, wrap_width: int = 34, face_alpha: float = 0.14) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")
    card = patches.FancyBboxPatch(
        (0.05, 0.08),
        0.90,
        0.84,
        boxstyle="round,pad=0.02,rounding_size=0.028",
        linewidth=1.35,
        edgecolor=edge,
        facecolor=edge,
        alpha=face_alpha,
    )
    ax.add_patch(card)
    ax.text(0.10, 0.86, title, ha="left", va="top", fontsize=9.4, weight="bold", color=PALETTE["ink"])
    ax.text(
        0.10,
        0.72,
        textwrap.fill(body, width=wrap_width),
        ha="left",
        va="top",
        fontsize=8.0,
        color=PALETTE["ink"],
        linespacing=1.25,
    )


def _type_pill(ax, label: str, edge: str) -> None:
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)
    ax.axis("off")
    pill = patches.FancyBboxPatch(
        (0.08, 0.36),
        0.84,
        0.28,
        boxstyle="round,pad=0.02,rounding_size=0.12",
        linewidth=1.2,
        edgecolor=edge,
        facecolor=edge,
        alpha=0.16,
    )
    ax.add_patch(pill)
    ax.text(0.50, 0.50, label, ha="center", va="center", fontsize=9.2, weight="bold", color=PALETTE["ink"])


def main() -> None:
    configure_matplotlib()

    rows = [
        (
            "STATE MODEL",
            PALETTE["blue"],
            "The old 18-state view hid left-open, right-open, and blocked-front geometry.",
            "Straight hall, U-turn mouth, and blocked-front recovery could all look like the same simple state.",
            "Add front-left-open and front-right-open bits. That expands the model to 72 states.",
        ),
        (
            "REWARD MODEL",
            PALETTE["amber"],
            "Going straight still gave points after the robot lost the right wall.",
            "The robot skipped the U-turn and drifted into open space instead of reattaching to the wall.",
            "Make straight lose points in too-far states.",
        ),
        (
            "REWARD MODEL",
            PALETTE["amber"],
            "Soft-right and hard-right turns looked almost equally good in U-turn recovery.",
            "SARSA made a huge right sweep instead of taking the tighter turn through the mouth.",
            "Reward turn_right_soft more than turn_right_hard in that situation.",
        ),
        (
            "ACTION MODEL",
            PALETTE["teal"],
            "Hard turns had zero forward speed, so they could pivot in place forever.",
            "The robot got stuck in one spot even though there was no literal stop action.",
            "Keep the training action set, but add a SARSA-only test escape for that local loop.",
        ),
        (
            "TEST SETUP",
            PALETTE["violet"],
            "The test-start command could run before Triton existed in Gazebo.",
            "A good policy could start in the wrong place and look bad for the wrong reason.",
            "Wait for the model first, then apply the requested start pose.",
        ),
    ]

    fig = plt.figure(figsize=(13.8, 8.8))
    gs = fig.add_gridspec(
        len(rows),
        4,
        width_ratios=[0.95, 2.45, 2.65, 2.75],
        wspace=0.06,
        hspace=0.24,
    )

    first_axes = None

    for idx, (kind, kind_color, wrong_body, robot_body, fix_body) in enumerate(rows):
        ax_t = fig.add_subplot(gs[idx, 0])
        ax_w = fig.add_subplot(gs[idx, 1])
        ax_r = fig.add_subplot(gs[idx, 2])
        ax_f = fig.add_subplot(gs[idx, 3])

        _type_pill(ax_t, kind, kind_color)
        _card(ax_w, "What was wrong", wrong_body, PALETTE["amber"], wrap_width=32, face_alpha=0.11)
        _card(ax_r, "What the robot did", robot_body, PALETTE["coral"], wrap_width=34, face_alpha=0.10)
        _card(ax_f, "How we fixed it", fix_body, PALETTE["green"], wrap_width=35, face_alpha=0.11)

        if idx == 0:
            first_axes = (ax_t, ax_w, ax_r, ax_f)

    if first_axes:
        for label, ax in zip(("Type", "What was wrong", "What the robot did", "Final fix"), first_axes):
            pos = ax.get_position()
            fig.text(pos.x0 + pos.width / 2.0, pos.y1 + 0.016, label, ha="center", va="bottom", fontsize=11.0, weight="bold")

    fig.suptitle(
        "Problem-structuring lessons from the D2 iterations",
        x=0.5,
        y=0.99,
        fontsize=13.2,
        weight="bold",
        color=PALETTE["ink"],
    )

    save_figure(fig, "failure_lessons_flow")
    plt.close(fig)


if __name__ == "__main__":
    main()
