#!/usr/bin/env python3
"""Plot D2 training metrics with rolling reward and evaluation score."""

import argparse
import csv
import os
from typing import List, Tuple

import matplotlib.pyplot as plt


def _read_reward_csv(path: str) -> Tuple[List[int], List[float]]:
    episodes: List[int] = []
    rewards: List[float] = []
    if not os.path.exists(path):
        return episodes, rewards
    with open(path, "r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            try:
                episodes.append(int(row["episode"]))
                rewards.append(float(row["reward"]))
            except (KeyError, TypeError, ValueError):
                continue
    return episodes, rewards


def _read_eval_csv(path: str) -> Tuple[List[int], List[float]]:
    episodes: List[int] = []
    scores: List[float] = []
    if not os.path.exists(path):
        return episodes, scores
    with open(path, "r", encoding="utf-8", newline="") as f:
        for row in csv.DictReader(f):
            try:
                episodes.append(int(row["completed_episodes"]))
                scores.append(float(row["score"]))
            except (KeyError, TypeError, ValueError):
                continue
    return episodes, scores


def _rolling_mean(values: List[float], window: int) -> List[float]:
    if window <= 1 or not values:
        return list(values)
    output: List[float] = []
    for idx in range(len(values)):
        start = max(0, idx - window + 1)
        chunk = values[start : idx + 1]
        output.append(sum(chunk) / float(len(chunk)))
    return output


def _plot_algorithm(ax_reward, ax_eval, label: str, reward_csv: str, eval_csv: str, window: int):
    reward_eps, reward_values = _read_reward_csv(reward_csv)
    eval_eps, eval_values = _read_eval_csv(eval_csv)

    if reward_eps:
        ax_reward.plot(reward_eps, reward_values, alpha=0.25, label=f"{label} raw")
        ax_reward.plot(reward_eps, _rolling_mean(reward_values, window), linewidth=2.0, label=f"{label} mean")

    if eval_eps:
        ax_eval.plot(eval_eps, eval_values, marker="o", linewidth=2.0, label=label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q-reward", default="artifacts/d2_qlearning_metrics.csv")
    parser.add_argument("--q-eval", default="artifacts/d2_qlearning_eval.csv")
    parser.add_argument("--s-reward", default="artifacts/d2_sarsa_metrics.csv")
    parser.add_argument("--s-eval", default="artifacts/d2_sarsa_eval.csv")
    parser.add_argument("--window", type=int, default=10)
    parser.add_argument("--output", default="artifacts/d2_converge_plots.png")
    args = parser.parse_args()

    fig, (ax_reward, ax_eval) = plt.subplots(2, 1, figsize=(10, 8), sharex=False)

    _plot_algorithm(ax_reward, ax_eval, "Q-learning", args.q_reward, args.q_eval, args.window)
    _plot_algorithm(ax_reward, ax_eval, "SARSA", args.s_reward, args.s_eval, args.window)

    ax_reward.set_title(f"D2 Reward (raw + {args.window}-episode rolling mean)")
    ax_reward.set_xlabel("Episode")
    ax_reward.set_ylabel("Reward")
    ax_reward.grid(True, alpha=0.3)
    ax_reward.legend()

    ax_eval.set_title("D2 Greedy Evaluation Score")
    ax_eval.set_xlabel("Completed Episodes")
    ax_eval.set_ylabel("Evaluation Score")
    ax_eval.grid(True, alpha=0.3)
    ax_eval.legend()

    fig.tight_layout()
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    fig.savefig(args.output, dpi=160)
    print(f"saved {args.output}")


if __name__ == "__main__":
    main()
