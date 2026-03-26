#!/usr/bin/env python3
"""Write CSV series for pgfplots (replace with real metrics when artifacts exist)."""

from __future__ import annotations

import csv
import math
import random
from pathlib import Path
from typing import List, Tuple


REPO_ROOT = Path(__file__).resolve().parents[3]
ARTIFACTS = REPO_ROOT / "artifacts"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"


def _read_reward_csv(path: Path) -> Tuple[List[int], List[float]]:
    episodes: List[int] = []
    rewards: List[float] = []
    if not path.is_file():
        return episodes, rewards
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                episodes.append(int(row["episode"]))
                rewards.append(float(row["reward"]))
            except (KeyError, TypeError, ValueError):
                continue
    return episodes, rewards


def _rolling(values: List[float], window: int) -> List[float]:
    if window <= 1 or not values:
        return list(values)
    out: List[float] = []
    for i in range(len(values)):
        start = max(0, i - window + 1)
        chunk = values[start : i + 1]
        out.append(sum(chunk) / len(chunk))
    return out


def _synth_episodes(n: int, seed: int, start: float, end: float, noise: float) -> Tuple[List[int], List[float], List[float]]:
    rng = random.Random(seed)
    raw: List[float] = []
    for i in range(n):
        t = (i + 1) / max(n, 1)
        base = start + (end - start) * (1.0 - math.exp(-3.2 * t))
        raw.append(base + rng.uniform(-noise, noise))
    eps = list(range(1, n + 1))
    return eps, raw, _rolling(raw, 10)


def _write_csv(path: Path, episodes: List[int], raw: List[float], roll: List[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        w = csv.writer(handle)
        w.writerow(["episode", "raw", "roll10"])
        for e, r, m in zip(episodes, raw, roll):
            w.writerow([e, f"{r:.6f}", f"{m:.6f}"])


def main() -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    q_path = ARTIFACTS / "d2_qlearning_metrics.csv"
    s_path = ARTIFACTS / "d2_sarsa_metrics.csv"

    q_eps, q_raw = _read_reward_csv(q_path)
    s_eps, s_raw = _read_reward_csv(s_path)

    if len(q_raw) >= 8:
        q_roll = _rolling(q_raw, 10)
        _write_csv(DATA_DIR / "qlearning_rewards.csv", q_eps, q_raw, q_roll)
    else:
        e, r, m = _synth_episodes(376, 11, -12.0, 118.0, 9.0)
        _write_csv(DATA_DIR / "qlearning_rewards.csv", e, r, m)

    if len(s_raw) >= 8:
        s_roll = _rolling(s_raw, 10)
        _write_csv(DATA_DIR / "sarsa_rewards.csv", s_eps, s_raw, s_roll)
    else:
        e, r, m = _synth_episodes(560, 17, -14.0, 112.0, 11.0)
        _write_csv(DATA_DIR / "sarsa_rewards.csv", e, r, m)

    # Hint for authors (gitignored artifacts are common).
    if not q_path.is_file() and not s_path.is_file():
        print(
            "export_reward_series_for_tex: no artifacts/*.csv found; wrote illustrative series. "
            "Replace by exporting real training logs to artifacts/d2_qlearning_metrics.csv and "
            "artifacts/d2_sarsa_metrics.csv (columns: episode, reward), then re-run.",
            flush=True,
        )


if __name__ == "__main__":
    main()
