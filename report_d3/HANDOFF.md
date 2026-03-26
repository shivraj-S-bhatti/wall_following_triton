# Deliverable 3 Handoff

This workspace is a compile-now draft for the D3 report. The LaTeX source is in `main.tex` and uses a safe two-column fallback layout so it can build even without the official IEEE conference class.

## Build

```bash
cd /Users/apple/Downloads/robotics/wall_following_triton/report_d3
./build.sh
```

If you later obtain the official course template, the swap point is the document class line at the top of `main.tex`.

## What Is Already In Place

- quantitative reward figure copied into `figures/d2_reward_vs_episode.png`
- explanatory vector figures generated into `figures/generated/`
- LaTeX draft with abstract, intro, approach, experiments, comparison, and conclusion
- bibliography entries for the core RL references used in the paper
- placeholder qualitative figure slots that compile even before screenshots are added

## Tasks Still For You

- choose the final five screenshots from the Q-learning and SARSA demo videos
- place those screenshots in `figures/qualitative/` using the filenames referenced in `main.tex`
- if the final submission must match the official conference look exactly, drop in the IEEE/ieeeconf class or template files and update the document class line
- run a final proofread and rename the exported PDF to `P2D3_firstname_lastname.pdf`

## Notes

The report draft deliberately keeps the write-up technical and aligned with the repository facts:

- 18-state to 72-state encoder transition
- state/action/reward definitions
- Q-table loading and initialization behavior
- Q-learning vs SARSA comparison
- SARSA 270-degree right-turn pathology
- zero-linear-motion deadlock from hard turns
- spawn race and test-only deadlock breaker
