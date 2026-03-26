# Figures

This folder holds the assets for the D3 report draft.

Tracked quantitative figure:

- `d2_reward_vs_episode.png`
- `generated/state_diagram_18state.pdf` / `.png`
- `generated/state_diagram_72state.pdf` / `.png`
- `generated/state_aliasing_cartoons.pdf` / `.png`
- `generated/action_set_d2.pdf` / `.png`
- `generated/failure_effects.pdf`
- `generated/failure_effects.png`

Qualitative screenshot slots expected by `main.tex`:

- `qualitative/ibeam_afterimages.png`
- `qualitative/uturn_deadend_afterimages.png`
- `qualitative/final_3_turns_afterimages.png`
- `qualitative/qualitative_afterimages_composite.png`

These three afterimage composites now replace the earlier placeholder-only
layout. Together they cover the five required qualitative scenarios.

Generation pipeline:

- `src/graphics_common.py`: shared palette/export helpers
- `src/state_action_design.py`: separate radar / aliasing / action PNG+PDF assets for `main.tex`
- `src/failure_effects.py`: modeling choices to observed failures to fixes
- `src/build_qualitative_composite.py`: compact qualitative panel for the report
