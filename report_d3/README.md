# D3 Report Workspace

Use this folder after the Ubuntu experiments are complete.

Expected inputs:

- `../artifacts/house_metric_map.png`
- `../artifacts/house_likelihood_field.png`
- `../artifacts/pf_run.csv`
- `../artifacts/pf_metrics.png`
- screenshots from RViz/Gazebo showing particle convergence

Build the plot first:

```bash
python3 ../scripts/plot_pf_metrics.py \
  --csv ../artifacts/pf_run.csv \
  --output ../artifacts/pf_metrics.png
```

Then replace the TODOs in `main.tex` with the measured results and compile with
the IEEE conference class.
