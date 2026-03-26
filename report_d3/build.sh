#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

mkdir -p /tmp/mpl
MPLCONFIGDIR=/tmp/mpl python3 figures/src/state_action_design.py
# failure_lessons_flow.py optional (paper uses tab:failure_lessons in main.tex)
python3 figures/src/build_qualitative_composite.py

pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex

echo "Built report_d3/main.pdf"
