#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACTS_DIR="${ROOT_DIR}/artifacts"
CONFIG_DIR="${ROOT_DIR}/config"
CHECKPOINTS_DIR="${ARTIFACTS_DIR}/checkpoints"
DISCARDED_DIR="${ARTIFACTS_DIR}/discarded"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/d2_state.sh status
  ./scripts/d2_state.sh list
  ./scripts/d2_state.sh save <label>
  ./scripts/d2_state.sh load <label>
  ./scripts/d2_state.sh discard current
  ./scripts/d2_state.sh adopt-legacy <q_learning|sarsa>
  ./scripts/d2_state.sh promote <q_learning|sarsa> <best|latest>

Notes:
  - save/load/discard operate on the live ignored D2 state in artifacts/.
  - load also migrates legacy config/*_latest.yaml checkpoints into artifacts/.
  - promote copies the chosen live artifact table into config/*best.yaml for submission.
EOF
}

require_label() {
  if [[ $# -lt 1 || -z ${1:-} ]]; then
    echo "Missing label." >&2
    usage >&2
    exit 64
  fi
}

artifact_latest_path() {
  case "$1" in
    q_learning) echo "${ARTIFACTS_DIR}/qtable_d2_qlearning_latest.yaml" ;;
    sarsa) echo "${ARTIFACTS_DIR}/qtable_d2_sarsa_latest.yaml" ;;
    *) return 1 ;;
  esac
}

artifact_best_path() {
  case "$1" in
    q_learning) echo "${ARTIFACTS_DIR}/qtable_d2_qlearning_best.yaml" ;;
    sarsa) echo "${ARTIFACTS_DIR}/qtable_d2_sarsa_best.yaml" ;;
    *) return 1 ;;
  esac
}

metrics_path() {
  case "$1" in
    q_learning) echo "${ARTIFACTS_DIR}/d2_qlearning_metrics.csv" ;;
    sarsa) echo "${ARTIFACTS_DIR}/d2_sarsa_metrics.csv" ;;
    *) return 1 ;;
  esac
}

legacy_config_latest_path() {
  case "$1" in
    q_learning) echo "${CONFIG_DIR}/qtable_d2_qlearning_latest.yaml" ;;
    sarsa) echo "${CONFIG_DIR}/qtable_d2_sarsa_latest.yaml" ;;
    *) return 1 ;;
  esac
}

config_best_path() {
  case "$1" in
    q_learning) echo "${CONFIG_DIR}/qtable_d2_qlearning_best.yaml" ;;
    sarsa) echo "${CONFIG_DIR}/qtable_d2_sarsa_best.yaml" ;;
    *) return 1 ;;
  esac
}

copy_if_exists() {
  local src="$1"
  local dest_dir="$2"
  [[ -f "${src}" ]] || return 0
  mkdir -p "${dest_dir}"
  cp -f "${src}" "${dest_dir}/"
}

move_if_exists() {
  local src="$1"
  local dest_dir="$2"
  [[ -f "${src}" ]] || return 0
  mkdir -p "${dest_dir}"
  mv -f "${src}" "${dest_dir}/"
}

yaml_summary() {
  local path="$1"
  python3 - "$path" <<'PY'
import sys, yaml
path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
except OSError as exc:
    print(f"unreadable ({exc})")
    raise SystemExit(0)
meta = data.get("metadata", {}) if isinstance(data, dict) else {}
algo = meta.get("algorithm", "?")
eps = meta.get("epsilon", "?")
completed = meta.get("completed_episodes", "?")
run_id = meta.get("run_id", "-")
saved = meta.get("last_saved_at_utc", "-")
print(f"algo={algo} completed={completed} epsilon={eps} run_id={run_id} saved={saved}")
PY
}

csv_summary() {
  local path="$1"
  python3 - "$path" <<'PY'
import csv, sys
path = sys.argv[1]
try:
    with open(path, "r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
except OSError as exc:
    print(f"unreadable ({exc})")
    raise SystemExit(0)
if not rows:
    print("empty")
    raise SystemExit(0)
last = rows[-1]
episodes = len(rows)
print(
    "rows={rows} last_episode={ep} last_reward={reward} last_steps={steps} last_epsilon={epsilon}".format(
        rows=episodes,
        ep=last.get("episode", "?"),
        reward=last.get("reward", "?"),
        steps=last.get("steps", "?"),
        epsilon=last.get("epsilon", "?"),
    )
)
PY
}

save_current_into() {
  local dest="$1"
  mkdir -p "${dest}/artifacts" "${dest}/config"

  copy_if_exists "${ARTIFACTS_DIR}/captured_start_poses.yaml" "${dest}/artifacts"
  copy_if_exists "${ARTIFACTS_DIR}/captured_named_poses.yaml" "${dest}/artifacts"

  for algo in q_learning sarsa; do
    copy_if_exists "$(artifact_latest_path "${algo}")" "${dest}/artifacts"
    copy_if_exists "$(artifact_best_path "${algo}")" "${dest}/artifacts"
    copy_if_exists "$(metrics_path "${algo}")" "${dest}/artifacts"
    copy_if_exists "$(legacy_config_latest_path "${algo}")" "${dest}/config"
    copy_if_exists "$(config_best_path "${algo}")" "${dest}/config"
  done
}

archive_current_state() {
  local reason="$1"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local dest="${DISCARDED_DIR}/${stamp}_${reason}"
  mkdir -p "${dest}/artifacts" "${dest}/config"

  [[ -f "${ARTIFACTS_DIR}/captured_start_poses.yaml" ]] && cp -f "${ARTIFACTS_DIR}/captured_start_poses.yaml" "${dest}/artifacts/"
  [[ -f "${ARTIFACTS_DIR}/captured_named_poses.yaml" ]] && cp -f "${ARTIFACTS_DIR}/captured_named_poses.yaml" "${dest}/artifacts/"

  for algo in q_learning sarsa; do
    move_if_exists "$(artifact_latest_path "${algo}")" "${dest}/artifacts"
    move_if_exists "$(artifact_best_path "${algo}")" "${dest}/artifacts"
    move_if_exists "$(metrics_path "${algo}")" "${dest}/artifacts"
    move_if_exists "$(legacy_config_latest_path "${algo}")" "${dest}/config"
  done

  echo "${dest}"
}

save_checkpoint() {
  local label="$1"
  local dest="${CHECKPOINTS_DIR}/${label}"
  if [[ -e "${dest}" ]]; then
    echo "Checkpoint already exists: ${dest}" >&2
    exit 65
  fi
  save_current_into "${dest}"
  echo "Saved checkpoint: ${dest}"
}

load_checkpoint() {
  local label="$1"
  local src="${CHECKPOINTS_DIR}/${label}"
  [[ -d "${src}" ]] || { echo "Checkpoint not found: ${src}" >&2; exit 66; }

  local archived
  archived="$(archive_current_state before_load)"
  echo "Archived current live state to: ${archived}"

  mkdir -p "${ARTIFACTS_DIR}"

  for algo in q_learning sarsa; do
    if [[ -f "${src}/artifacts/$(basename "$(artifact_latest_path "${algo}")")" ]]; then
      cp -f "${src}/artifacts/$(basename "$(artifact_latest_path "${algo}")")" "$(artifact_latest_path "${algo}")"
    elif [[ -f "${src}/config/$(basename "$(legacy_config_latest_path "${algo}")")" ]]; then
      cp -f "${src}/config/$(basename "$(legacy_config_latest_path "${algo}")")" "$(artifact_latest_path "${algo}")"
    fi

    if [[ -f "${src}/artifacts/$(basename "$(artifact_best_path "${algo}")")" ]]; then
      cp -f "${src}/artifacts/$(basename "$(artifact_best_path "${algo}")")" "$(artifact_best_path "${algo}")"
    elif [[ -f "${src}/config/$(basename "$(config_best_path "${algo}")")" ]]; then
      cp -f "${src}/config/$(basename "$(config_best_path "${algo}")")" "$(artifact_best_path "${algo}")"
    fi

    if [[ -f "${src}/artifacts/$(basename "$(metrics_path "${algo}")")" ]]; then
      cp -f "${src}/artifacts/$(basename "$(metrics_path "${algo}")")" "$(metrics_path "${algo}")"
    fi
  done

  if [[ -f "${src}/artifacts/captured_start_poses.yaml" ]]; then
    cp -f "${src}/artifacts/captured_start_poses.yaml" "${ARTIFACTS_DIR}/captured_start_poses.yaml"
  fi
  if [[ -f "${src}/artifacts/captured_named_poses.yaml" ]]; then
    cp -f "${src}/artifacts/captured_named_poses.yaml" "${ARTIFACTS_DIR}/captured_named_poses.yaml"
  fi

  echo "Loaded checkpoint: ${src}"
}

discard_current() {
  local archived
  archived="$(archive_current_state discarded)"
  echo "Discarded current live state into: ${archived}"
}

promote_live_policy() {
  local algo="$1"
  local source_kind="$2"
  local src
  local dest

  case "${source_kind}" in
    latest) src="$(artifact_latest_path "${algo}")" ;;
    best) src="$(artifact_best_path "${algo}")" ;;
    *) echo "Unknown source kind: ${source_kind}" >&2; exit 67 ;;
  esac

  dest="$(config_best_path "${algo}")"
  [[ -f "${src}" ]] || { echo "Source table not found: ${src}" >&2; exit 68; }
  cp -f "${src}" "${dest}"
  echo "Promoted ${src} -> ${dest}"
}

adopt_legacy_checkpoint() {
  local algo="$1"
  local legacy_latest legacy_best
  legacy_latest="$(legacy_config_latest_path "${algo}")"
  legacy_best="$(config_best_path "${algo}")"

  [[ -f "${legacy_latest}" ]] || {
    echo "Legacy checkpoint not found: ${legacy_latest}" >&2
    exit 69
  }

  local stamp dest
  stamp="$(date +%Y%m%d_%H%M%S)"
  dest="${DISCARDED_DIR}/${stamp}_adopt_legacy_${algo}/artifacts"
  mkdir -p "${dest}"

  move_if_exists "$(artifact_latest_path "${algo}")" "${dest}"
  move_if_exists "$(artifact_best_path "${algo}")" "${dest}"

  cp -f "${legacy_latest}" "$(artifact_latest_path "${algo}")"
  if [[ -f "${legacy_best}" ]]; then
    cp -f "${legacy_best}" "$(artifact_best_path "${algo}")"
  fi

  echo "Adopted legacy config checkpoint for ${algo} into live artifacts."
}

status() {
  echo "Repo: ${ROOT_DIR}"
  echo

  for algo in q_learning sarsa; do
    echo "[${algo}]"

    local live_latest live_best live_metrics legacy_latest submitted_best
    live_latest="$(artifact_latest_path "${algo}")"
    live_best="$(artifact_best_path "${algo}")"
    live_metrics="$(metrics_path "${algo}")"
    legacy_latest="$(legacy_config_latest_path "${algo}")"
    submitted_best="$(config_best_path "${algo}")"

    if [[ -f "${live_latest}" ]]; then
      echo "  artifacts latest : $(yaml_summary "${live_latest}")"
    else
      echo "  artifacts latest : missing"
    fi

    if [[ -f "${live_best}" ]]; then
      echo "  artifacts best   : $(yaml_summary "${live_best}")"
    else
      echo "  artifacts best   : missing"
    fi

    if [[ -f "${live_metrics}" ]]; then
      echo "  metrics          : $(csv_summary "${live_metrics}")"
    else
      echo "  metrics          : missing"
    fi

    if [[ -f "${legacy_latest}" ]]; then
      echo "  legacy config latest : $(yaml_summary "${legacy_latest}")"
    else
      echo "  legacy config latest : missing"
    fi

    if [[ -f "${submitted_best}" ]]; then
      echo "  submitted config best: $(yaml_summary "${submitted_best}")"
    else
      echo "  submitted config best: missing"
    fi
    echo
  done

  echo "Checkpoints:"
  mkdir -p "${CHECKPOINTS_DIR}"
  find "${CHECKPOINTS_DIR}" -mindepth 1 -maxdepth 1 -type d -print 2>/dev/null | sort || true
  echo
  echo "Discarded snapshots:"
  mkdir -p "${DISCARDED_DIR}"
  find "${DISCARDED_DIR}" -mindepth 1 -maxdepth 1 -type d -print 2>/dev/null | sort || true
}

list_checkpoints() {
  mkdir -p "${CHECKPOINTS_DIR}"
  find "${CHECKPOINTS_DIR}" -mindepth 1 -maxdepth 1 -type d -print | sort
}

main() {
  local command="${1:-}"
  case "${command}" in
    status)
      status
      ;;
    list)
      list_checkpoints
      ;;
    save)
      shift
      require_label "$@"
      save_checkpoint "$1"
      ;;
    load)
      shift
      require_label "$@"
      load_checkpoint "$1"
      ;;
    discard)
      shift
      if [[ ${1:-} != "current" ]]; then
        echo "Usage: ./scripts/d2_state.sh discard current" >&2
        exit 64
      fi
      discard_current
      ;;
    promote)
      shift
      if [[ $# -ne 2 ]]; then
        echo "Usage: ./scripts/d2_state.sh promote <q_learning|sarsa> <best|latest>" >&2
        exit 64
      fi
      promote_live_policy "$1" "$2"
      ;;
    adopt-legacy)
      shift
      if [[ $# -ne 1 ]]; then
        echo "Usage: ./scripts/d2_state.sh adopt-legacy <q_learning|sarsa>" >&2
        exit 64
      fi
      adopt_legacy_checkpoint "$1"
      ;;
    ""|-h|--help|help)
      usage
      ;;
    *)
      echo "Unknown command: ${command}" >&2
      usage >&2
      exit 64
      ;;
  esac
}

main "$@"
