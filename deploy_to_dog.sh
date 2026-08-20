#!/usr/bin/env bash
# Differential source deployment from this PC to the Odin1 dog workspace.
# Defaults are deliberately preview-only; set DRY_RUN=0 to make changes.
set -euo pipefail

DOG_USER="${DOG_USER:-firefly}"
DOG_IP="${DOG_IP:-192.168.0.109}"
LOCAL_WS="${LOCAL_WS:-/home/robot/project/odin1_jie_nav_ws}"
REMOTE_WS="${REMOTE_WS:-/home/firefly/odin1_jie_nav_ws}"

DRY_RUN="${DRY_RUN:-1}"
BUILD="${BUILD:-0}"
DELETE="${DELETE:-0}"
ALLOW_DIRTY="${ALLOW_DIRTY:-0}"
BUILD_PACKAGES="${BUILD_PACKAGES:-jie_map_msgs jie_octomap octo_planner}"

for flag_name in DRY_RUN BUILD DELETE ALLOW_DIRTY; do
  flag_value="${!flag_name}"
  case "$flag_value" in
    0|1) ;;
    *)
      printf 'ERROR: %s must be 0 or 1 (got %q).\n' "$flag_name" "$flag_value" >&2
      exit 2
      ;;
  esac
done

if [[ ! -d "$LOCAL_WS/src" ]]; then
  printf 'ERROR: local source directory does not exist: %s\n' "$LOCAL_WS/src" >&2
  exit 1
fi

for required_command in git ssh rsync; do
  if ! command -v "$required_command" >/dev/null 2>&1; then
    printf 'ERROR: required local command is unavailable: %s\n' "$required_command" >&2
    exit 1
  fi
done

GIT_BRANCH="$(git -C "$LOCAL_WS" branch --show-current)"
GIT_SHA="$(git -C "$LOCAL_WS" rev-parse HEAD)"
GIT_STATUS="$(git -C "$LOCAL_WS" status --short)"

printf '%s\n' 'Local Git preflight:'
printf '  git branch --show-current: %s\n' "$GIT_BRANCH"
printf '  git rev-parse HEAD: %s\n' "$GIT_SHA"
printf '%s\n' '  git status --short:'
printf '%s\n' "$GIT_STATUS"

if [[ -n "$GIT_STATUS" && "$ALLOW_DIRTY" != '1' ]]; then
  printf '%s\n' 'ERROR: local Git worktree is dirty; set ALLOW_DIRTY=1 only for an explicit override.' >&2
  exit 1
fi

IFS=' ' read -r -a BUILD_PACKAGE_ARRAY <<< "$BUILD_PACKAGES"
if (( ${#BUILD_PACKAGE_ARRAY[@]} == 0 )); then
  printf '%s\n' 'ERROR: BUILD_PACKAGES must contain at least one package.' >&2
  exit 2
fi
for package_name in "${BUILD_PACKAGE_ARRAY[@]}"; do
  if [[ ! "$package_name" =~ ^[A-Za-z0-9_]+$ ]]; then
    printf 'ERROR: invalid ROS package name: %q\n' "$package_name" >&2
    exit 2
  fi
done

REMOTE_TARGET="${DOG_USER}@${DOG_IP}"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
REMOTE_BACKUP_DIR="/home/${DOG_USER}/deploy_backups/${TIMESTAMP}"
SSH_OPTIONS=(-o ConnectTimeout=10)
RSYNC_SSH_COMMAND=(ssh "${SSH_OPTIONS[@]}")
printf -v RSYNC_RSH '%q ' "${RSYNC_SSH_COMMAND[@]}"
RSYNC_RSH="${RSYNC_RSH% }"

printf 'Checking SSH and remote prerequisites on %s ...\n' "$REMOTE_TARGET"
printf -v remote_check_command 'bash -s -- %q' "$BUILD"
ssh "${SSH_OPTIONS[@]}" "$REMOTE_TARGET" "$remote_check_command" <<'REMOTE_CHECK'
set -euo pipefail
build_requested="$1"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf '%s\n' 'ERROR: remote ROS 2 Humble setup file is missing: /opt/ros/humble/setup.bash' >&2
  exit 1
fi
if ! command -v rsync >/dev/null 2>&1; then
  printf '%s\n' 'ERROR: remote rsync command is unavailable.' >&2
  exit 1
fi
if [[ "$build_requested" == '1' ]] && ! command -v colcon >/dev/null 2>&1; then
  printf '%s\n' 'ERROR: remote colcon command is unavailable while BUILD=1.' >&2
  exit 1
fi

printf '  remote architecture: %s\n' "$(uname -m)"
printf '%s\n' '  remote disk information:'
df -h "$HOME"
REMOTE_CHECK

if [[ "$DRY_RUN" == '0' ]]; then
  printf -v remote_mkdir_command 'bash -s -- %q %q' "$REMOTE_WS/src" "$REMOTE_BACKUP_DIR"
  ssh "${SSH_OPTIONS[@]}" "$REMOTE_TARGET" "$remote_mkdir_command" <<'REMOTE_MKDIR'
set -euo pipefail
mkdir -p -- "$1" "$2"
REMOTE_MKDIR
else
  printf '%s\n' 'DRY_RUN=1: remote workspace and backup directories will not be created.'
fi

RSYNC_OPTIONS=(
  --archive
  --compress
  --human-readable
  --itemize-changes
  --protect-args
  --rsh="$RSYNC_RSH"
  --backup
  "--backup-dir=${REMOTE_BACKUP_DIR}"
  --exclude='**/build/'
  --exclude='**/install/'
  --exclude='**/log/'
  --exclude='**/__pycache__/'
  --exclude='**/*.pyc'
  --exclude='**/.git/'
  --exclude='odin_ros_driver/recorddata/'
  --exclude='odin_ros_driver/map/'
)

if [[ "$DRY_RUN" == '1' ]]; then
  RSYNC_OPTIONS+=(--dry-run)
fi
if [[ "$DELETE" == '1' ]]; then
  RSYNC_OPTIONS+=(--delete)
fi

printf 'Synchronizing %s to %s:%s ...\n' "$LOCAL_WS/src/" "$REMOTE_TARGET" "$REMOTE_WS/src/"
rsync "${RSYNC_OPTIONS[@]}" "$LOCAL_WS/src/" "${REMOTE_TARGET}:${REMOTE_WS}/src/"

if [[ "$BUILD" == '1' && "$DRY_RUN" == '0' ]]; then
  remote_build_command="$(printf 'bash -s --'; printf ' %q' "$REMOTE_WS" "${BUILD_PACKAGE_ARRAY[@]}")"
  printf 'Building selected packages remotely: %s\n' "${BUILD_PACKAGE_ARRAY[*]}"
  ssh "${SSH_OPTIONS[@]}" "$REMOTE_TARGET" "$remote_build_command" <<'REMOTE_BUILD'
set -euo pipefail
remote_ws="$1"
shift
cd -- "$remote_ws"
source /opt/ros/humble/setup.bash
colcon build --packages-select "$@"
REMOTE_BUILD
elif [[ "$BUILD" == '1' ]]; then
  printf '%s\n' 'DRY_RUN=1: remote colcon build was not executed.'
fi

printf '\n%s\n' 'Deployment summary:'
printf '  local Git branch: %s\n' "$GIT_BRANCH"
printf '  local Git SHA: %s\n' "$GIT_SHA"
printf '  remote target: %s:%s\n' "$REMOTE_TARGET" "$REMOTE_WS"
printf '  DRY_RUN: %s\n' "$DRY_RUN"
printf '  BUILD: %s\n' "$BUILD"
printf '  DELETE: %s\n' "$DELETE"
printf '  build packages: %s\n' "${BUILD_PACKAGE_ARRAY[*]}"
if [[ "$DRY_RUN" == '1' ]]; then
  printf '%s\n' '  deployment result: PREVIEW ONLY (no remote files were changed).'
else
  printf '%s\n' '  deployment result: SUCCESS.'
fi
