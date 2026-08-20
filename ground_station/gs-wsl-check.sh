#!/usr/bin/env bash

set -euo pipefail

expected_platformio_version='6.1.19'
workspace='mirror'
always_install_dependencies=false
verify_sync=false
sync_only=false

usage() {
  cat <<'EOF'
Usage: gs-wsl-check.sh [options] <WSL repository path>

Options:
  --workspace mounted|mirror  Run from /mnt/c or a native WSL mirror (default: mirror).
  --always-install-deps       Run `platformio pkg install` even when inputs are unchanged.
  --verify-sync               Verify the mirror after synchronizing it.
  --sync-only                 Synchronize the mirror without building or linting.
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --workspace)
      workspace="${2:-}"
      shift 2
      ;;
    --always-install-deps)
      always_install_dependencies=true
      shift
      ;;
    --verify-sync)
      verify_sync=true
      shift
      ;;
    --sync-only)
      sync_only=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      break
      ;;
  esac
done

if (( $# != 1 )); then
  usage >&2
  exit 2
fi
if [[ "$workspace" != 'mounted' && "$workspace" != 'mirror' ]]; then
  echo "Unsupported workspace: $workspace" >&2
  exit 2
fi

repo_root="${1%/}"
source_project="$repo_root/ground_station"
default_cache_root="$(realpath -m "$HOME/.cache/cats-gs-precommit")"
cache_root="$(realpath -m "${CATS_GS_WSL_CACHE:-$default_cache_root}")"
venv="$cache_root/venv"
platformio="$venv/bin/platformio"

case "$cache_root/" in
  "$default_cache_root/"*) ;;
  *)
    echo "The WSL cache must stay under: $default_cache_root" >&2
    exit 2
    ;;
esac
if [[ ! -d "$source_project" || ! -f "$repo_root/.clang-tidy" ]]; then
  echo "Ground Station repository not found at: $repo_root" >&2
  exit 1
fi
if [[ ! -x "$platformio" ]]; then
  echo 'WSL pre-commit tools are missing. Follow the one-time setup in ground_station/AGENTS.md.' >&2
  exit 1
fi
if ! command -v rsync >/dev/null; then
  echo 'rsync is required in WSL for the Ground Station source mirror.' >&2
  exit 1
fi

platformio_version="$($platformio --version)"
if [[ "$platformio_version" != *"$expected_platformio_version"* ]]; then
  echo "PlatformIO $expected_platformio_version is required in WSL. Found: $platformio_version" >&2
  exit 1
fi

timed() {
  local label="$1"
  shift
  local start_ns end_ns elapsed_ms status=0

  start_ns="$(date +%s%N)"
  "$@" || status=$?
  end_ns="$(date +%s%N)"
  elapsed_ms=$(( (end_ns - start_ns) / 1000000 ))
  printf 'CATS_TIMING|wsl:%s|%d.%03d\n' "$label" "$(( elapsed_ms / 1000 ))" "$(( elapsed_ms % 1000 ))"
  return "$status"
}

rsync_excludes=(
  --exclude '/.pio/'
  --exclude '/.vscode/'
  --exclude '/simulator/build-wasm/'
  --exclude '/simulator/scenarios/*.actual.json'
  --exclude '/simulator/scenarios/*.actual.png'
  --exclude '__pycache__/'
  --exclude '*.pyc'
)

mirror_repo="$cache_root/worktree"
mirror_project="$mirror_repo/ground_station"

sync_mirror() {
  mkdir -p "$mirror_project"
  rsync --archive --delete --delete-delay "${rsync_excludes[@]}" "$source_project/" "$mirror_project/"
  rsync --archive "$repo_root/.clang-tidy" "$mirror_repo/.clang-tidy"
}

verify_mirror() {
  local differences
  differences="$(rsync --archive --delete --dry-run --itemize-changes \
    "${rsync_excludes[@]}" "$source_project/" "$mirror_project/")"
  if [[ -n "$differences" ]]; then
    printf '%s\n' "$differences" >&2
    echo 'The WSL mirror differs from the Windows working tree.' >&2
    return 1
  fi
  cmp --silent "$repo_root/.clang-tidy" "$mirror_repo/.clang-tidy"
}

if [[ "$workspace" == 'mirror' ]]; then
  timed sync sync_mirror
  if $verify_sync; then
    timed verify-sync verify_mirror
  fi
  if $sync_only; then
    echo 'Ground Station WSL mirror is synchronized.'
    exit 0
  fi
  project_root="$mirror_project"
  build_dir="${CATS_GS_WSL_BUILD_DIR:-$cache_root/build-mirror}"
  libdeps_dir="${CATS_GS_WSL_LIBDEPS_DIR:-$cache_root/libdeps-mirror}"
else
  if $sync_only; then
    echo '--sync-only requires --workspace mirror.' >&2
    exit 2
  fi
  project_root="$source_project"
  build_dir="${CATS_GS_WSL_BUILD_DIR:-$cache_root/build}"
  libdeps_dir="${CATS_GS_WSL_LIBDEPS_DIR:-}"
fi

export PLATFORMIO_BUILD_DIR="$build_dir"
if [[ -n "$libdeps_dir" ]]; then
  export PLATFORMIO_LIBDEPS_DIR="$libdeps_dir"
fi

dependency_fingerprint() {
  {
    printf '%s\n' "$platformio_version"
    sha256sum "$project_root/platformio.ini"
  } | sha256sum | cut -d ' ' -f 1
}

prepare_dependencies() {
  local fingerprint stamp

  if [[ -z "$libdeps_dir" ]]; then
    "$platformio" pkg install --project-dir "$project_root"
    return
  fi

  mkdir -p "$libdeps_dir"
  stamp="$libdeps_dir/.cats-dependencies.sha256"
  fingerprint="$(dependency_fingerprint)"
  if ! $always_install_dependencies && [[ -f "$stamp" ]] && [[ "$(<"$stamp")" == "$fingerprint" ]]; then
    echo 'Ground Station WSL dependencies are unchanged; using the cached installation.'
    return
  fi

  "$platformio" pkg install --project-dir "$project_root"
  printf '%s\n' "$fingerprint" > "$stamp"
}

build_project() {
  "$platformio" run -d "$project_root"
}

lint_project() {
  local lint_output lint_status filtered_output

  lint_status=0
  lint_output="$(cd "$project_root" && "$platformio" check 2>&1)" || lint_status=$?
  printf '%s\n' "$lint_output"
  if (( lint_status != 0 )); then
    return "$lint_status"
  fi

  filtered_output="$(printf '%s\n' "$lint_output" | \
    grep -vE 'clang-analyzer-valist.Uninitialized|/\.platformio/packages/' || true)"
  if grep -q 'warning' <<<"$filtered_output"; then
    echo 'clang-tidy check failed: unexpected warning found.' >&2
    return 1
  fi
}

timed dependencies prepare_dependencies
timed build build_project
timed lint lint_project

echo "Ground Station WSL checks passed from the $workspace workspace."
