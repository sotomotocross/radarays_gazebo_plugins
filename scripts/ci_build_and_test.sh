#!/usr/bin/env bash
# Workspace-level build + regression check, meant to be the single source of
# truth for what CI runs. Also runnable locally: from the workspace root
# (the directory containing src/), run:
#
#   src/radarays_gazebo_plugins/scripts/ci_build_and_test.sh
#
# Assumes ROS 2 Jazzy is already installed at /opt/ros/jazzy and the
# workspace's four packages are checked out under src/ (rmagine,
# rmagine_gazebo_plugins, radarays_gazebo_plugins, radarays_ros).
set -euo pipefail

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: /opt/ros/jazzy/setup.bash not found -- is ROS 2 Jazzy installed?" >&2
  exit 1
fi

# Walk up from this script to the workspace root (the parent of src/).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
cd "${WS_ROOT}"

if [ ! -d "${WS_ROOT}/src/rmagine" ]; then
  echo "ERROR: ${WS_ROOT}/src/rmagine not found -- expected to resolve to the workspace root." >&2
  echo "Resolved WS_ROOT=${WS_ROOT} from SCRIPT_DIR=${SCRIPT_DIR}." >&2
  exit 1
fi

# ROS 2's setup.bash references variables it doesn't guarantee are set
# (e.g. AMENT_TRACE_SETUP_FILES) -- not compatible with `set -u`. Scope
# that check off just for sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

# rmagine's CMake auto-downloads a precompiled Embree release when no local
# Embree is found; that release needs libiomp5.so (Intel's OpenMP runtime),
# which most systems -- including CI images -- don't ship. See
# MIGRATION_HANDOFF.md, "Known Environment Issues" for how this was found
# and confirmed fixed. GitHub-hosted runners have passwordless sudo by
# default, so this can be automated here (on a personal dev machine, the
# equivalent step needed a human to run it once, since sudo wasn't
# passwordless there).
# Note: `ldconfig -p | grep libiomp5` is NOT a reliable presence check here
# -- ldconfig's cache indexes a library by its real embedded SONAME, not by
# a symlink's own filename, so the symlink below never shows up in that
# listing even once applied and working (confirmed empirically). Check the
# file directly instead.
if [ ! -e /usr/local/lib/libiomp5.so ]; then
  echo "Applying libiomp5.so workaround (see MIGRATION_HANDOFF.md)..."
  sudo ln -sf /lib/x86_64-linux-gnu/libgomp.so.1 /usr/local/lib/libiomp5.so
  sudo ldconfig
fi

# This machine (and potentially others used to run this script locally)
# has two OptiX SDKs installed side by side: an older one that actually
# matches the installed driver's ABI, and a newer default that doesn't
# (silently -- compiles fine, only fails at runtime with
# OPTIX_ERROR_UNSUPPORTED_ABI_VERSION). rmagine's own OptiX CTest suite
# caught this after a clean rebuild picked the wrong one by default (see
# MIGRATION_HANDOFF.md, "Phase 1 Regression Pass"). GitHub-hosted CPU
# runners have no GPU/CUDA at all, so rmagine's own CMake auto-skips the
# OptiX build there and this is a no-op -- but pin it defensively whenever
# the known-good SDK is actually present, so this script stays safe to run
# on a real GPU dev machine too, not just in CPU-only CI.
OPTIX_CMAKE_ARGS=()
if [ -d "${HOME}/optix-7.5/include" ]; then
  OPTIX_CMAKE_ARGS=(-DOptiX_INCLUDE_DIR="${HOME}/optix-7.5/include")
fi

echo "=== colcon build ==="
colcon build \
  --packages-select rmagine rmagine_gazebo_plugins radarays_gazebo_plugins radarays_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "${OPTIX_CMAKE_ARGS[@]}"

set +u
# shellcheck disable=SC1091
source "${WS_ROOT}/install/setup.bash"
set -u
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

echo "=== radarays_ros interfaces sanity check ==="
ros2 interface show radarays_ros/action/GenRadarImage > /dev/null

# `colcon test` runs every fixture wired via `ament_add_test` across all
# four packages -- rmagine's own core/Embree/CUDA CTest suite (its OptiX
# tests are a no-op on a CPU-only runner, see above), rmagine_gazebo_plugins'
# embree_fixture_* (baseline/dynamic/pinhole/o1dn/ondn/plane/heightmap/
# ignore_link), and radarays_gazebo_plugins' radarays_fixture_*
# (static_cpu/dynamic_cpu/dynamic_multi_cpu). This used to be reimplemented
# by hand here as a bash loop over just the radarays_gazebo_plugins
# fixtures -- meaning CI never actually ran rmagine's own test suite or
# rmagine_gazebo_plugins' fixtures at all, and would NOT have caught the
# OptiX SDK regression above even on a machine where it mattered. Fixed by
# just running the real `colcon test`/`colcon test-result` instead of
# partially reimplementing it.
echo "=== colcon test ==="
colcon test \
  --packages-select rmagine rmagine_gazebo_plugins radarays_gazebo_plugins radarays_ros

if ! colcon test-result --all --verbose; then
  echo "One or more regression tests failed." >&2
  exit 1
fi

echo "All regression tests passed."
