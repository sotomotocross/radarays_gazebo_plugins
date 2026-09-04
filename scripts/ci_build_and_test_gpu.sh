#!/usr/bin/env bash
# GPU/OptiX counterpart to ci_build_and_test.sh -- MUST run on a self-hosted
# runner with a real NVIDIA GPU (GitHub-hosted runners have no GPU/CUDA at
# all). Covers what the CPU-only workflow can't: rmagine's own OptiX CTest
# suite (optix_simulation_spherical/pinhole/o1dn/ondn, optix_correction_rcc)
# and, transitively, that rmagine_optix_map_system/rmagine_optix_sensor_system/
# radarays_optix_sensor_system/radar_simulator_gpu all still link and load.
#
# From the workspace root (the directory containing src/), run:
#   src/radarays_gazebo_plugins/scripts/ci_build_and_test_gpu.sh
#
# Assumes ROS 2 Jazzy is already installed at /opt/ros/jazzy and the
# workspace's four packages are checked out under src/ (rmagine,
# rmagine_gazebo_plugins, radarays_gazebo_plugins, radarays_ros).
#
# Required runner preconditions (not automated here -- provisioning a
# self-hosted runner is a one-time, machine-specific setup step, not
# something this script should be trusted to do unattended):
#   - A real NVIDIA GPU + driver, with an OptiX SDK on disk whose ABI
#     version the installed driver actually supports.
#   - OPTIX_INCLUDE_DIR set to that SDK's include/ dir. There is
#     deliberately no hardcoded default path here: a wrong-but-present
#     default (as opposed to a missing one) is exactly the failure mode
#     that bit this migration once already -- a clean rebuild silently
#     picked a newer, ABI-incompatible SDK over the one that actually
#     works, and it only surfaced as a runtime crash in rmagine's own
#     OptiX CTest suite, not a build failure. See MIGRATION_HANDOFF.md,
#     "Phase 1 Regression Pass -- 2026-07-29" for the full story. Failing
#     loudly here if OPTIX_INCLUDE_DIR isn't set is deliberate, so a
#     misconfigured runner can't silently fall back to a wrong SDK the way
#     the unpinned CMake default did.
set -euo pipefail

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: /opt/ros/jazzy/setup.bash not found -- is ROS 2 Jazzy installed?" >&2
  exit 1
fi

if [ -z "${OPTIX_INCLUDE_DIR:-}" ]; then
  echo "ERROR: OPTIX_INCLUDE_DIR is not set. This script requires a GPU" >&2
  echo "runner with a working OptiX SDK -- point OPTIX_INCLUDE_DIR at its" >&2
  echo "include/ dir. See this script's own header comment for why there is" >&2
  echo "deliberately no hardcoded default." >&2
  exit 1
fi

if ! command -v nvidia-smi > /dev/null 2>&1; then
  echo "ERROR: nvidia-smi not found -- this script must run on a real GPU runner." >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
cd "${WS_ROOT}"

if [ ! -d "${WS_ROOT}/src/rmagine" ]; then
  echo "ERROR: ${WS_ROOT}/src/rmagine not found -- expected to resolve to the workspace root." >&2
  echo "Resolved WS_ROOT=${WS_ROOT} from SCRIPT_DIR=${SCRIPT_DIR}." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

if [ ! -e /usr/local/lib/libiomp5.so ]; then
  echo "Applying libiomp5.so workaround (see MIGRATION_HANDOFF.md)..."
  sudo ln -sf /lib/x86_64-linux-gnu/libgomp.so.1 /usr/local/lib/libiomp5.so
  sudo ldconfig
fi

echo "=== colcon build (OptiX_INCLUDE_DIR=${OPTIX_INCLUDE_DIR}) ==="
colcon build \
  --packages-select rmagine rmagine_gazebo_plugins radarays_gazebo_plugins radarays_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DOptiX_INCLUDE_DIR="${OPTIX_INCLUDE_DIR}"

set +u
# shellcheck disable=SC1091
source "${WS_ROOT}/install/setup.bash"
set -u
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# See ci_build_and_test.sh's matching comment for the full root-cause
# writeup (stale FastRTPS shared-memory segments accumulating across
# repeated test invocations measurably worsened fixture flakiness).
# This is the actual script the GPU workflow runs -- ci_build_and_test.sh
# (CPU) never touches this job at all, confirmed the hard way after a
# fix there had zero effect here.
echo "=== clearing stale FastRTPS shared-memory segments ==="
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

echo "=== colcon test ==="
colcon test \
  --packages-select rmagine rmagine_gazebo_plugins radarays_gazebo_plugins radarays_ros

if ! colcon test-result --all --verbose; then
  echo "One or more regression tests failed." >&2
  exit 1
fi

# Known gap, not fixed by this script (see MIGRATION_HANDOFF.md and
# rmagine_gazebo_plugins/README.md): there is no automated fixture at all
# for OptiX-side rmagine_gazebo_plugins/radarays_gazebo_plugins features
# (noise models, GPU multi-topic, GPU egomotion) or for radar_simulator_gpu
# itself -- `colcon test` above only proves rmagine's own OptiX unit tests
# pass, not that the downstream GPU integration paths still work. Manual
# verification is still required for those until fixtures exist.
echo "All regression tests passed (rmagine's own OptiX suite only -- see this script's tail comment for what's still uncovered)."
