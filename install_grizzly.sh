#!/usr/bin/env bash
set -euo pipefail
WANT_ROSDEP=1
BUILD_TYPE="RelWithDebInfo"
for arg in "$@"; do
  case "$arg" in
    --no-rosdep) WANT_ROSDEP=0 ;;
    --release)   BUILD_TYPE="Release" ;;
    *) echo "Unknown arg: $arg" >&2; exit 2 ;;
  esac
done
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found. Make sure ROS 2 Humble is installed and sourced." >&2
  echo "For Docker: use 'ros:humble' or source /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [ "$WANT_ROSDEP" -eq 1 ] && command -v rosdep >/dev/null 2>&1; then
  set +e
  rosdep update
  set -e
  rosdep install --from-paths . --ignore-src -r -y || {
    echo "rosdep install completed with non-fatal errors; continuing." >&2
  }
fi
echo "Building with CMAKE_BUILD_TYPE=${BUILD_TYPE} ..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
cat <<EONOTE
=======================
✅ Build finished.
To use this workspace in the current shell:
    source install/setup.bash
You can add this line to your shell RC:
    echo 'source "$(pwd)/install/setup.bash"' >> ~/.bashrc   # or ~/.zshrc
=======================
EONOTE
