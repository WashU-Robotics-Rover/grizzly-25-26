# grizzly – source release
This archive contains the **source** for the `grizzly` ROS 2 (Humble) package(s),
plus a helper installer script.
## Requirements
- ROS 2 Humble environment (e.g., `/opt/ros/humble`), including `colcon`
- `rosdep` (recommended)
## Build & Use
```bash
./install_grizzly.sh
source install/setup.bash
```
Optional flags:
- `--no-rosdep`  : skip dependency installation
- `--release`    : use `-DCMAKE_BUILD_TYPE=Release`
