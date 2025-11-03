#!/usr/bin/env python3
"""
Cross-platform run script for Grizzly workspace

This script detects the platform and launches the Grizzly minimal system
using the appropriate method for each operating system:
- macOS: Uses run.sh with conda environment
- Linux: Sources ROS 2 Humble and launches directly
- Windows: Sources ROS 2 Humble from standard Windows installation

Usage:
    python3 run.py              # Launch the minimal system
    python3 run.py --help       # Show help message
"""

import os
import sys
import platform
import subprocess
import argparse


def main():
    parser = argparse.ArgumentParser(
        description="Launch the Grizzly minimal system",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 run.py                    # Launch the system
  python3 run.py --workspace ./     # Specify workspace directory

Platform-specific behavior:
  macOS:   Uses conda environment (ros_env) via run.sh
  Linux:   Sources /opt/ros/humble/setup.bash
  Windows: Sources C:\\dev\\ros2_humble\\setup.bat
        """
    )
    parser.add_argument(
        '--workspace',
        default='.',
        help='Path to the workspace directory (default: current directory)'
    )
    
    args = parser.parse_args()
    
    # Get absolute workspace path
    workspace = os.path.abspath(args.workspace)
    
    # Detect platform and run
    system = platform.system()
    
    print(f"{'='*60}")
    print(f"  Grizzly Launch Script")
    print(f"{'='*60}")
    print(f"Platform: {system}")
    print(f"Workspace: {workspace}")
    print(f"{'='*60}\n")
    
    if system == "Darwin":  # macOS
        run_macos(workspace)
    elif system == "Linux":
        run_linux(workspace)
    elif system == "Windows":
        run_windows(workspace)
    else:
        print(f"❌ Error: Unsupported platform: {system}")
        print("This script supports macOS, Linux, and Windows only.")
        sys.exit(1)


def run_macos(workspace):
    """Run on macOS using run.sh script with conda environment."""
    print("Detected macOS - using conda environment (ros_env)...\n")
    
    # Check if run.sh exists
    run_script = os.path.join(workspace, "run.sh")
    if not os.path.exists(run_script):
        print(f"❌ Error: run.sh not found at {run_script}")
        print("\nFor macOS, the run.sh script is required to properly configure")
        print("the conda environment and launch the system.")
        print("\nMake sure you're in the correct workspace directory.")
        sys.exit(1)
    
    # Check if workspace is built
    install_dir = os.path.join(workspace, "install")
    if not os.path.exists(install_dir):
        print(f"❌ Error: Workspace not built. Install directory not found.")
        print("\nPlease build the workspace first:")
        print(f"  cd {workspace}")
        print(f"  ./build.sh")
        sys.exit(1)
    
    # Make run.sh executable
    try:
        import stat
        current_permissions = os.stat(run_script).st_mode
        os.chmod(run_script, current_permissions | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    except Exception as e:
        print(f"⚠️  Warning: Could not make run.sh executable: {e}")
    
    # Run the script
    try:
        subprocess.run(
            ["./run.sh"],
            cwd=workspace,
            shell=True,
            executable='/bin/zsh',
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Error: Launch failed with exit code {e.returncode}")
        print("\nThis may be due to:")
        print("  - Conda environment 'ros_env' not found")
        print("  - Workspace not built (run ./build.sh)")
        print("  - ROS 2 not properly installed in conda environment")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n✅ Grizzly system stopped by user (Ctrl+C)")
        sys.exit(0)


def run_linux(workspace):
    """Run on Linux using native ROS 2 Humble installation."""
    print("Detected Linux - using native ROS 2 Humble installation...\n")
    
    # Check if workspace is built
    install_dir = os.path.join(workspace, "install")
    setup_script = os.path.join(install_dir, "setup.bash")
    
    if not os.path.exists(install_dir):
        print(f"❌ Error: Workspace not built. Install directory not found.")
        print("\nPlease build the workspace first:")
        print(f"  cd {workspace}")
        print(f"  ./install_grizzly.sh")
        sys.exit(1)
    
    if not os.path.exists(setup_script):
        print(f"❌ Error: Workspace setup script not found at {setup_script}")
        print("\nThe workspace may not be properly built.")
        print("Try rebuilding:")
        print(f"  cd {workspace}")
        print(f"  rm -rf build/ install/ log/")
        print(f"  ./install_grizzly.sh")
        sys.exit(1)
    
    # Check for ROS 2 Humble
    ros_setup = "/opt/ros/humble/setup.bash"
    if not os.path.exists(ros_setup):
        print(f"❌ Error: ROS 2 Humble not found at {ros_setup}")
        print("\nPlease install ROS 2 Humble:")
        print("  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html")
        sys.exit(1)
    
    # Build the launch command
    # We need to source both ROS 2 and the workspace, then launch
    launch_command = f"""
set -e
source {ros_setup}
source {setup_script}

echo ""
echo "================================================"
echo "  Launching Grizzly Minimal System"
echo "================================================"
echo ""
echo "Starting nodes:"
echo "  - system_manager (lifecycle node)"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""

ros2 launch grizzly_stack grizzly_minimal.launch.py
"""
    
    try:
        subprocess.run(
            launch_command,
            cwd=workspace,
            shell=True,
            executable='/bin/bash',
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Error: Launch failed with exit code {e.returncode}")
        print("\nThis may be due to:")
        print("  - ROS 2 Humble not sourced properly")
        print("  - Workspace not built correctly")
        print("  - Missing dependencies")
        print("\nTry:")
        print(f"  source {ros_setup}")
        print(f"  source {setup_script}")
        print(f"  ros2 launch grizzly_stack grizzly_minimal.launch.py")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n✅ Grizzly system stopped by user (Ctrl+C)")
        sys.exit(0)


def run_windows(workspace):
    """Run on Windows using ROS 2 Humble installation."""
    print("Detected Windows - using ROS 2 Humble installation...\n")
    
    # Check if workspace is built
    install_dir = os.path.join(workspace, "install")
    setup_script = os.path.join(install_dir, "setup.bat")
    
    if not os.path.exists(install_dir):
        print(f"❌ Error: Workspace not built. Install directory not found.")
        print("\nPlease build the workspace first:")
        print(f"  cd {workspace}")
        print(f"  colcon build --symlink-install")
        sys.exit(1)
    
    if not os.path.exists(setup_script):
        print(f"❌ Error: Workspace setup script not found at {setup_script}")
        print("\nThe workspace may not be properly built.")
        print("Try rebuilding:")
        print(f"  cd {workspace}")
        print(f"  colcon build --symlink-install")
        sys.exit(1)
    
    # Check for ROS 2 Humble (common installation paths)
    possible_ros_paths = [
        r"C:\dev\ros2_humble\setup.bat",
        r"C:\opt\ros\humble\setup.bat",
        os.path.expandvars(r"%LOCALAPPDATA%\ros2_humble\setup.bat"),
    ]
    
    ros_setup = None
    for path in possible_ros_paths:
        if os.path.exists(path):
            ros_setup = path
            break
    
    if ros_setup is None:
        print(f"❌ Error: ROS 2 Humble not found in common locations:")
        for path in possible_ros_paths:
            print(f"  - {path}")
        print("\nPlease install ROS 2 Humble for Windows:")
        print("  https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html")
        print("\nOr set up the environment manually and run:")
        print(f'  call "{setup_script}"')
        print(f"  ros2 launch grizzly_stack grizzly_minimal.launch.py")
        sys.exit(1)
    
    print(f"Found ROS 2 at: {ros_setup}\n")
    
    # Build the launch command
    # Windows uses call for batch files and doesn't use source
    launch_command = f"""
@echo off
call "{ros_setup}"
call "{setup_script}"

echo.
echo ================================================
echo   Launching Grizzly Minimal System
echo ================================================
echo.
echo Starting nodes:
echo   - system_manager (lifecycle node)
echo.
echo Press Ctrl+C to stop all nodes
echo.

ros2 launch grizzly_stack grizzly_minimal.launch.py
"""
    
    # Write to a temporary batch file and execute it
    import tempfile
    with tempfile.NamedTemporaryFile(mode='w', suffix='.bat', delete=False) as f:
        f.write(launch_command)
        batch_file = f.name
    
    try:
        subprocess.run(
            [batch_file],
            cwd=workspace,
            shell=True,
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Error: Launch failed with exit code {e.returncode}")
        print("\nThis may be due to:")
        print("  - ROS 2 Humble not installed properly")
        print("  - Workspace not built correctly")
        print("  - Missing dependencies")
        print("\nTry manually:")
        print(f'  call "{ros_setup}"')
        print(f'  call "{setup_script}"')
        print(f"  ros2 launch grizzly_stack grizzly_minimal.launch.py")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\n✅ Grizzly system stopped by user (Ctrl+C)")
        sys.exit(0)
    finally:
        # Clean up temporary batch file
        try:
            os.unlink(batch_file)
        except:
            pass


if __name__ == "__main__":
    main()
