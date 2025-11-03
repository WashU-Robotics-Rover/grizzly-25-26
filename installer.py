#!/usr/bin/env python3
"""
Grizzly Installer - Automatic release downloader and builder for Grizzly ROS 2 packages

This script checks GitHub for the latest release of Grizzly and automatically downloads
and installs it. It supports both macOS (with robostack/conda) and Linux (with native ROS 2).
"""

import json
import urllib.request
import urllib.error
import os
import tarfile
import subprocess
import stat
import platform
import sys
import shutil

def main():
    release = get_latest_release("WashU-Robotics-Rover/grizzly-25-26")
    tag = release.get("tag_name", "unknown")
    name = release.get("name", tag)
    print(f"Latest release: {name} ({tag})")

    # Check if version.json exists and compare versions
    os.makedirs("downloads", exist_ok=True)
    version_file = os.path.join("downloads", "version.json")
    
    if os.path.exists(version_file):
        try:
            with open(version_file, 'r') as f:
                local_version = json.load(f)
                local_version_string = local_version.get("version", "unknown")
                
            if tag == local_version_string or check_version_match(release, local_version):
                print(f"Local version ({local_version_string}) is up to date. Skipping download.")
                return
            else:
                print(f"Local version ({local_version_string}) differs from latest ({tag}). Downloading...")
                # Remove everything in downloads folder
                for f in os.listdir("downloads"):
                    file_path = os.path.join("downloads", f)
                    if os.path.isfile(file_path):
                        os.remove(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)

        except (json.JSONDecodeError, IOError) as e:
            print(f"Error reading version.json: {e}. Proceeding with download...")
    else:
        print("No local version found. Downloading...")

    assets = release.get("assets", [])
    downloaded_files = []

    if assets:
        for asset in assets:
            url = asset["browser_download_url"]
            filename = os.path.join("downloads", asset["name"])
            download_file(url, filename)
            downloaded_files.append(filename)
    else:
        source_url = release.get("zipball_url")
        filename = os.path.join("downloads", f"grizzly_{tag}.zip")
        download_file(source_url, filename)
        downloaded_files.append(filename)

    print("Download complete.")
    
    # Extract tarball or zip files to downloads directory
    extract_path = extract_archives(downloaded_files)
    
    # Look for source directory
    src_path = os.path.join(extract_path, "src")
    if not os.path.exists(src_path):
        print(f"Error: src directory not found in {extract_path}")
        sys.exit(1)
    
    # Build the workspace based on the platform
    build_workspace(src_path)


def check_version_match(release, local_version):
    """Check if the release matches the local version by examining assets."""
    assets = release.get("assets", [])
    for asset in assets:
        if asset["name"] == "version.json":
            # Download and compare the remote version.json
            try:
                url = asset["browser_download_url"]
                req = urllib.request.Request(url, headers={"User-Agent": "release-checker"})
                with urllib.request.urlopen(req) as resp:
                    remote_version = json.load(resp)
                    return remote_version.get("version") == local_version.get("version")
            except Exception as e:
                print(f"Error fetching remote version.json: {e}")
                return False
    return False

def get_latest_release(owner_repo):
    """Fetch metadata for the latest release of a public repo."""
    url = f"https://api.github.com/repos/{owner_repo}/releases"
    req = urllib.request.Request(url, headers={"User-Agent": "release-checker"})
    try:
        with urllib.request.urlopen(req) as resp:
            releases = json.load(resp)
            # Find the latest pre-release
            for release in releases:
                if release.get("prerelease", False):
                    return release
            print("No pre-releases found.")
            return {}
    except urllib.error.HTTPError as e:
        print(f"HTTP Error {e.code}: {e.reason}")
        return {}

def download_file(url, filename):
    """Download a file from a URL."""
    print(f"Downloading {filename}...")
    urllib.request.urlretrieve(url, filename)
    print(f"Saved: {filename}")

def extract_archives(files):
    """Extract tar.gz and zip files to downloads directory."""
    extract_path = 'downloads'
    for filepath in files:
        if filepath.endswith('.tar.gz') or filepath.endswith('.tgz'):
            print(f"Extracting {filepath} to {extract_path}...")
            try:
                with tarfile.open(filepath, 'r:gz') as tar:
                    # Extract to the downloads directory
                    tar.extractall(path=extract_path)
                print(f"Extracted {filepath}")
            except Exception as e:
                print(f"Error extracting {filepath}: {e}")
        elif filepath.endswith('.zip'):
            print(f"Extracting {filepath} to {extract_path}...")
            try:
                import zipfile
                with zipfile.ZipFile(filepath, 'r') as zip_ref:
                    zip_ref.extractall(extract_path)
                print(f"Extracted {filepath}")
            except Exception as e:
                print(f"Error extracting {filepath}: {e}")
    
    return extract_path

def build_workspace(working_dir):
    """Build the ROS 2 workspace using platform-specific methods."""
    system = platform.system()
    
    print(f"\n{'='*60}")
    print(f"Building Grizzly workspace")
    print(f"Platform: {system}")
    print(f"Working directory: {working_dir}")
    print(f"{'='*60}\n")
    
    if system == "Darwin":  # macOS
        build_workspace_macos(working_dir)
    elif system == "Linux":
        build_workspace_linux(working_dir)
    else:
        print(f"❌ Unsupported platform: {system}")
        print("This installer supports macOS and Linux only.")
        sys.exit(1)


def build_workspace_macos(working_dir):
    """Build on macOS using conda environment and build.sh script."""
    print("Detected macOS - using robostack/conda environment...")
    
    # Check if build.sh exists
    build_script = os.path.join(working_dir, "build.sh")
    if not os.path.exists(build_script):
        print(f"❌ Error: build.sh not found at {build_script}")
        print("The macOS build requires build.sh to properly configure conda environment.")
        sys.exit(1)
    
    # Make the script executable
    try:
        current_permissions = os.stat(build_script).st_mode
        os.chmod(build_script, current_permissions | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    except Exception as e:
        print(f"Warning: Could not make build.sh executable: {e}")
    
    # Run build.sh with zsh
    try:
        subprocess.run(
            ["./build.sh"],
            shell=True,
            check=True,
            cwd=working_dir,
            executable='/bin/zsh',
            stdout=None,
            stderr=None
        )
        print(f"\n{'='*60}")
        print(f"✅ macOS build completed successfully")
        print(f"{'='*60}\n")
    except subprocess.CalledProcessError as e:
        print(f"\n{'='*60}")
        print(f"❌ Error running build.sh")
        print(f"Exit code: {e.returncode}")
        print(f"{'='*60}\n")
        print("\nBuild failed. This may be due to:")
        print("  - Conda environment 'ros_env' not found")
        print("  - ROS 2 Humble not properly installed in conda environment")
        print("  - Missing dependencies")
        print("\nMake sure conda is available and ros_env exists:")
        print("  conda info --envs")
        print("\nTo retry the build manually:")
        print(f"  cd {working_dir}")
        print(f"  ./build.sh")
        sys.exit(1)


def build_workspace_linux(working_dir):
    """Build on Linux using native ROS 2 installation."""
    print("Detected Linux - using native ROS 2 installation...")
    
    # Check for required tools
    if not check_command_exists("colcon"):
        print("❌ Error: colcon not found.")
        print("Make sure ROS 2 Humble is installed and sourced.")
        print("Install with: sudo apt install python3-colcon-common-extensions")
        sys.exit(1)
    
    # Check if ROS 2 Humble is available
    ros_setup = "/opt/ros/humble/setup.bash"
    if not os.path.exists(ros_setup):
        print(f"❌ Error: ROS 2 Humble not found at {ros_setup}")
        print("Please install ROS 2 Humble:")
        print("  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html")
        sys.exit(1)
    
    # Prepare the build command with ROS 2 sourcing
    build_type = os.environ.get("CMAKE_BUILD_TYPE", "RelWithDebInfo")
    skip_rosdep = os.environ.get("SKIP_ROSDEP", "0") == "1"
    
    # Build the command as a shell script
    commands = [
        "set -euo pipefail",
        f"source {ros_setup}",
    ]
    
    # Run rosdep if available and not skipped
    if not skip_rosdep and check_command_exists("rosdep"):
        print("Running rosdep to install dependencies...")
        commands.extend([
            "set +e",  # Don't exit on rosdep errors
            "rosdep update 2>/dev/null || true",
            "rosdep install --from-paths . --ignore-src -r -y || echo 'rosdep completed with warnings'",
            "set -e",
        ])
    
    # Run colcon build
    commands.append(f'echo "Building with CMAKE_BUILD_TYPE={build_type}..."')
    commands.append(f'colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE={build_type}')
    
    # Combine commands into a single script
    script = " && ".join(commands)
    
    try:
        subprocess.run(
            script,
            shell=True,
            check=True,
            cwd=working_dir,
            executable='/bin/bash',
            stdout=None,
            stderr=None
        )
        print(f"\n{'='*60}")
        print(f"✅ Linux build completed successfully")
        print(f"{'='*60}")
        print("\nTo use this workspace:")
        print(f"  cd {working_dir}")
        print(f"  source install/setup.bash")
        print("\nOr add to your ~/.bashrc:")
        print(f"  echo 'source {working_dir}/install/setup.bash' >> ~/.bashrc")
        print(f"{'='*60}\n")
    except subprocess.CalledProcessError as e:
        print(f"\n{'='*60}")
        print(f"❌ Error during build")
        print(f"Exit code: {e.returncode}")
        print(f"{'='*60}\n")
        print("\nBuild failed. This may be due to:")
        print("  - Missing dependencies (try: sudo apt update && sudo apt install -y python3-dev)")
        print("  - ROS 2 Humble not properly sourced")
        print("  - Missing build tools")
        print("\nTo retry the build manually:")
        print(f"  cd {working_dir}")
        print(f"  source {ros_setup}")
        print(f"  colcon build --symlink-install")
        sys.exit(1)


def check_command_exists(command):
    """Check if a command exists in the system PATH."""
    return shutil.which(command) is not None

if __name__ == "__main__":
    main()

