#!/usr/bin/env python3
"""
Grizzly - Unified CLI tool for the Grizzly Rover Software Stack

This is the single entry point for all Grizzly operations:
    grizzly.py install    - Download and install the latest release
    grizzly.py build      - Build the workspace
    grizzly.py run        - Launch the Grizzly system
    grizzly.py test       - Run the test suite

Cross-platform support for macOS, Linux, and Windows.
"""

import os
import sys
import platform
import subprocess
import argparse
import json
import urllib.request
import urllib.error
import tarfile
import shutil
import stat
from pathlib import Path


class GrizzlyCLI:
    """Unified command-line interface for Grizzly operations."""
    
    def __init__(self):
        self.workspace_root = Path(__file__).parent.resolve()
        self.system = platform.system()
        self.downloads_dir = self.workspace_root / "downloads"
        self.install_dir = self.workspace_root / "install"
        self.test_dir = self.workspace_root / "grizzly_stack" / "test"
        
    def main(self):
        """Main entry point - parse arguments and dispatch to appropriate command."""
        parser = argparse.ArgumentParser(
            description="Grizzly Rover Software Stack - Unified CLI",
            formatter_class=argparse.RawDescriptionHelpFormatter,
            epilog="""
Commands:
  install      Download and install the latest release from GitHub
  build        Build the workspace using platform-specific configuration
  run          Launch the Grizzly minimal system
  test         Run the test suite

Examples:
  ./grizzly.py install              # Download and build latest release
  ./grizzly.py build                # Build the current workspace
  ./grizzly.py build --clean        # Clean build from scratch
  ./grizzly.py run                  # Launch the system
  ./grizzly.py test                 # Run all tests
  ./grizzly.py test -v              # Run tests with verbose output
  ./grizzly.py test --coverage      # Run tests with coverage report

Platform: {system}
            """.format(system=self.system)
        )
        
        subparsers = parser.add_subparsers(dest='command', help='Command to run')
        
        # Install command
        install_parser = subparsers.add_parser('install', help='Download and install latest release')
        install_parser.add_argument('--repo', default='WashU-Robotics-Rover/grizzly-25-26',
                                   help='GitHub repository (default: WashU-Robotics-Rover/grizzly-25-26)')
        install_parser.add_argument('--force', action='store_true',
                                   help='Force re-download even if version matches')
        
        # Build command
        build_parser = subparsers.add_parser('build', help='Build the workspace')
        build_parser.add_argument('--clean', action='store_true',
                                 help='Clean build (remove build/, install/, log/ first)')
        build_parser.add_argument('--release', action='store_true',
                                 help='Build in Release mode (default: RelWithDebInfo)')
        build_parser.add_argument('--no-rosdep', action='store_true',
                                 help='Skip rosdep dependency installation (Linux only)')
        build_parser.add_argument('--symlink-install', action='store_true',
                                 help='Use symlink install for faster iteration')
        build_parser.add_argument('--packages-select', nargs='+',
                                 help='Build only specific packages')
        
        # Run command
        run_parser = subparsers.add_parser('run', help='Launch the Grizzly system')
        run_parser.add_argument('--debug', action='store_true',
                               help='Launch with debug output')
        
        # Test command
        test_parser = subparsers.add_parser('test', help='Run the test suite')
        test_parser.add_argument('-v', '--verbose', action='store_true',
                                help='Verbose test output')
        test_parser.add_argument('--coverage', action='store_true',
                                help='Generate coverage report')
        test_parser.add_argument('--html', action='store_true',
                                help='Generate HTML coverage report')
        test_parser.add_argument('--list', action='store_true',
                                help='List all available tests')
        test_parser.add_argument('--specific', metavar='TEST',
                                help='Run specific test file or test case')
        
        # Clean command
        clean_parser = subparsers.add_parser('clean', help='Clean build artifacts and downloads')
        clean_parser.add_argument('--all', action='store_true',
                                 help='Clean everything (build, install, log, downloads)')
        clean_parser.add_argument('--downloads', action='store_true',
                                 help='Clean only downloads directory')
        clean_parser.add_argument('--build', action='store_true',
                                 help='Clean only build artifacts (build, install, log)')
        
        args = parser.parse_args()
        
        if not args.command:
            parser.print_help()
            sys.exit(1)
        
        # Dispatch to appropriate handler
        if args.command == 'install':
            return self.cmd_install(args)
        elif args.command == 'build':
            return self.cmd_build(args)
        elif args.command == 'run':
            return self.cmd_run(args)
        elif args.command == 'test':
            return self.cmd_test(args)
        elif args.command == 'clean':
            return self.cmd_clean(args)
    
    # ========================================================================
    # INSTALL COMMAND
    # ========================================================================
    
    def cmd_install(self, args):
        """Install command - download and build the latest release."""
        print(f"\n{'='*70}")
        print(f"  Grizzly Installer")
        print(f"{'='*70}")
        print(f"Platform: {self.system}")
        print(f"Repository: {args.repo}")
        print(f"{'='*70}\n")
        
        # Get latest release
        release = self.get_latest_release(args.repo)
        if not release:
            print("❌ Failed to fetch release information")
            return 1
        
        tag = release.get("tag_name", "unknown")
        name = release.get("name", tag)
        print(f"Latest release: {name} ({tag})")
        
        # Check if already installed
        self.downloads_dir.mkdir(exist_ok=True)
        version_file = self.downloads_dir / "version.json"
        
        if not args.force and version_file.exists():
            try:
                with open(version_file, 'r') as f:
                    local_version = json.load(f)
                    local_tag = local_version.get("version", "unknown")
                
                if tag == local_tag:
                    print(f"✅ Version {local_tag} is already installed.")
                    print("\nUse --force to re-download and rebuild.")
                    return 0
                else:
                    print(f"Local version ({local_tag}) differs from latest ({tag})")
                    print("Cleaning downloads directory...")
                    self.clean_downloads()
            except Exception as e:
                print(f"⚠️  Warning: Error reading version.json: {e}")
        
        # Download release assets
        assets = release.get("assets", [])
        downloaded_files = []
        
        if assets:
            for asset in assets:
                url = asset["browser_download_url"]
                filename = self.downloads_dir / asset["name"]
                self.download_file(url, filename)
                downloaded_files.append(filename)
        else:
            # Fallback to source zip
            source_url = release.get("zipball_url")
            filename = self.downloads_dir / f"grizzly_{tag}.zip"
            self.download_file(source_url, filename)
            downloaded_files.append(filename)
        
        print("\n✅ Download complete.")
        
        # Extract archives
        extract_path = self.extract_archives(downloaded_files)
        
        # Find source directory
        src_path = Path(extract_path) / "src"
        if not src_path.exists():
            print(f"❌ Error: src directory not found in {extract_path}")
            return 1
        
        # Build the workspace
        print(f"\n{'='*70}")
        print(f"  Building Workspace")
        print(f"{'='*70}\n")
        
        return self.build_workspace(src_path)
    
    def get_latest_release(self, repo):
        """Fetch the latest release from GitHub API."""
        url = f"https://api.github.com/repos/{repo}/releases"
        req = urllib.request.Request(url, headers={"User-Agent": "grizzly-cli"})
        
        try:
            with urllib.request.urlopen(req) as resp:
                releases = json.load(resp)
                # Find latest pre-release
                for release in releases:
                    if release.get("prerelease", False):
                        return release
                print("⚠️  No pre-releases found, using latest release")
                if releases:
                    return releases[0]
                return {}
        except urllib.error.HTTPError as e:
            print(f"❌ HTTP Error {e.code}: {e.reason}")
            return {}
    
    def download_file(self, url, filename):
        """Download a file from URL."""
        print(f"📥 Downloading {filename.name}...")
        try:
            urllib.request.urlretrieve(url, filename)
            print(f"✅ Saved: {filename}")
        except Exception as e:
            print(f"❌ Error downloading {filename}: {e}")
            raise
    
    def extract_archives(self, files):
        """Extract tar.gz and zip files."""
        extract_path = self.downloads_dir
        
        for filepath in files:
            filepath = Path(filepath)
            if filepath.suffix in ['.gz', '.tgz'] or str(filepath).endswith('.tar.gz'):
                print(f"📦 Extracting {filepath.name}...")
                try:
                    with tarfile.open(filepath, 'r:gz') as tar:
                        tar.extractall(path=extract_path)
                    print(f"✅ Extracted {filepath.name}")
                except Exception as e:
                    print(f"❌ Error extracting {filepath}: {e}")
            elif filepath.suffix == '.zip':
                print(f"📦 Extracting {filepath.name}...")
                try:
                    import zipfile
                    with zipfile.ZipFile(filepath, 'r') as zip_ref:
                        zip_ref.extractall(extract_path)
                    print(f"✅ Extracted {filepath.name}")
                except Exception as e:
                    print(f"❌ Error extracting {filepath}: {e}")
        
        return extract_path
    
    def clean_downloads(self):
        """Clean the downloads directory."""
        if self.downloads_dir.exists():
            for item in self.downloads_dir.iterdir():
                if item.is_file():
                    item.unlink()
                elif item.is_dir():
                    shutil.rmtree(item)
    
    def build_workspace(self, working_dir):
        """Build the workspace using platform-specific methods."""
        if self.system == "Darwin":
            return self.build_macos(working_dir)
        elif self.system == "Linux":
            return self.build_linux(working_dir)
        else:
            print(f"❌ Unsupported platform: {self.system}")
            return 1
    
    # ========================================================================
    # BUILD COMMAND
    # ========================================================================
    
    def cmd_build(self, args):
        """Build command - build the workspace."""
        print(f"\n{'='*70}")
        print(f"  Building Grizzly Workspace")
        print(f"{'='*70}")
        print(f"Platform: {self.system}")
        print(f"Workspace: {self.workspace_root}")
        print(f"{'='*70}\n")
        
        # Clean if requested
        if args.clean:
            print("🧹 Cleaning build directories...")
            for dir_name in ['build', 'install', 'log']:
                dir_path = self.workspace_root / dir_name
                if dir_path.exists():
                    print(f"   Removing {dir_name}/")
                    shutil.rmtree(dir_path)
            print("✅ Clean complete\n")
        
        # Build based on platform
        if self.system == "Darwin":
            return self.build_macos(self.workspace_root, args)
        elif self.system == "Linux":
            return self.build_linux(self.workspace_root, args)
        elif self.system == "Windows":
            return self.build_windows(self.workspace_root, args)
        else:
            print(f"❌ Unsupported platform: {self.system}")
            return 1
    
    def build_macos(self, working_dir, args=None):
        """Build on macOS using conda environment."""
        print("🍎 Building for macOS (conda environment)...\n")
        
        # Check for conda
        if not shutil.which('conda'):
            print("❌ Conda not found. macOS builds require conda with robostack.")
            print("\nInstall conda from: https://docs.conda.io/en/latest/miniconda.html")
            return 1
        
        # Build colcon command
        colcon_args = [
            'colcon', 'build',
            '--cmake-args',
            '-DPython_EXECUTABLE=/opt/anaconda3/envs/ros_env/bin/python',
            '-DPython_INCLUDE_DIR=/opt/anaconda3/envs/ros_env/include/python3.11',
            '-DPython_LIBRARY=/opt/anaconda3/envs/ros_env/lib/libpython3.11.dylib'
        ]
        
        if args and args.symlink_install:
            colcon_args.append('--symlink-install')
        
        if args and args.packages_select:
            colcon_args.extend(['--packages-select'] + args.packages_select)
        
        # Build the shell command with conda activation
        shell_cmd = f"""
eval "$(conda shell.zsh hook)"
conda activate ros_env
source /opt/anaconda3/envs/ros_env/setup.zsh
cd {working_dir}
{' '.join(colcon_args)}
source install/setup.zsh
"""
        
        try:
            result = subprocess.run(
                shell_cmd,
                shell=True,
                executable='/bin/zsh',
                check=True
            )
            print(f"\n{'='*70}")
            print("✅ Build completed successfully")
            print(f"{'='*70}\n")
            return 0
        except subprocess.CalledProcessError as e:
            print(f"\n{'='*70}")
            print(f"❌ Build failed with exit code {e.returncode}")
            print(f"{'='*70}\n")
            print("This may be due to:")
            print("  - Conda environment 'ros_env' not found")
            print("  - ROS 2 not properly installed in conda")
            print("  - Missing dependencies")
            return 1
    
    def build_linux(self, working_dir, args=None):
        """Build on Linux using native ROS 2."""
        print("🐧 Building for Linux (native ROS 2)...\n")
        
        # Check for colcon
        if not shutil.which('colcon'):
            print("❌ colcon not found.")
            print("Install with: sudo apt install python3-colcon-common-extensions")
            return 1
        
        # Check for ROS 2
        ros_setup = Path("/opt/ros/humble/setup.bash")
        if not ros_setup.exists():
            print(f"❌ ROS 2 Humble not found at {ros_setup}")
            print("Install from: https://docs.ros.org/en/humble/Installation.html")
            return 1
        
        # Build type
        build_type = "Release" if (args and args.release) else "RelWithDebInfo"
        
        # Build colcon command
        colcon_cmd = f'colcon build --cmake-args -DCMAKE_BUILD_TYPE={build_type}'
        
        if args and args.symlink_install:
            colcon_cmd += ' --symlink-install'
        
        if args and args.packages_select:
            colcon_cmd += ' --packages-select ' + ' '.join(args.packages_select)
        
        # Build shell script
        commands = [
            "set -euo pipefail",
            f"source {ros_setup}",
        ]
        
        # Run rosdep if not skipped
        if not (args and args.no_rosdep) and shutil.which('rosdep'):
            print("📦 Installing dependencies with rosdep...\n")
            commands.extend([
                "set +e",
                "rosdep update 2>/dev/null || true",
                "rosdep install --from-paths . --ignore-src -r -y || echo 'rosdep completed'",
                "set -e",
            ])
        
        commands.append(f'echo "Building with {build_type}..."')
        commands.append(colcon_cmd)
        
        script = " && ".join(commands)
        
        try:
            subprocess.run(
                script,
                shell=True,
                executable='/bin/bash',
                check=True,
                cwd=str(working_dir)
            )
            print(f"\n{'='*70}")
            print("✅ Build completed successfully")
            print(f"{'='*70}")
            print("\nTo use this workspace:")
            print(f"  source {working_dir}/install/setup.bash")
            print(f"{'='*70}\n")
            return 0
        except subprocess.CalledProcessError as e:
            print(f"\n{'='*70}")
            print(f"❌ Build failed with exit code {e.returncode}")
            print(f"{'='*70}\n")
            return 1
    
    def build_windows(self, working_dir, args=None):
        """Build on Windows using native ROS 2."""
        print("🪟 Building for Windows (native ROS 2)...\n")
        
        # Check for colcon
        if not shutil.which('colcon'):
            print("❌ colcon not found.")
            print("Make sure ROS 2 Humble is installed and sourced.")
            return 1
        
        build_type = "Release" if (args and args.release) else "RelWithDebInfo"
        
        colcon_cmd = [
            'colcon', 'build',
            '--cmake-args', f'-DCMAKE_BUILD_TYPE={build_type}'
        ]
        
        if args and args.symlink_install:
            colcon_cmd.append('--symlink-install')
        
        if args and args.packages_select:
            colcon_cmd.extend(['--packages-select'] + args.packages_select)
        
        try:
            subprocess.run(
                colcon_cmd,
                check=True,
                cwd=str(working_dir)
            )
            print(f"\n{'='*70}")
            print("✅ Build completed successfully")
            print(f"{'='*70}\n")
            return 0
        except subprocess.CalledProcessError as e:
            print(f"\n{'='*70}")
            print(f"❌ Build failed with exit code {e.returncode}")
            print(f"{'='*70}\n")
            return 1
    
    # ========================================================================
    # WORKSPACE VERIFICATION
    # ========================================================================
    
    def check_and_source_workspace(self):
        """Verify that the workspace is built and can be sourced."""
        if not self.install_dir.exists():
            print(f"\n{'='*70}")
            print("❌ ERROR: Workspace not built")
            print(f"{'='*70}")
            print(f"\nInstall directory not found: {self.install_dir}")
            print("\nThe workspace must be built before running or testing.")
            print("\nPlease run one of:")
            print("  ./grizzly.py build        # Build the workspace")
            print("  ./grizzly.py install      # Install and build latest release")
            print(f"{'='*70}\n")
            return False
        
        # Check for platform-specific setup script
        if self.system == "Darwin":
            setup_script = self.install_dir / "setup.zsh"
        elif self.system == "Linux":
            setup_script = self.install_dir / "setup.bash"
        elif self.system == "Windows":
            setup_script = self.install_dir / "setup.bat"
        else:
            print(f"❌ Unsupported platform: {self.system}")
            return False
        
        if not setup_script.exists():
            print(f"\n{'='*70}")
            print("❌ ERROR: Setup script not found")
            print(f"{'='*70}")
            print(f"\nExpected setup script: {setup_script}")
            print("\nThe workspace may be corrupted or incomplete.")
            print("\nPlease rebuild:")
            print("  ./grizzly.py build --clean")
            print(f"{'='*70}\n")
            return False
        
        return True
    
    # ========================================================================
    # CLEAN COMMAND
    # ========================================================================
    
    def cmd_clean(self, args):
        """Clean command - remove build artifacts and downloads."""
        print(f"\n{'='*70}")
        print(f"  Grizzly Clean")
        print(f"{'='*70}")
        print(f"Workspace: {self.workspace_root}")
        print(f"{'='*70}\n")
        
        # Determine what to clean
        clean_build_artifacts = args.all or args.build or (not args.downloads and not args.all)
        clean_downloads_dir = args.all or args.downloads
        
        removed_items = []
        
        # Clean build artifacts
        if clean_build_artifacts:
            print("🧹 Cleaning build artifacts...\n")
            for dir_name in ['build', 'install', 'log']:
                dir_path = self.workspace_root / dir_name
                if dir_path.exists():
                    print(f"   Removing {dir_name}/")
                    shutil.rmtree(dir_path)
                    removed_items.append(dir_name)
                else:
                    print(f"   ℹ️  {dir_name}/ does not exist")
        
        # Clean downloads
        if clean_downloads_dir:
            print("\n🧹 Cleaning downloads...\n")
            if self.downloads_dir.exists():
                print(f"   Removing downloads/")
                shutil.rmtree(self.downloads_dir)
                removed_items.append('downloads')
            else:
                print(f"   ℹ️  downloads/ does not exist")
        
        # Summary
        print(f"\n{'='*70}")
        if removed_items:
            print("✅ Clean complete")
            print(f"\nRemoved: {', '.join(removed_items)}")
        else:
            print("ℹ️  Nothing to clean")
        print(f"{'='*70}\n")
        
        return 0
    
    # ========================================================================
    # RUN COMMAND
    # ========================================================================
    
    def cmd_run(self, args):
        """Run command - launch the Grizzly system."""
        print(f"\n{'='*70}")
        print(f"  Grizzly Launch")
        print(f"{'='*70}")
        print(f"Platform: {self.system}")
        print(f"Workspace: {self.workspace_root}")
        print(f"{'='*70}\n")
        
        # Verify workspace is built
        if not self.check_and_source_workspace():
            return 1
        
        # Launch based on platform
        if self.system == "Darwin":
            return self.run_macos(args)
        elif self.system == "Linux":
            return self.run_linux(args)
        elif self.system == "Windows":
            return self.run_windows(args)
        else:
            print(f"❌ Unsupported platform: {self.system}")
            return 1
    
    def run_macos(self, args):
        """Run on macOS using conda environment."""
        print("🍎 Launching on macOS (conda environment)...\n")
        
        launch_cmd = """
eval "$(conda shell.zsh hook)"
conda activate ros_env
source /opt/anaconda3/envs/ros_env/setup.zsh

# Source workspace - fail if not found
if [ ! -f "install/setup.zsh" ]; then
    echo ""
    echo "======================================================================"
    echo "❌ ERROR: Workspace setup script not found"
    echo "======================================================================"
    echo ""
    echo "Expected: install/setup.zsh"
    echo ""
    echo "Please build the workspace first:"
    echo "  ./grizzly.py build"
    echo ""
    echo "======================================================================"
    echo ""
    exit 1
fi

source install/setup.zsh

echo ""
echo "================================================"
echo "  Launching Grizzly Minimal System"
echo "================================================"
echo ""
echo "Workspace sourced: install/setup.zsh"
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
                launch_cmd,
                shell=True,
                executable='/bin/zsh',
                cwd=str(self.workspace_root),
                check=True
            )
            return 0
        except subprocess.CalledProcessError as e:
            print(f"\n❌ Launch failed with exit code {e.returncode}")
            return 1
        except KeyboardInterrupt:
            print("\n\n✅ Grizzly stopped by user (Ctrl+C)")
            return 0
    
    def run_linux(self, args):
        """Run on Linux using native ROS 2."""
        print("🐧 Launching on Linux (native ROS 2)...\n")
        
        ros_setup = Path("/opt/ros/humble/setup.bash")
        
        launch_cmd = f"""
set -e
source {ros_setup}

# Source workspace - fail if not found
if [ ! -f "install/setup.bash" ]; then
    echo ""
    echo "======================================================================"
    echo "❌ ERROR: Workspace setup script not found"
    echo "======================================================================"
    echo ""
    echo "Expected: install/setup.bash"
    echo ""
    echo "Please build the workspace first:"
    echo "  ./grizzly.py build"
    echo ""
    echo "======================================================================"
    echo ""
    exit 1
fi

source install/setup.bash

echo ""
echo "================================================"
echo "  Launching Grizzly Minimal System"
echo "================================================"
echo ""
echo "Workspace sourced: install/setup.bash"
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
                launch_cmd,
                shell=True,
                executable='/bin/bash',
                cwd=str(self.workspace_root),
                check=True
            )
            return 0
        except subprocess.CalledProcessError as e:
            print(f"\n❌ Launch failed with exit code {e.returncode}")
            return 1
        except KeyboardInterrupt:
            print("\n\n✅ Grizzly stopped by user (Ctrl+C)")
            return 0
    
    def run_windows(self, args):
        """Run on Windows using native ROS 2."""
        print("🪟 Launching on Windows (native ROS 2)...\n")
        
        # Check setup script exists
        setup_script = self.install_dir / "setup.bat"
        if not setup_script.exists():
            print(f"\n{'='*70}")
            print("❌ ERROR: Workspace setup script not found")
            print(f"{'='*70}")
            print(f"\nExpected: {setup_script}")
            print("\nPlease build the workspace first:")
            print("  ./grizzly.py build")
            print(f"{'='*70}\n")
            return 1
        
        try:
            # Source and launch
            subprocess.run(
                f'call {setup_script} && ros2 launch grizzly_stack grizzly_minimal.launch.py',
                shell=True,
                cwd=str(self.workspace_root),
                check=True
            )
            return 0
        except subprocess.CalledProcessError as e:
            print(f"\n❌ Launch failed with exit code {e.returncode}")
            return 1
        except KeyboardInterrupt:
            print("\n\n✅ Grizzly stopped by user (Ctrl+C)")
            return 0
    
    # ========================================================================
    # TEST COMMAND
    # ========================================================================
    
    def cmd_test(self, args):
        """Test command - run the test suite."""
        print(f"\n{'='*70}")
        print(f"  Grizzly Test Suite")
        print(f"{'='*70}")
        print(f"Platform: {self.system}")
        print(f"Test Directory: {self.test_dir}")
        print(f"{'='*70}\n")
        
        # Verify workspace is built
        if not self.check_and_source_workspace():
            return 1
        
        # Check environment
        if not self.check_test_environment():
            return 1
        
        # Handle list command
        if args.list:
            return self.list_tests()
        
        # Source workspace before running tests
        print("🔧 Sourcing workspace environment...\n")
        env = os.environ.copy()
        
        # Add workspace to environment
        if self.system == "Darwin":
            source_cmd = """
eval "$(conda shell.zsh hook)"
conda activate ros_env
source /opt/anaconda3/envs/ros_env/setup.zsh
source install/setup.zsh
env
"""
            try:
                result = subprocess.run(
                    source_cmd,
                    shell=True,
                    executable='/bin/zsh',
                    cwd=str(self.workspace_root),
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if '=' in line:
                            key, _, value = line.partition('=')
                            env[key] = value
            except Exception as e:
                print(f"⚠️  Warning: Could not source environment: {e}")
        
        elif self.system == "Linux":
            ros_setup = Path("/opt/ros/humble/setup.bash")
            source_cmd = f"""
source {ros_setup}
source install/setup.bash
env
"""
            try:
                result = subprocess.run(
                    source_cmd,
                    shell=True,
                    executable='/bin/bash',
                    cwd=str(self.workspace_root),
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                if result.returncode == 0:
                    for line in result.stdout.split('\n'):
                        if '=' in line:
                            key, _, value = line.partition('=')
                            env[key] = value
            except Exception as e:
                print(f"⚠️  Warning: Could not source environment: {e}")
        
        # Build pytest command
        pytest_args = ['python3', '-m', 'pytest']
        
        if args.specific:
            # Run specific test
            test_path = self.test_dir / args.specific
            if not test_path.exists():
                test_path = self.test_dir / f"{args.specific}.py"
            if not test_path.exists():
                print(f"❌ Test not found: {args.specific}")
                return 1
            pytest_args.append(str(test_path))
        else:
            # Run all tests
            pytest_args.append(str(self.test_dir))
        
        # Add verbosity
        if args.verbose:
            pytest_args.append('-v')
        else:
            pytest_args.append('-q')
        
        # Add coverage
        if args.coverage or args.html:
            pytest_args.extend(['--cov=grizzly_stack', '--cov-report=term'])
            if args.html:
                pytest_args.append('--cov-report=html')
        
        # Run tests
        try:
            print("🧪 Running tests...\n")
            result = subprocess.run(
                pytest_args,
                cwd=str(self.workspace_root),
                env=env
            )
            
            if result.returncode == 0:
                print(f"\n{'='*70}")
                print("✅ All tests passed")
                print(f"{'='*70}\n")
            else:
                print(f"\n{'='*70}")
                print("❌ Some tests failed")
                print(f"{'='*70}\n")
            
            return result.returncode
        except Exception as e:
            print(f"❌ Error running tests: {e}")
            return 1
    
    def check_test_environment(self):
        """Check if test environment is ready."""
        # Check test directory
        if not self.test_dir.exists():
            print(f"❌ Test directory not found: {self.test_dir}")
            return False
        
        # Check pytest
        try:
            result = subprocess.run(
                ['python3', '-m', 'pytest', '--version'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode != 0:
                print("❌ pytest not found. Install with:")
                print("   pip3 install pytest pytest-cov")
                return False
        except Exception as e:
            print(f"❌ Error checking pytest: {e}")
            return False
        
        return True
    
    def list_tests(self):
        """List all available tests."""
        print("📋 Available Tests:\n")
        
        test_files = sorted(self.test_dir.glob("test_*.py"))
        
        if not test_files:
            print("No test files found.")
            return 0
        
        for test_file in test_files:
            print(f"📄 {test_file.name}")
            
            try:
                result = subprocess.run(
                    ['python3', '-m', 'pytest', '--collect-only', '-q', str(test_file)],
                    capture_output=True,
                    text=True,
                    timeout=10,
                    cwd=str(self.workspace_root)
                )
                
                if result.returncode == 0:
                    lines = result.stdout.strip().split('\n')
                    for line in lines:
                        if '::' in line and 'test_' in line:
                            print(f"   └─ {line.strip()}")
            except Exception:
                pass
            
            print()
        
        return 0


def main():
    """Entry point for the CLI."""
    cli = GrizzlyCLI()
    try:
        sys.exit(cli.main())
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
