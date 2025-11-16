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
    
    def get_conda_env_info(self):
        """
        Detect the active conda environment name and prefix.
        Returns (env_name, env_prefix) tuple or (None, None) if not found.
        """
        conda_env = os.environ.get('CONDA_DEFAULT_ENV')
        conda_prefix = os.environ.get('CONDA_PREFIX')
        
        if not conda_env or not conda_prefix:
            # Try to get conda info if not in environment
            try:
                conda_info = subprocess.run(
                    ['conda', 'info', '--json'],
                    capture_output=True,
                    text=True,
                    check=True
                )
                conda_data = json.loads(conda_info.stdout)
                conda_env = conda_data.get('active_prefix_name') or conda_data.get('default_prefix', '').split('/')[-1]
                conda_prefix = conda_data.get('active_prefix') or conda_data.get('default_prefix')
            except (subprocess.CalledProcessError, json.JSONDecodeError, KeyError):
                return None, None
        
        return conda_env, conda_prefix
        
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
  update       Fetch and update from the latest GitHub commit on main
  web          Launch web interface with rosbridge server

Examples:
  ./grizzly.py install              # Download and build latest release
  ./grizzly.py build                # Build the current workspace
  ./grizzly.py build --clean        # Clean build from scratch
  ./grizzly.py run                  # Launch the system
  ./grizzly.py web                  # Launch web interface (auto-starts rosbridge)
  ./grizzly.py web --port 3000      # Launch web interface on custom port
  ./grizzly.py web --no-rosbridge   # Launch only web server
  ./grizzly.py test                 # Run all tests
  ./grizzly.py test -v              # Run tests with verbose output
  ./grizzly.py test --coverage      # Run tests with coverage report
  ./grizzly.py update               # Update repository from main branch

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
        
        # Update command
        update_parser = subparsers.add_parser('update', help='Fetch and update from latest GitHub commit on main')
        update_parser.add_argument('--branch', default='main',
                                  help='Branch to update from (default: main)')
        update_parser.add_argument('--rebuild', action='store_true',
                                  help='Rebuild after updating')
        
        # Web command
        web_parser = subparsers.add_parser('web', help='Launch web interface with rosbridge')
        web_parser.add_argument('--port', type=int, default=8000,
                               help='Port for web server (default: 8000)')
        web_parser.add_argument('--rosbridge-port', type=int, default=9090,
                               help='Port for rosbridge server (default: 9090)')
        web_parser.add_argument('--no-rosbridge', action='store_true',
                               help='Launch only the web server (assume rosbridge is already running)')
        web_parser.add_argument('--no-browser', action='store_true',
                               help='Do not open browser automatically')
        
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
        elif args.command == 'update':
            return self.cmd_update(args)
        elif args.command == 'web':
            return self.cmd_web(args)
    
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
        version_file = self.workspace_root / "version.json"
        
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
            except Exception as e:
                print(f"⚠️  Warning: Error reading version.json: {e}")
        
        # Create temporary directory for download and build
        import tempfile
        temp_dir = Path(tempfile.mkdtemp(prefix="grizzly_install_"))
        print(f"📁 Using temporary directory: {temp_dir}\n")
        
        try:
            # Download release assets to temp directory
            assets = release.get("assets", [])
            downloaded_files = []
            
            if assets:
                for asset in assets:
                    url = asset["browser_download_url"]
                    filename = temp_dir / asset["name"]
                    self.download_file(url, filename)
                    downloaded_files.append(filename)
            else:
                # Fallback to source zip
                source_url = release.get("zipball_url")
                filename = temp_dir / f"grizzly_{tag}.zip"
                self.download_file(source_url, filename)
                downloaded_files.append(filename)
            
            print("\n✅ Download complete.")
            
            # Extract archives to temp directory
            print(f"\n📦 Extracting to temporary directory...")
            extract_path = self.extract_archives_to_path(downloaded_files, temp_dir)
            
            # Find source directory
            src_path = self.find_source_directory(extract_path)
            if not src_path:
                print(f"❌ Error: Could not find source directory in {extract_path}")
                return 1
            
            print(f"✅ Found source directory: {src_path}\n")
            
            # Build the workspace in temp directory
            print(f"\n{'='*70}")
            print(f"  Building Workspace")
            print(f"{'='*70}\n")
            
            build_result = self.build_workspace(src_path)
            if build_result != 0:
                print("\n❌ Build failed")
                return 1
            
            # Copy install directory to workspace
            print(f"\n{'='*70}")
            print(f"  Installing to Workspace")
            print(f"{'='*70}\n")
            
            temp_install_dir = src_path / "install"
            if not temp_install_dir.exists():
                print(f"❌ Error: Built install directory not found at {temp_install_dir}")
                return 1
            
            # Remove old install directory if it exists
            if self.install_dir.exists():
                print(f"🧹 Removing old install directory...")
                shutil.rmtree(self.install_dir)
            
            # Copy new install directory
            print(f"📦 Copying install directory to workspace...")
            shutil.copytree(temp_install_dir, self.install_dir)
            print(f"✅ Installed to: {self.install_dir}")
            
            # Save version info
            print(f"\n💾 Saving version information...")
            with open(version_file, 'w') as f:
                json.dump({
                    "version": tag,
                    "name": name,
                    "installed_at": subprocess.run(
                        ["date", "+%Y-%m-%d %H:%M:%S"],
                        capture_output=True,
                        text=True
                    ).stdout.strip()
                }, f, indent=2)
            print(f"✅ Version saved: {tag}")
            
            print(f"\n{'='*70}")
            print("✅ Installation complete")
            print(f"{'='*70}\n")
            
            return 0
            
        finally:
            # Clean up temporary directory
            print(f"\n🧹 Cleaning up temporary directory...")
            try:
                shutil.rmtree(temp_dir)
                print(f"✅ Temporary directory removed")
            except Exception as e:
                print(f"⚠️  Warning: Could not remove temp directory: {e}")
    
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
    
    def extract_archives_to_path(self, files, extract_path):
        """Extract tar.gz and zip files to a specific path."""
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
                    raise
            elif filepath.suffix == '.zip':
                print(f"📦 Extracting {filepath.name}...")
                try:
                    import zipfile
                    with zipfile.ZipFile(filepath, 'r') as zip_ref:
                        zip_ref.extractall(extract_path)
                    print(f"✅ Extracted {filepath.name}")
                except Exception as e:
                    print(f"❌ Error extracting {filepath}: {e}")
                    raise
        
        return extract_path
    
    def find_source_directory(self, base_path):
        """Find the source directory containing ROS packages."""
        base_path = Path(base_path)
        
        # Check if base_path itself is the source directory
        if (base_path / "grizzly_stack").exists() or (base_path / "grizzly_interfaces").exists():
            return base_path
        
        # Check for src subdirectory
        src_path = base_path / "src"
        if src_path.exists():
            return src_path
        
        # Check subdirectories (for GitHub zipball structure like "WashU-Robotics-Rover-grizzly-25-26-abc123")
        for item in base_path.iterdir():
            if item.is_dir():
                # Check if this directory contains packages
                if (item / "grizzly_stack").exists() or (item / "grizzly_interfaces").exists():
                    return item
                # Check if this directory has a src subdirectory
                src_candidate = item / "src"
                if src_candidate.exists():
                    return src_candidate
                # Recursively check one level deeper
                for subitem in item.iterdir():
                    if subitem.is_dir():
                        if (subitem / "grizzly_stack").exists() or (subitem / "grizzly_interfaces").exists():
                            return subitem
        
        return None
    
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
        
        # Detect active conda environment
        conda_env, conda_prefix = self.get_conda_env_info()
        
        if not conda_env or not conda_prefix:
            print("❌ Could not detect conda environment.")
            print("   Please activate your conda environment first:")
            print("   conda activate ros_env  # or ros_env_test")
            return 1
        
        print(f"📦 Using conda environment: {conda_env}")
        print(f"   Path: {conda_prefix}\n")
        
        # Detect Python version and paths
        python_exe = shutil.which('python3') or shutil.which('python')
        if not python_exe:
            print("❌ Python not found in PATH")
            return 1
        
        # Get Python version
        try:
            python_version = subprocess.run(
                [python_exe, '--version'],
                capture_output=True,
                text=True,
                check=True
            ).stdout.strip().split()[-1]
            python_major_minor = '.'.join(python_version.split('.')[:2])
        except (subprocess.CalledProcessError, IndexError):
            python_major_minor = '3.11'  # fallback
            print(f"⚠️  Could not detect Python version, using {python_major_minor} as fallback")
        
        # Construct Python paths based on detected environment
        python_executable = f"{conda_prefix}/bin/python"
        python_include_dir = f"{conda_prefix}/include/python{python_major_minor}"
        python_library = f"{conda_prefix}/lib/libpython{python_major_minor}.dylib"
        
        # Verify paths exist
        if not os.path.exists(python_executable):
            print(f"❌ Python executable not found: {python_executable}")
            return 1
        if not os.path.exists(python_include_dir):
            print(f"⚠️  Python include directory not found: {python_include_dir}")
            print("   Attempting build anyway...")
        
        # Build colcon command
        colcon_args = [
            'colcon', 'build',
            '--cmake-args',
            f'-DPython_EXECUTABLE={python_executable}',
            f'-DPython_INCLUDE_DIR={python_include_dir}',
            f'-DPython_LIBRARY={python_library}'
        ]
        
        if args and args.symlink_install:
            colcon_args.append('--symlink-install')
        
        if args and args.packages_select:
            colcon_args.extend(['--packages-select'] + args.packages_select)
        
        # Build the shell command with conda activation
        setup_script = f"{conda_prefix}/setup.zsh"
        shell_cmd = f"""
eval "$(conda shell.zsh hook)"
conda activate {conda_env}
{'source ' + setup_script + ' 2>/dev/null || true'}
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
            print(f"  - Conda environment '{conda_env}' not properly configured")
            print("  - ROS 2 not properly installed in conda")
            print("  - Missing dependencies")
            print("  - Incorrect Python paths")
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
    # UPDATE COMMAND
    # ========================================================================
    
    def cmd_update(self, args):
        """Update command - fetch and update from latest GitHub commit on main."""
        print(f"\n{'='*70}")
        print(f"  Grizzly Update")
        print(f"{'='*70}")
        print(f"Workspace: {self.workspace_root}")
        print(f"Branch: {args.branch}")
        print(f"{'='*70}\n")
        
        # Check if we're in a git repository
        git_dir = self.workspace_root / ".git"
        if not git_dir.exists():
            print("❌ Error: Not a git repository")
            print("   This command only works in a git repository.")
            return 1
        
        try:
            # Save current branch
            print("📍 Saving current branch...")
            result = subprocess.run(
                ["git", "rev-parse", "--abbrev-ref", "HEAD"],
                cwd=self.workspace_root,
                capture_output=True,
                text=True,
                check=True
            )
            current_branch = result.stdout.strip()
            print(f"   Current branch: {current_branch}")
            
            # Fetch latest changes
            print(f"\n📥 Fetching latest changes from origin...")
            subprocess.run(
                ["git", "fetch", "origin"],
                cwd=self.workspace_root,
                check=True
            )
            print("✅ Fetch complete")
            
            # Check for uncommitted changes
            print(f"\n🔍 Checking for uncommitted changes...")
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=self.workspace_root,
                capture_output=True,
                text=True,
                check=True
            )
            
            has_changes = bool(result.stdout.strip())
            if has_changes:
                print("⚠️  Warning: You have uncommitted changes")
                print("\nUncommitted changes:")
                print(result.stdout)
                
                response = input("\nDo you want to stash these changes and continue? (y/n): ")
                if response.lower() != 'y':
                    print("\n❌ Update cancelled")
                    return 1
                
                print("\n📦 Stashing changes...")
                subprocess.run(
                    ["git", "stash", "push", "-m", f"Auto-stash before update to {args.branch}"],
                    cwd=self.workspace_root,
                    check=True
                )
                print("✅ Changes stashed")
            
            # Pull latest changes from specified branch
            print(f"\n⬇️  Pulling latest changes from {args.branch}...")
            subprocess.run(
                ["git", "pull", "origin", args.branch],
                cwd=self.workspace_root,
                check=True
            )
            print("✅ Pull complete")
            
            # Show what changed
            print(f"\n📊 Recent commits:")
            subprocess.run(
                ["git", "log", "--oneline", "-5"],
                cwd=self.workspace_root
            )
            
            # Optionally rebuild
            if args.rebuild:
                print(f"\n{'='*70}")
                print("  Rebuilding workspace...")
                print(f"{'='*70}\n")
                
                # Create a mock args object for build
                class BuildArgs:
                    clean = False
                    release = False
                    no_rosdep = False
                    symlink_install = False
                    packages_select = None
                
                build_result = self.cmd_build(BuildArgs())
                if build_result != 0:
                    print("\n❌ Build failed")
                    return 1
            
            # Summary
            print(f"\n{'='*70}")
            print("✅ Update complete")
            if has_changes:
                print("\n💡 Your changes were stashed. To restore them:")
                print("   git stash pop")
            if not args.rebuild:
                print("\n💡 To rebuild the workspace:")
                print("   ./grizzly.py build")
            print(f"{'='*70}\n")
            
            return 0
            
        except subprocess.CalledProcessError as e:
            print(f"\n❌ Git command failed: {e}")
            print("\nPlease resolve any conflicts and try again.")
            return 1
        except Exception as e:
            print(f"\n❌ Error during update: {e}")
            return 1
    
    # ========================================================================
    # WEB COMMAND
    # ========================================================================
    
    def cmd_web(self, args):
        """Web command - launch web interface with rosbridge server."""
        print(f"\n{'='*70}")
        print(f"  Grizzly Web Interface")
        print(f"{'='*70}")
        print(f"Platform: {self.system}")
        print(f"Workspace: {self.workspace_root}")
        print(f"Web Port: {args.port}")
        print(f"Rosbridge Port: {args.rosbridge_port}")
        print(f"{'='*70}\n")
        
        # Check if grizzly_command directory exists
        web_dir = self.workspace_root / "grizzly_command"
        if not web_dir.exists():
            print(f"❌ Error: grizzly_command directory not found")
            print(f"   Expected: {web_dir}")
            return 1
        
        # Check if index.html exists
        index_file = web_dir / "index.html"
        if not index_file.exists():
            print(f"❌ Error: index.html not found")
            print(f"   Expected: {index_file}")
            return 1
        
        # Verify workspace is built (needed for rosbridge to find grizzly_interfaces)
        if not args.no_rosbridge:
            if not self.check_and_source_workspace():
                print("\n❌ Workspace must be built before launching rosbridge")
                print("   Run: ./grizzly.py build")
                return 1
        
        import threading
        import http.server
        import socketserver
        import webbrowser
        import time
        
        # Flag to track if we should continue running
        keep_running = True
        rosbridge_process = None
        
        # Start rosbridge in background if requested
        if not args.no_rosbridge:
            print("🌉 Starting rosbridge_server...")
            print(f"   WebSocket URL: ws://localhost:{args.rosbridge_port}\n")
            
            # Build command to launch rosbridge with workspace sourced
            if self.system == "Darwin":
                conda_env, conda_prefix = self.get_conda_env_info()
                if not conda_env or not conda_prefix:
                    print("❌ Could not detect conda environment for rosbridge.")
                    print("   Please activate your conda environment first:")
                    print("   conda activate ros_env  # or ros_env_test")
                    return 1
                setup_script = f"{conda_prefix}/setup.zsh"
                rosbridge_cmd = f"""
eval "$(conda shell.zsh hook)"
conda activate {conda_env}
source {setup_script} 2>/dev/null || true
source {self.workspace_root}/install/setup.zsh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={args.rosbridge_port}
"""
                rosbridge_process = subprocess.Popen(
                    rosbridge_cmd,
                    shell=True,
                    executable="/bin/zsh",
                    cwd=self.workspace_root
                )
            elif self.system == "Linux":
                rosbridge_cmd = f"""
source /opt/ros/humble/setup.bash
source {self.workspace_root}/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={args.rosbridge_port}
"""
                rosbridge_process = subprocess.Popen(
                    rosbridge_cmd,
                    shell=True,
                    executable="/bin/bash",
                    cwd=self.workspace_root
                )
            else:
                print(f"⚠️  Warning: Automatic rosbridge launch not supported on {self.system}")
                print("   Please start rosbridge manually in another terminal:")
                print(f"   ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:={args.rosbridge_port}")
            
            # Give rosbridge time to start
            time.sleep(2)
        else:
            print("⚠️  Skipping rosbridge launch (--no-rosbridge specified)")
            print(f"   Make sure rosbridge is running on port {args.rosbridge_port}\n")
        
        # Create HTTP server with CORS support
        class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
            def end_headers(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                super().end_headers()
            
            def log_message(self, format, *args):
                # Suppress standard logging, we'll do our own
                pass
        
        # Start web server in a thread
        def run_server():
            nonlocal keep_running
            try:
                os.chdir(web_dir)
                with socketserver.TCPServer(("", args.port), CORSRequestHandler) as httpd:
                    print("🌐 Web server started!")
                    print(f"   URL: http://localhost:{args.port}")
                    print(f"   Directory: {web_dir}\n")
                    
                    while keep_running:
                        httpd.handle_request()
                        
            except OSError as e:
                if e.errno == 48 or e.errno == 98:  # Address already in use
                    print(f"\n❌ Error: Port {args.port} is already in use!")
                    print(f"   Try a different port: ./grizzly.py web --port {args.port + 1}")
                else:
                    print(f"\n❌ Error starting web server: {e}")
                keep_running = False
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
        # Give server time to start
        time.sleep(1)
        
        if not keep_running:
            # Server failed to start
            if rosbridge_process:
                rosbridge_process.terminate()
            return 1
        
        # Open browser if requested
        if not args.no_browser:
            print("🚀 Opening browser...")
            time.sleep(0.5)
            webbrowser.open(f"http://localhost:{args.port}")
        
        print(f"\n{'='*70}")
        print("✅ Grizzly Web Interface is running!")
        print(f"{'='*70}")
        print(f"\n📱 Access the interface at: http://localhost:{args.port}")
        if not args.no_rosbridge:
            print(f"🌉 Rosbridge WebSocket: ws://localhost:{args.rosbridge_port}")
        print("\n💡 Make sure the Grizzly system is running:")
        print("   ./grizzly.py run")
        print("\n⏹  Press Ctrl+C to stop\n")
        print(f"{'='*70}\n")
        
        # Keep main thread alive
        try:
            while keep_running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\n⏹  Shutting down...")
            keep_running = False
            
            # Terminate rosbridge if we started it
            if rosbridge_process:
                print("   Stopping rosbridge...")
                rosbridge_process.terminate()
                try:
                    rosbridge_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    rosbridge_process.kill()
            
            print("👋 Goodbye!\n")
        
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
        
        # Detect active conda environment
        conda_env, conda_prefix = self.get_conda_env_info()
        
        if not conda_env or not conda_prefix:
            print("❌ Could not detect conda environment.")
            print("   Please activate your conda environment first:")
            print("   conda activate ros_env  # or ros_env_test")
            return 1
        
        setup_script = f"{conda_prefix}/setup.zsh"
        launch_cmd = f"""
eval "$(conda shell.zsh hook)"
conda activate {conda_env}
source {setup_script} 2>/dev/null || true

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
            conda_env, conda_prefix = self.get_conda_env_info()
            if not conda_env or not conda_prefix:
                print("❌ Could not detect conda environment for tests.")
                print("   Please activate your conda environment first:")
                print("   conda activate ros_env  # or ros_env_test")
                return 1
            setup_script = f"{conda_prefix}/setup.zsh"
            source_cmd = f"""
eval "$(conda shell.zsh hook)"
conda activate {conda_env}
source {setup_script} 2>/dev/null || true
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
