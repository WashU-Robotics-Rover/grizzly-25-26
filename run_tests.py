#!/usr/bin/env python3
"""
Grizzly Automated Test Suite Runner

This script runs all tests in the grizzly_stack test suite with various options
for verbosity, coverage reporting, and output formatting.

Usage:
    ./run_tests.py                    # Run all tests with standard output
    ./run_tests.py -v                 # Run with verbose output
    ./run_tests.py --coverage         # Run with coverage report
    ./run_tests.py --html             # Generate HTML coverage report
    ./run_tests.py --specific <test>  # Run specific test file or test case
    ./run_tests.py --list             # List all available tests
"""

import sys
import os
import subprocess
import argparse
from pathlib import Path


class GrizzlyTestRunner:
    """Automated test suite runner for Grizzly ROS 2 package."""
    
    def __init__(self):
        self.workspace_root = Path(__file__).parent.resolve()
        self.test_dir = self.workspace_root / "grizzly_stack" / "test"
        self.setup_script = self.workspace_root / "install" / "setup.zsh"
        
    def check_environment(self):
        """Verify ROS 2 environment and pytest are available."""
        print("🔍 Checking test environment...")
        
        # Check if test directory exists
        if not self.test_dir.exists():
            print(f"❌ Test directory not found: {self.test_dir}")
            return False
        
        # Check if install directory exists
        if not self.setup_script.exists():
            print(f"❌ ROS 2 install not found. Please build the workspace first:")
            print(f"   ./build.sh")
            return False
        
        # Check if pytest is available
        try:
            result = subprocess.run(
                ["python3", "-m", "pytest", "--version"],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode != 0:
                print("❌ pytest not found. Install with:")
                print("   pip3 install pytest pytest-cov")
                return False
            print(f"✅ Found {result.stdout.strip()}")
        except Exception as e:
            print(f"❌ Error checking pytest: {e}")
            return False
        
        print("✅ Environment check passed\n")
        return True
    
    def list_tests(self):
        """List all available test files and test cases."""
        print("📋 Available Tests:\n")
        
        test_files = sorted(self.test_dir.glob("test_*.py"))
        
        if not test_files:
            print("No test files found.")
            return
        
        for test_file in test_files:
            print(f"📄 {test_file.name}")
            
            # Try to collect test cases from the file
            try:
                result = subprocess.run(
                    ["python3", "-m", "pytest", "--collect-only", "-q", str(test_file)],
                    capture_output=True,
                    text=True,
                    timeout=10,
                    cwd=str(self.workspace_root)
                )
                
                if result.returncode == 0:
                    lines = result.stdout.strip().split('\n')
                    test_count = 0
                    for line in lines:
                        if '::' in line and 'test_' in line:
                            test_count += 1
                            print(f"   └─ {line.strip()}")
                    
                    if test_count == 0:
                        print("   └─ (no tests found or file is empty)")
                else:
                    print(f"   └─ (unable to collect tests)")
                    
            except Exception as e:
                print(f"   └─ (error collecting: {e})")
            
            print()
    
    def run_tests(self, args):
        """Run the test suite with specified options."""
        print("🚀 Starting Grizzly Test Suite\n")
        print("=" * 70)
        
        # Build pytest command
        pytest_cmd = ["python3", "-m", "pytest"]
        
        # Determine which tests to run
        if args.specific:
            test_path = args.specific
            if not test_path.startswith("grizzly_stack/test/"):
                test_path = f"grizzly_stack/test/{test_path}"
            pytest_cmd.append(test_path)
            print(f"🎯 Running specific test: {test_path}")
        else:
            pytest_cmd.append("grizzly_stack/test/")
            print(f"📦 Running all tests in: grizzly_stack/test/")
        
        # Add verbosity
        if args.verbose:
            pytest_cmd.append("-v")
        else:
            pytest_cmd.append("-v")  # Always use verbose by default
        
        # Add coverage options
        if args.coverage or args.html:
            pytest_cmd.extend([
                "--cov=grizzly_stack",
                "--cov-report=term"
            ])
            
            if args.html:
                pytest_cmd.append("--cov-report=html")
                print("📊 Coverage report will be generated in htmlcov/")
        
        # Add output options
        if args.show_output:
            pytest_cmd.append("-s")  # Don't capture output
        
        # Add fail fast option
        if args.fail_fast:
            pytest_cmd.append("-x")
        
        # Add color output
        pytest_cmd.append("--color=yes")
        
        print("=" * 70)
        print()
        
        # Run pytest
        try:
            result = subprocess.run(
                pytest_cmd,
                cwd=str(self.workspace_root),
                env=self._get_ros_env()
            )
            
            print("\n" + "=" * 70)
            
            if result.returncode == 0:
                print("✅ All tests passed!")
                if args.html:
                    print(f"📊 Open coverage report: {self.workspace_root}/htmlcov/index.html")
            else:
                print("❌ Some tests failed. See output above for details.")
            
            print("=" * 70)
            
            return result.returncode
            
        except KeyboardInterrupt:
            print("\n⚠️  Tests interrupted by user")
            return 130
        except Exception as e:
            print(f"\n❌ Error running tests: {e}")
            return 1
    
    def _get_ros_env(self):
        """Get environment variables with ROS 2 setup sourced."""
        env = os.environ.copy()
        
        # Add Python path for the workspace
        python_path = str(self.workspace_root / "install" / "grizzly_stack" / "lib" / "python3.12" / "site-packages")
        if os.path.exists(python_path):
            if "PYTHONPATH" in env:
                env["PYTHONPATH"] = f"{python_path}:{env['PYTHONPATH']}"
            else:
                env["PYTHONPATH"] = python_path
        
        # Add common Python paths for ROS 2
        ros_python_paths = [
            "/opt/ros/jazzy/lib/python3.12/site-packages",
            "/opt/ros/jazzy/local/lib/python3.12/dist-packages"
        ]
        
        for path in ros_python_paths:
            if os.path.exists(path):
                if "PYTHONPATH" in env:
                    env["PYTHONPATH"] = f"{env['PYTHONPATH']}:{path}"
                else:
                    env["PYTHONPATH"] = path
        
        return env


def main():
    """Main entry point for the test runner."""
    parser = argparse.ArgumentParser(
        description="Grizzly Automated Test Suite Runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                           # Run all tests
  %(prog)s -v                        # Run with verbose output
  %(prog)s --coverage                # Run with coverage report
  %(prog)s --html                    # Generate HTML coverage report
  %(prog)s --specific test_basic.py  # Run specific test file
  %(prog)s --list                    # List all available tests
  %(prog)s -x                        # Stop at first failure
        """
    )
    
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose test output"
    )
    
    parser.add_argument(
        "--coverage",
        action="store_true",
        help="Generate coverage report"
    )
    
    parser.add_argument(
        "--html",
        action="store_true",
        help="Generate HTML coverage report (implies --coverage)"
    )
    
    parser.add_argument(
        "--specific",
        metavar="TEST",
        help="Run specific test file or test case (e.g., test_basic.py or test_system_manager.py::TestSystemManagerStateMachine::test_initial_state)"
    )
    
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all available tests without running them"
    )
    
    parser.add_argument(
        "-s", "--show-output",
        action="store_true",
        help="Show print statements and logging output"
    )
    
    parser.add_argument(
        "-x", "--fail-fast",
        action="store_true",
        help="Stop at first test failure"
    )
    
    args = parser.parse_args()
    
    # Create test runner
    runner = GrizzlyTestRunner()
    
    # Check environment
    if not runner.check_environment():
        return 1
    
    # Handle list command
    if args.list:
        runner.list_tests()
        return 0
    
    # Run tests
    return runner.run_tests(args)


if __name__ == "__main__":
    sys.exit(main())
