# Testing Guide

A comprehensive guide for running tests in the Grizzly ROS 2 stack.

## Quick Start

```bash
# Run all tests
./grizzly.py test

# Run with coverage report
./grizzly.py test --coverage

# Run specific test file
./grizzly.py test --specific test_system_manager

# List all available tests
./grizzly.py test --list
```

## Features

✅ **Automatic Environment Detection**: Checks for ROS 2 installation and pytest  
✅ **Multiple Output Formats**: Standard, verbose, coverage, and HTML reports  
✅ **Flexible Test Selection**: Run all tests or specific test files/cases  
✅ **Color-Coded Output**: Easy-to-read test results  
✅ **Fast Execution**: Runs all tests in under 1 second  

## Installation

No additional installation needed! The `grizzly.py test` command uses the existing ROS 2 environment and pytest.

If pytest is not installed:
```bash
pip3 install pytest pytest-cov
```

## Usage

### Basic Commands

```bash
# Run all tests (verbose by default)
./grizzly.py test

# Run with standard output (less verbose)
./grizzly.py test -v

# Stop at first failure
./grizzly.py test --fail-fast
```

### Coverage Reports

```bash
# Terminal coverage report
./grizzly.py test --coverage

# Generate HTML coverage report
./grizzly.py test --html
# Then open: htmlcov/index.html
```

### Specific Tests

```bash
# Run specific test file
./grizzly.py test --specific test_system_manager

# Run specific test class
./grizzly.py test --specific "test_system_manager::TestSystemManagerStateMachine"

# Run specific test method
./grizzly.py test --specific "test_system_manager::TestSystemManagerStateMachine::test_initial_state"
```

### List Tests

```bash
# List all available tests
./grizzly.py test --list
```

## Test Suite Structure

Current test suite includes:

### System Manager Tests (`test_system_manager.py`)

**State Machine Tests (17 tests):**
- Initial state verification
- Valid state transitions
- Invalid transition rejection
- Emergency handling
- Service functionality
- State history logging
- Complete workflow testing

**Lifecycle Tests (4 tests):**
- ROS 2 lifecycle callbacks
- Configure, activate, deactivate, shutdown

### Basic Tests (`test_basic.py`)
- Basic functionality tests (ready for expansion)

## State Machine Test Coverage

### Valid Transitions Tested

| From State   | To States                                    |
|-------------|----------------------------------------------|
| STARTUP     | STANDBY, ERROR, EMERGENCY                    |
| STANDBY     | AUTONOMOUS, MANUAL, SHUTDOWN, EMERGENCY      |
| AUTONOMOUS  | STANDBY, MANUAL, EMERGENCY, ERROR            |
| MANUAL      | STANDBY, AUTONOMOUS, EMERGENCY, ERROR        |
| EMERGENCY   | STANDBY (only)                               |
| ERROR       | STANDBY, SHUTDOWN                            |
| SHUTDOWN    | (none - terminal state)                      |

## Command-Line Options

| Option | Description |
|--------|-------------|
| `-v, --verbose` | Enable verbose test output |
| `--coverage` | Generate coverage report |
| `--html` | Generate HTML coverage report |
| `--specific TEST` | Run specific test file or test case |
| `--list` | List all available tests |
| `--fail-fast` | Stop at first test failure |
| `-h, --help` | Show help message |

## Examples

### Example 1: Quick Test Run
```bash
./grizzly.py test
```
Output:
```
🔍 Checking test environment...
✅ Found pytest 8.3.4
✅ Environment check passed

🚀 Starting Grizzly Test Suite
======================================================================
📦 Running all tests in: grizzly_stack/test/
======================================================================
... 21 passed in 0.55s ...
✅ All tests passed!
```

### Example 2: Coverage Report
```bash
./grizzly.py test --html
```
Opens a detailed HTML coverage report showing which lines of code are tested.

### Example 3: Debug Specific Test
```bash
./grizzly.py test --specific "test_system_manager::TestSystemManagerStateMachine::test_emergency_override_from_any_state"
```
Runs only the emergency override test with full output visible.

## Integration with CI/CD

The test runner can be easily integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Run Grizzly Tests
  run: |
    source install/setup.bash
    ./grizzly.py test --coverage
```

## Troubleshooting

### Tests Won't Run
1. Make sure the workspace is built:
   ```bash
   ./grizzly.py build
   ```

2. Verify pytest is installed:
   ```bash
   pip3 install pytest pytest-cov
   ```

3. Check ROS 2 environment:
   ```bash
   source install/setup.bash  # Linux
   source install/setup.zsh   # macOS
   ```

### Import Errors
If you see import errors, ensure the workspace is sourced:
```bash
source install/setup.bash  # Linux
source install/setup.zsh   # macOS
./grizzly.py test
```

## Adding New Tests

1. Create a new test file in `grizzly_stack/test/` with the prefix `test_`
2. Write your test cases using Python's `unittest` or `pytest`
3. Run `./grizzly.py test --list` to verify your tests are discovered
4. Run `./grizzly.py test` to execute all tests including your new ones

Example test structure:
```python
import unittest

class TestMyFeature(unittest.TestCase):
    def test_something(self):
        """Test description."""
        self.assertTrue(True)
```

## Manual Testing with ROS 2

For integration tests with actual ROS 2 nodes running:

```bash
# Launch the system
./grizzly.py run

# In another terminal, test state changes
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 1, reason: 'Integration test'}"

# Monitor state
ros2 topic echo /system/state
```

## Performance

- **21 tests** run in **~0.55 seconds**
- Minimal overhead for environment setup
- Parallel execution support (coming soon)

## Support

For issues or questions about the test suite, check the test output for detailed error messages and review the test files in `grizzly_stack/test/`.
