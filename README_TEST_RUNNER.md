# Grizzly Automated Test Suite

A comprehensive test runner for the Grizzly ROS 2 stack that makes running tests simple and efficient.

## Quick Start

```bash
# Run all tests
./run_tests.py

# Run with coverage report
./run_tests.py --coverage

# Run specific test file
./run_tests.py --specific test_system_manager.py

# List all available tests
./run_tests.py --list
```

## Features

✅ **Automatic Environment Detection**: Checks for ROS 2 installation and pytest  
✅ **Multiple Output Formats**: Standard, verbose, coverage, and HTML reports  
✅ **Flexible Test Selection**: Run all tests or specific test files/cases  
✅ **Color-Coded Output**: Easy-to-read test results  
✅ **Fast Execution**: Runs all tests in under 1 second  

## Installation

No additional installation needed! The script uses the existing ROS 2 environment and pytest.

If pytest is not installed:
```bash
pip3 install pytest pytest-cov
```

## Usage

### Basic Commands

```bash
# Run all tests (verbose by default)
./run_tests.py

# Run with standard output (less verbose)
./run_tests.py

# Show print statements and logging
./run_tests.py --show-output
./run_tests.py -s

# Stop at first failure
./run_tests.py --fail-fast
./run_tests.py -x
```

### Coverage Reports

```bash
# Terminal coverage report
./run_tests.py --coverage

# Generate HTML coverage report
./run_tests.py --html
# Then open: htmlcov/index.html
```

### Specific Tests

```bash
# Run specific test file
./run_tests.py --specific test_system_manager.py

# Run specific test class
./run_tests.py --specific "test_system_manager.py::TestSystemManagerStateMachine"

# Run specific test method
./run_tests.py --specific "test_system_manager.py::TestSystemManagerStateMachine::test_initial_state"
```

### List Tests

```bash
# List all available tests
./run_tests.py --list
```

## Command-Line Options

| Option | Short | Description |
|--------|-------|-------------|
| `--verbose` | `-v` | Enable verbose test output |
| `--coverage` | | Generate coverage report |
| `--html` | | Generate HTML coverage report |
| `--specific TEST` | | Run specific test file or test case |
| `--list` | | List all available tests |
| `--show-output` | `-s` | Show print statements and logging |
| `--fail-fast` | `-x` | Stop at first test failure |
| `--help` | `-h` | Show help message |

## Test Structure

Current test suite includes:

### `test_system_manager.py` (21 tests)
- **State Machine Tests (17 tests)**: Tests all state transitions, invalid transitions, emergency handling, service functionality, and state history
- **Lifecycle Tests (4 tests)**: Tests ROS 2 lifecycle callbacks

### `test_basic.py`
- Basic functionality tests (currently empty, ready for expansion)

## Examples

### Example 1: Quick Test Run
```bash
./run_tests.py
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
./run_tests.py --html
```
Opens a detailed HTML coverage report showing which lines of code are tested.

### Example 3: Debug Specific Test
```bash
./run_tests.py --specific "test_system_manager.py::TestSystemManagerStateMachine::test_emergency_override_from_any_state" -s
```
Runs only the emergency override test with full output visible.

## Integration with CI/CD

The test runner can be easily integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Run Grizzly Tests
  run: |
    source install/setup.zsh
    ./run_tests.py --coverage
```

## Troubleshooting

### Tests Won't Run
1. Make sure the workspace is built:
   ```bash
   ./build.sh
   ```

2. Verify pytest is installed:
   ```bash
   pip3 install pytest pytest-cov
   ```

3. Check ROS 2 environment:
   ```bash
   source install/setup.zsh
   ```

### Import Errors
If you see import errors, ensure the workspace is sourced:
```bash
source install/setup.zsh
./run_tests.py
```

## Adding New Tests

1. Create a new test file in `grizzly_stack/test/` with the prefix `test_`
2. Write your test cases using Python's `unittest` or `pytest`
3. Run `./run_tests.py --list` to verify your tests are discovered
4. Run `./run_tests.py` to execute all tests including your new ones

Example test structure:
```python
import unittest

class TestMyFeature(unittest.TestCase):
    def test_something(self):
        """Test description."""
        self.assertTrue(True)
```

## Performance

- **21 tests** run in **~0.55 seconds**
- Minimal overhead for environment setup
- Parallel execution support (coming soon)

## Support

For issues or questions about the test suite:
1. Check the test output for detailed error messages
2. Review the test files in `grizzly_stack/test/`
3. See `grizzly_stack/test/README_TESTS.md` for test-specific documentation

## License

Part of the Grizzly ROS 2 stack.
