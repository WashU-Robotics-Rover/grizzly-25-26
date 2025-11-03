# System Manager State Machine Tests

This test suite verifies the operational state machine functionality of the SystemManager node.

## Test Coverage

### State Machine Tests (17 tests)

1. **Initial State**: Verifies node starts in `STARTUP` state
2. **Valid Transitions**: Tests all allowed state transitions
   - STARTUP → STANDBY
   - STANDBY → AUTONOMOUS/MANUAL
   - AUTONOMOUS ↔ MANUAL (bidirectional)
3. **Invalid Transitions**: Ensures invalid transitions are rejected
   - STARTUP → AUTONOMOUS (must go through STANDBY)
   - SHUTDOWN → any state (terminal state)
4. **Emergency Handling**: 
   - Emergency can be triggered from most states
   - Only exit from EMERGENCY is to STANDBY
5. **Same State Transitions**: No-op transitions are allowed
6. **Service Functionality**: Tests external state change requests
7. **State History**: Verifies all transitions are logged with timestamps
8. **State Publishing**: Confirms state is published to `/system/state`
9. **Complete Workflow**: Tests full operational flow through all states

### Lifecycle Tests (4 tests)

Tests the ROS2 lifecycle callbacks:
- `on_configure()`
- `on_activate()`
- `on_deactivate()`
- `on_shutdown()`

## Running the Tests

### Run All Tests
```bash
cd /Users/danielhuinda/robotics/grizzly-25-26
source install/setup.zsh
python3 -m pytest grizzly_stack/test/test_system_manager.py -v
```

### Run Specific Test Class
```bash
# Only state machine tests
pytest grizzly_stack/test/test_system_manager.py::TestSystemManagerStateMachine -v

# Only lifecycle tests
pytest grizzly_stack/test/test_system_manager.py::TestSystemManagerLifecycle -v
```

### Run Single Test
```bash
pytest grizzly_stack/test/test_system_manager.py::TestSystemManagerStateMachine::test_complete_state_flow -v
```

### Run with Coverage
```bash
pytest grizzly_stack/test/test_system_manager.py --cov=grizzly_stack.core.system_manager --cov-report=html
```

## Test Results

All 21 tests pass successfully:
- ✅ 17 state machine tests
- ✅ 4 lifecycle tests
- ⚡ Runs in ~0.3 seconds

## State Transition Matrix

Valid transitions tested:

| From State   | To States                                    |
|-------------|----------------------------------------------|
| STARTUP     | STANDBY, ERROR, EMERGENCY                    |
| STANDBY     | AUTONOMOUS, MANUAL, SHUTDOWN, EMERGENCY      |
| AUTONOMOUS  | STANDBY, MANUAL, EMERGENCY, ERROR            |
| MANUAL      | STANDBY, AUTONOMOUS, EMERGENCY, ERROR        |
| EMERGENCY   | STANDBY (only)                               |
| ERROR       | STANDBY, SHUTDOWN                            |
| SHUTDOWN    | (none - terminal state)                      |

## Adding New Tests

To add new test cases:

1. Add a method to `TestSystemManagerStateMachine` class
2. Name it with `test_` prefix
3. Use standard unittest assertions:
   ```python
   def test_my_new_feature(self):
       """Test description."""
       # Setup
       self.node._current_state = OperationalState.STANDBY
       
       # Execute
       result = self.node._some_method()
       
       # Assert
       self.assertTrue(result)
       self.assertEqual(self.node._current_state, OperationalState.AUTONOMOUS)
   ```

## Integration Testing

For integration tests with actual ROS2 nodes running:

```bash
# Launch the system
ros2 launch grizzly_stack grizzly_minimal.launch.py

# In another terminal, test state changes
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 1, reason: 'Integration test'}"

# Monitor state
ros2 topic echo /system/state
```

## CI/CD Integration

These tests are designed to run in CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Run State Machine Tests
  run: |
    source install/setup.bash
    pytest grizzly_stack/test/test_system_manager.py --junitxml=test-results.xml
```
