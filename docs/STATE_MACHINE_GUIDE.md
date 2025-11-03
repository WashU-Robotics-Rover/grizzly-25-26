# System Manager State Machine - Quick Reference

## Overview

The System Manager implements a global operational state machine for coordinating the rover's operational modes.

## States

| State | Value | Description |
|-------|-------|-------------|
| STARTUP | 0 | System initializing |
| STANDBY | 1 | Ready but not active |
| AUTONOMOUS | 2 | Autonomous operation mode |
| MANUAL | 3 | Manual/teleoperation mode |
| EMERGENCY | 4 | Emergency stop engaged |
| ERROR | 5 | System error state |
| SHUTDOWN | 6 | System shutting down |

## Topics

### Published Topics

- `/system/state` (grizzly_interfaces/msg/OperationalState)
  - Current operational state
  - Published at 1 Hz (configurable)
  - Contains: state value, timestamp, human-readable description

- `/system/health` (std_msgs/msg/String)
  - Health status messages
  - Published at 1 Hz

## Services

### `/system/change_state` (grizzly_interfaces/srv/ChangeState)

Change the operational state externally.

**Request:**
- `requested_state` (uint8): Target state
- `reason` (string): Optional reason for the change

**Response:**
- `success` (bool): True if transition was allowed
- `current_state` (uint8): Current state after request
- `message` (string): Human-readable result

## Usage Examples

### Command Line

```bash
# Monitor current state
ros2 topic echo /system/state

# Change to STANDBY
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 1, reason: 'Ready for operations'}"

# Change to AUTONOMOUS
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 2, reason: 'Starting autonomous mission'}"

# Trigger EMERGENCY
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 4, reason: 'EMERGENCY STOP'}"

# Return to STANDBY from EMERGENCY
ros2 service call /system/change_state grizzly_interfaces/srv/ChangeState \
  "{requested_state: 1, reason: 'Emergency cleared'}"
```

### Python Code

```python
import rclpy
from rclpy.node import Node
from grizzly_interfaces.msg import OperationalState
from grizzly_interfaces.srv import ChangeState

class StateControllerNode(Node):
    def __init__(self):
        super().__init__('state_controller')
        
        # Create service client
        self.client = self.create_client(
            ChangeState, 
            '/system/change_state'
        )
        
        # Subscribe to state updates
        self.subscription = self.create_subscription(
            OperationalState,
            '/system/state',
            self.state_callback,
            10
        )
    
    def state_callback(self, msg):
        """Called when state updates are published."""
        self.get_logger().info(
            f'Current state: {msg.description} (value: {msg.state})'
        )
    
    def change_state(self, target_state, reason=""):
        """Request a state change."""
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('State change service not available')
            return None
        
        request = ChangeState.Request()
        request.requested_state = target_state
        request.reason = reason
        
        future = self.client.call_async(request)
        return future

# Example usage
def main():
    rclpy.init()
    node = StateControllerNode()
    
    # Request transition to AUTONOMOUS
    future = node.change_state(
        OperationalState.AUTONOMOUS,
        "Starting autonomous navigation"
    )
    
    # Wait for response
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        response = future.result()
        if response.success:
            print(f"Success: {response.message}")
        else:
            print(f"Failed: {response.message}")
    
    rclpy.shutdown()
```

### C++ Code

```cpp
#include "rclcpp/rclcpp.hpp"
#include "grizzly_interfaces/msg/operational_state.hpp"
#include "grizzly_interfaces/srv/change_state.hpp"

class StateController : public rclcpp::Node {
public:
    StateController() : Node("state_controller") {
        // Create service client
        client_ = this->create_client<grizzly_interfaces::srv::ChangeState>(
            "/system/change_state"
        );
        
        // Subscribe to state updates
        subscription_ = this->create_subscription<
            grizzly_interfaces::msg::OperationalState
        >(
            "/system/state", 10,
            std::bind(&StateController::state_callback, this, std::placeholders::_1)
        );
    }
    
    void change_state(uint8_t target_state, const std::string& reason) {
        auto request = std::make_shared<grizzly_interfaces::srv::ChangeState::Request>();
        request->requested_state = target_state;
        request->reason = reason;
        
        auto result_future = client_->async_send_request(request);
        
        // Wait for result
        if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "State change: %s", response->message.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "State change failed: %s", response->message.c_str());
            }
        }
    }
    
private:
    void state_callback(const grizzly_interfaces::msg::OperationalState::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "State: %s", msg->description.c_str());
    }
    
    rclcpp::Client<grizzly_interfaces::srv::ChangeState>::SharedPtr client_;
    rclcpp::Subscription<grizzly_interfaces::msg::OperationalState>::SharedPtr subscription_;
};
```

## Valid State Transitions

```
STARTUP ──────────────► STANDBY ◄──────────────┐
                           │ ▲                  │
                           │ │                  │
                           ▼ │                  │
                      AUTONOMOUS ◄──────────► MANUAL
                           │                    │
                           │                    │
                           └──► EMERGENCY ◄─────┘
                                    │
                                    ▼
                                STANDBY ──────► SHUTDOWN
                                    ▲
                                    │
                                  ERROR
```

### Transition Rules

1. **STARTUP** can go to: STANDBY, ERROR, EMERGENCY
2. **STANDBY** can go to: AUTONOMOUS, MANUAL, SHUTDOWN, EMERGENCY
3. **AUTONOMOUS** can go to: STANDBY, MANUAL, EMERGENCY, ERROR
4. **MANUAL** can go to: STANDBY, AUTONOMOUS, EMERGENCY, ERROR
5. **EMERGENCY** can only go to: STANDBY
6. **ERROR** can go to: STANDBY, SHUTDOWN
7. **SHUTDOWN** is terminal (no transitions out)

## Best Practices

### 1. Always Check Service Response
```python
response = future.result()
if not response.success:
    self.get_logger().error(f'Transition failed: {response.message}')
```

### 2. Monitor State Before Critical Operations
```python
# Wait until in AUTONOMOUS mode
while self.current_state != OperationalState.AUTONOMOUS:
    time.sleep(0.1)
```

### 3. Handle Emergency States
```python
def state_callback(self, msg):
    if msg.state == OperationalState.EMERGENCY:
        # Stop all motion immediately
        self.halt_all_systems()
```

### 4. Log State Changes with Reasons
```python
self.change_state(
    OperationalState.ERROR,
    f"Sensor failure: {sensor_name}"
)
```

## Troubleshooting

### State Change Rejected
- **Cause**: Invalid transition
- **Solution**: Check transition rules above, may need intermediate state

### Service Not Available
- **Cause**: SystemManager not running or not configured
- **Solution**: Launch grizzly_minimal.launch.py, transition node to Active state

### State Not Publishing
- **Cause**: SystemManager not activated
- **Solution**: Use ros2 lifecycle commands to activate the node

## Lifecycle Management

The SystemManager is a lifecycle node. To use it:

```bash
# Configure the node
ros2 lifecycle set /system_manager configure

# Activate the node (starts state publishing)
ros2 lifecycle set /system_manager activate

# Deactivate (stops publishing but keeps resources)
ros2 lifecycle set /system_manager deactivate

# Shutdown
ros2 lifecycle set /system_manager shutdown
```
