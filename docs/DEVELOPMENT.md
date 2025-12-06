# Grizzly Development Guide

## Quick Start

```bash
git clone https://github.com/WashU-Robotics-Rover/grizzly-25-26.git
cd grizzly-25-26
./grizzly.py build
./grizzly.py run
```

---

## Adding a New Node (3 Steps)

### Step 1: Create Your Node

Create a file like `src/grizzly_stack/perception/my_sensor.py`:

```python
from std_msgs.msg import Float32
from grizzly_stack.core import GrizzlyLifecycleNode, run_node


class MySensor(GrizzlyLifecycleNode):
    def __init__(self):
        super().__init__('my_sensor')
        self._pub = None
    
    def on_configure_hook(self) -> bool:
        self._pub = self.create_publisher(Float32, '/my_sensor/data', 10)
        return True
    
    def on_cleanup_hook(self) -> bool:
        if self._pub:
            self.destroy_publisher(self._pub)
        return True
    
    def tick(self):
        msg = Float32()
        msg.data = 42.0
        self._pub.publish(msg)


def main():
    run_node(MySensor)
```

### Step 2: Register in setup.py

Add one line to `entry_points`:

```python
'my_sensor = grizzly_stack.perception.my_sensor:main',
```

### Step 3: Add to layers.yaml

Add to `config/layers.yaml`:

```yaml
nodes:
  my_sensor:
    enabled: true
    layer: perception
    params:
      rate_hz: 10.0
```

### Build and Run

```bash
./grizzly.py build
./grizzly.py run
```

**Done!** The launch file auto-discovers all nodes from `layers.yaml`.

---

## Project Structure

```
grizzly_stack/
├── config/
│   ├── core.yaml       # System manager settings
│   └── layers.yaml     # ALL node config (single source of truth)
├── src/grizzly_stack/
│   ├── core/           # System infrastructure (don't modify)
│   ├── perception/     # Sensor nodes
│   ├── planner/        # Planning nodes
│   ├── control/        # Motor control nodes
│   └── examples/       # Templates to copy
└── setup.py            # Node registration
```

---

## layers.yaml Reference

```yaml
nodes:
  node_name:
    enabled: true       # Set to false to disable
    layer: perception   # perception, planning, or control
    params:             # Passed to node as ROS parameters
      rate_hz: 10.0
      my_param: value

layer_order:            # Startup order (lower = first)
  perception: 1
  planning: 2
  control: 3
```

---

## GrizzlyLifecycleNode Reference

Override these methods:

| Method | Purpose |
|--------|---------|
| `on_configure_hook()` | Create publishers/subscribers |
| `on_cleanup_hook()` | Destroy publishers/subscribers |
| `tick()` | Main loop (called at `rate_hz`) |
| `on_activate_hook()` | Extra setup when activated |
| `on_deactivate_hook()` | Cleanup when deactivated |

Built-in features:
- `self.tick_count` - Loop counter
- `self.publish_status("msg")` - Publish to `/<node>/status`
- `rate_hz` parameter - Loop frequency
- `enabled` parameter - Can disable node

---

## Commands

```bash
./grizzly.py build          # Build
./grizzly.py build --clean  # Clean rebuild
./grizzly.py run            # Start system
./grizzly.py test           # Run tests
```

---

## Troubleshooting

**"Package not found"**
```bash
source install/setup.bash  # Linux
source install/setup.zsh   # macOS
```

**Node not starting**
- Check it's `enabled: true` in layers.yaml
- Check it's registered in setup.py
- Run `./grizzly.py build --clean`

---

**Go Bears! 🐻**
