# 🐻 Grizzly Rover 2025-26

ROS 2 software stack for the WashU Robotics Rover Team.

## Quick Start

```bash
git clone https://github.com/WashU-Robotics-Rover/grizzly-25-26.git
cd grizzly-25-26
./grizzly.py build
./grizzly.py run
```

## Adding a Node (3 Steps)

**Step 1:** Create your node inheriting from `GrizzlyLifecycleNode`

**Step 2:** Add to `setup.py`:
```python
'my_node = grizzly_stack.mymodule.my_node:main',
```

**Step 3:** Add to `config/layers.yaml`:
```yaml
my_node:
  enabled: true
  layer: perception
  params:
    rate_hz: 10.0
```

**Build:** `./grizzly.py build`

The launch file auto-discovers nodes from `layers.yaml`. No other files to edit!

## Structure

```
grizzly_stack/
├── config/layers.yaml   # Single config for ALL nodes
├── src/grizzly_stack/
│   ├── core/            # System management
│   ├── perception/      # Sensor nodes
│   ├── planner/         # Planning nodes
│   ├── control/         # Motor control
│   └── examples/        # Templates
└── setup.py             # Node registration
```

## Commands

| Command | Description |
|---------|-------------|
| `./grizzly.py build` | Build |
| `./grizzly.py run` | Run system |
| `./grizzly.py test` | Run tests |

## Docs

📖 **[Development Guide](docs/DEVELOPMENT.md)**

---

**Go Bears! 🐻** — WashU Robotics Rover Team
