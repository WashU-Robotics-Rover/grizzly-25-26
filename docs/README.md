# Grizzly Rover Docs

ROS 2 robotics stack for the WashU Rover Team.

## Quick Links

- **[Development Guide](DEVELOPMENT.md)** - Start here! Everything you need to work on the project.

## Quick Start

```bash
./grizzly.py build   # Build
./grizzly.py run     # Run
./grizzly.py test    # Test
```

## Adding a New Node

1. Create file inheriting from `GrizzlyLifecycleNode`
2. Add to `setup.py` entry_points
3. Add to `config/layers.yaml`
4. Build and run

See [Development Guide](DEVELOPMENT.md#adding-a-new-node) for details.

---

**Go Bears! 🐻**
