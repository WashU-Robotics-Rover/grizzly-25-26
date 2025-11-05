"""
Core Subsystem for Grizzly Stack

This module contains core system management nodes that monitor and coordinate
the overall rover system health and status.

Nodes:
- system_manager: Monitors system health and publishes periodic status updates
- lifecycle_manager: Manages startup and lifecycle transitions of all nodes
- layer_manager: Manages lifecycle of nodes organized by layers

The core subsystem runs continuously and provides essential telemetry for
debugging and monitoring the rover's operational state.
"""

from .layer_manager import LayerManager

__all__ = ['LayerManager']
