"""
Shared Utilities for Grizzly Robotics Stack
"""

import os
import yaml
from typing import Dict, List, Tuple, Optional
from ament_index_python.packages import get_package_share_directory
from grizzly_interfaces.msg import OperationalState


# =============================================================================
# Constants
# =============================================================================

STATE_NAMES: Dict[int, str] = {
    OperationalState.STARTUP: 'STARTUP',
    OperationalState.STANDBY: 'STANDBY',
    OperationalState.AUTONOMOUS: 'AUTONOMOUS',
    OperationalState.MANUAL: 'MANUAL',
    OperationalState.EMERGENCY: 'EMERGENCY',
    OperationalState.ERROR: 'ERROR',
    OperationalState.SHUTDOWN: 'SHUTDOWN',
}

# Default state-to-layer mapping
DEFAULT_STATE_LAYER_MAPPING: Dict[int, List[str]] = {
    OperationalState.STARTUP: [],
    OperationalState.STANDBY: [],
    OperationalState.AUTONOMOUS: ['perception', 'planning', 'control'],
    OperationalState.MANUAL: ['perception', 'planning', 'control'],
    OperationalState.EMERGENCY: [],
    OperationalState.ERROR: [],
    OperationalState.SHUTDOWN: [],
}


# =============================================================================
# Helper Functions
# =============================================================================

def get_state_name(state: int) -> str:
    """Get human-readable name for a state constant."""
    return STATE_NAMES.get(state, f'UNKNOWN({state})')


def get_config_path(filename: str) -> str:
    """Get full path to a config file in grizzly_stack."""
    package_share = get_package_share_directory('grizzly_stack')
    return os.path.join(package_share, 'config', filename)


def load_yaml_config(filename: str) -> Optional[dict]:
    """Load a YAML config file."""
    try:
        with open(get_config_path(filename), 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def load_layers_config(logger=None) -> Tuple[Dict[str, List[str]], List[str]]:
    """
    Load layer configuration from layers.yaml.
    
    Returns:
        Tuple of (layers_dict, layer_order_list)
    """
    try:
        config = load_yaml_config('layers.yaml')
        
        if not config:
            if logger:
                logger.warn('No config found, using defaults')
            return _default_layers()
        
        nodes_config = config.get('nodes', {})
        layer_order_config = config.get('layer_order', {})
        
        # Group enabled nodes by layer
        layers: Dict[str, List[str]] = {}
        for node_name, node_config in nodes_config.items():
            if not node_config.get('enabled', False):
                continue
            
            layer = node_config.get('layer', 'unknown')
            if layer not in layers:
                layers[layer] = []
            layers[layer].append(node_name)
        
        # Sort layers by order
        layer_order = sorted(
            layers.keys(),
            key=lambda x: layer_order_config.get(x, 999)
        )
        
        if logger:
            logger.info(f'Loaded {len(layers)} layers with {sum(len(n) for n in layers.values())} nodes')
        
        return layers, layer_order
        
    except Exception as e:
        if logger:
            logger.error(f'Failed to load config: {e}')
        return _default_layers()


def _default_layers():
    """Return default empty layer config."""
    return {
        'perception': [],
        'planning': [],
        'control': [],
    }, ['perception', 'planning', 'control']


def get_valid_transitions() -> Dict[int, List[int]]:
    """Get the valid state transition mapping."""
    return {
        OperationalState.STARTUP: [
            OperationalState.STANDBY,
            OperationalState.ERROR,
            OperationalState.EMERGENCY
        ],
        OperationalState.STANDBY: [
            OperationalState.AUTONOMOUS,
            OperationalState.MANUAL,
            OperationalState.SHUTDOWN,
            OperationalState.EMERGENCY
        ],
        OperationalState.AUTONOMOUS: [
            OperationalState.STANDBY,
            OperationalState.MANUAL,
            OperationalState.EMERGENCY,
            OperationalState.ERROR
        ],
        OperationalState.MANUAL: [
            OperationalState.STANDBY,
            OperationalState.AUTONOMOUS,
            OperationalState.EMERGENCY,
            OperationalState.ERROR
        ],
        OperationalState.EMERGENCY: [
            OperationalState.STANDBY
        ],
        OperationalState.ERROR: [
            OperationalState.STANDBY,
            OperationalState.SHUTDOWN
        ],
        OperationalState.SHUTDOWN: []
    }
