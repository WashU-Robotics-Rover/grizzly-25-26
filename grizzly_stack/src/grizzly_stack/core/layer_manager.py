"""
Layer Manager for Grizzly Robotics Stack

This module manages the lifecycle of nodes organized into logical layers.
Instead of managing individual nodes, the system manager delegates lifecycle
control to the layer manager, which manages entire layers based on operational state.

This architecture prevents the system manager from becoming bloated as more nodes
are added to the system.
"""

from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState as LifecycleChangeState
from lifecycle_msgs.msg import Transition
from grizzly_interfaces.msg import OperationalState

from .utils import (
    get_state_name,
    load_layers_config,
    DEFAULT_STATE_LAYER_MAPPING,
)


class LayerManager:
    """
    Manages lifecycle transitions for nodes organized into layers.
    
    The layer manager receives operational state transitions from the system manager
    and controls which layers should be active based on the current operational state.
    Each layer contains one or more nodes that are managed together.
    """
    
    def __init__(self, node: Node):
        """
        Initialize the Layer Manager.
        
        Args:
            node: ROS 2 node instance (typically SystemManager) for creating clients
        """
        self._node = node
        self._logger = node.get_logger()
        
        # Load layer configuration from YAML file
        self._layers, _ = load_layers_config(self._logger)
        
        # Define which layers should be active for each operational state
        self._state_layer_mapping = DEFAULT_STATE_LAYER_MAPPING.copy()
        
        # Track lifecycle clients for each node
        self._lifecycle_clients = {}
        # Track the current lifecycle state of each node
        self._node_states = {}
        # Track all managed nodes
        self._managed_nodes = []
    
    def initialize_node_states(self, node_states: dict):
        """
        Initialize the cached node states from the lifecycle manager.
        
        This should be called after the lifecycle manager has configured nodes
        to their initial inactive state.
        
        Args:
            node_states: Dictionary mapping node names to their current state
        """
        self._node_states.update(node_states)
        self._logger.debug(f'Initialized node states: {node_states}')
    
    def handle_state_transition(self, old_state: int, new_state: int):
        """
        Handle operational state transition by managing layers accordingly.
        
        This is the main entry point called by the system manager when
        operational state changes. The layer manager determines which layers
        should be active/inactive and manages their nodes accordingly.
        
        Args:
            old_state: Previous operational state
            new_state: New operational state
        """
        old_state_name = get_state_name(old_state)
        new_state_name = get_state_name(new_state)
        
        self._logger.info(
            f'Layer Manager: Handling state transition {old_state_name} -> {new_state_name}'
        )
        
        # Special handling for emergency and shutdown states
        if new_state == OperationalState.EMERGENCY:
            self._logger.error('EMERGENCY: Deactivating all operational layers immediately')
            self._deactivate_all_layers()
            return
        
        if new_state == OperationalState.SHUTDOWN:
            self._logger.info('SHUTDOWN: Deactivating all layers')
            self._deactivate_all_layers()
            return
        
        # Normal layer management based on state mappings
        required_layers = self._state_layer_mapping.get(new_state, [])
        previous_layers = self._state_layer_mapping.get(old_state, [])
        
        # Determine which layers need to be activated/deactivated
        layers_to_activate = set(required_layers) - set(previous_layers)
        layers_to_deactivate = set(previous_layers) - set(required_layers)
        
        # Activate required layers
        for layer_name in layers_to_activate:
            self._logger.info(f'Activating layer: {layer_name}')
            self._activate_layer(layer_name)
        
        # Deactivate layers no longer needed
        for layer_name in layers_to_deactivate:
            self._logger.info(f'Deactivating layer: {layer_name}')
            self._deactivate_layer(layer_name)
    
    def _activate_layer(self, layer_name: str):
        """Activate all nodes in a layer."""
        if layer_name not in self._layers:
            self._logger.warn(f'Unknown layer: {layer_name}')
            return
        
        for node_name in self._layers[layer_name]:
            self._activate_node(node_name)
    
    def _deactivate_layer(self, layer_name: str):
        """Deactivate all nodes in a layer."""
        if layer_name not in self._layers:
            self._logger.warn(f'Unknown layer: {layer_name}')
            return
        
        for node_name in self._layers[layer_name]:
            self._deactivate_node(node_name)
    
    def _deactivate_all_layers(self):
        """Deactivate all layers (for emergency/shutdown)."""
        for layer_name in self._layers.keys():
            self._deactivate_layer(layer_name)
    
    def _get_lifecycle_client(self, node_name: str):
        """Get or create a lifecycle change state client for a managed node."""
        if node_name not in self._lifecycle_clients:
            service_name = f'/{node_name}/change_state'
            self._lifecycle_clients[node_name] = self._node.create_client(
                LifecycleChangeState,
                service_name
            )
            self._logger.debug(f'Created lifecycle client for {node_name}')
        
        return self._lifecycle_clients[node_name]
    
    def _activate_node(self, node_name: str, timeout_sec: float = 2.0):
        """
        Activate a lifecycle node (Inactive -> Active).
        
        Uses async calls to avoid deadlock issues.
        """
        cached_state = self._node_states.get(node_name, 'inactive')
        
        if cached_state == 'active':
            self._logger.info(f'Skipping activation of {node_name} - already active')
            return
        
        if cached_state == 'unconfigured':
            self._logger.info(f'{node_name} is unconfigured - configuring first')
            self._configure_node(node_name, timeout_sec)
            return
        
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._logger.warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        
        self._logger.info(f'Requesting activation of {node_name}...')
        
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_activate_response(f, node_name)
        )
    
    def _handle_activate_response(self, future, node_name: str):
        """Handle the response from lifecycle activation request."""
        try:
            response = future.result()
            if response and response.success:
                self._logger.info(f'✅ Successfully activated {node_name}')
                self._node_states[node_name] = 'active'
                if node_name not in self._managed_nodes:
                    self._managed_nodes.append(node_name)
            else:
                self._logger.error(f'❌ Failed to activate {node_name}')
        except Exception as e:
            self._logger.error(f'Exception while activating {node_name}: {e}')
    
    def _configure_node(self, node_name: str, timeout_sec: float = 2.0):
        """Configure a lifecycle node (Unconfigured -> Inactive)."""
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._logger.warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_CONFIGURE
        
        self._logger.info(f'Requesting configuration of {node_name}...')
        
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_configure_response(f, node_name)
        )
    
    def _handle_configure_response(self, future, node_name: str):
        """Handle the response from lifecycle configuration request."""
        try:
            response = future.result()
            if response and response.success:
                self._logger.info(f'✅ Successfully configured {node_name}')
                self._node_states[node_name] = 'inactive'
                # Now activate the node
                self._logger.info(f'Now activating {node_name}...')
                self._activate_node(node_name)
            else:
                self._logger.error(f'❌ Failed to configure {node_name}')
        except Exception as e:
            self._logger.error(f'Exception while configuring {node_name}: {e}')
    
    def _deactivate_node(self, node_name: str, timeout_sec: float = 2.0):
        """Deactivate a lifecycle node (Active -> Inactive)."""
        cached_state = self._node_states.get(node_name, 'inactive')
        
        if cached_state != 'active':
            self._logger.info(
                f'Skipping deactivation of {node_name} - state is {cached_state}'
            )
            return
        
        client = self._get_lifecycle_client(node_name)
        
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._logger.warn(
                f'Lifecycle service for {node_name} not available after {timeout_sec}s'
            )
            return
        
        request = LifecycleChangeState.Request()
        request.transition.id = Transition.TRANSITION_DEACTIVATE
        
        self._logger.info(f'Requesting deactivation of {node_name}...')
        
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_deactivate_response(f, node_name)
        )
    
    def _handle_deactivate_response(self, future, node_name: str):
        """Handle the response from lifecycle deactivation request."""
        try:
            response = future.result()
            if response and response.success:
                self._logger.info(f'✅ Successfully deactivated {node_name}')
                self._node_states[node_name] = 'inactive'
            else:
                self._logger.error(f'❌ Failed to deactivate {node_name}')
        except Exception as e:
            self._logger.error(f'Exception while deactivating {node_name}: {e}')
    
    # =========================================================================
    # Public API
    # =========================================================================
    
    def get_managed_nodes(self) -> list:
        """Get list of all managed nodes."""
        return self._managed_nodes.copy()
    
    def get_layer_nodes(self, layer_name: str) -> list:
        """Get list of nodes in a specific layer."""
        return self._layers.get(layer_name, []).copy()
    
    def get_all_layers(self) -> list:
        """Get list of all layer names."""
        return list(self._layers.keys())
