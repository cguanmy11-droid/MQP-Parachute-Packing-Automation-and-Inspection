#!/usr/bin/env python3
"""
State Machine Engine

Lightweight, reusable state machine that loads transitions from a YAML config.
Designed for ROS 2 node integration — the node subclasses or composes this
and implements state handler methods.

Usage:
    sm = StateMachine('path/to/transitions.yaml', logger=self.get_logger())
    sm.transition('start')  # IDLE -> AT_LOOP
    sm.transition('positioned')  # AT_LOOP -> INSERT
"""

from enum import Enum, auto
from typing import Callable, Dict, Optional, Any
import yaml


class StowState(Enum):
    """States matching the stow transitions config."""
    IDLE = auto()
    AT_LOOP = auto()
    INSERT = auto()
    HANDOFF = auto()
    RETRACT = auto()
    RELEASE = auto()
    COMPLETE = auto()
    ERROR = auto()


class StateMachine:
    """
    Event-driven state machine loaded from YAML config.

    Calls on_enter_<STATE>, on_exit_<STATE> methods on the owner
    when transitions occur. The owner registers these by passing
    itself as the handler_object.
    """

    def __init__(self, config_path: str, handler_object: Any = None, logger=None):
        self._logger = logger
        self._handler = handler_object
        self._transitions: Dict[StowState, Dict[str, StowState]] = {}
        self._state_configs: Dict[StowState, dict] = {}
        self._error_source: Optional[StowState] = None
        self._retry_count: int = 0

        self._load_config(config_path)
        self.current_state: StowState = self._initial_state

        # History for _PREVIOUS transitions (error recovery)
        self._previous_state: Optional[StowState] = None

        self._log(f'State machine initialized in {self.current_state.name}')

    def _load_config(self, config_path: str):
        """Load state transitions from YAML file."""
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Parse initial state
        initial = config.get('initial_state', 'IDLE')
        self._initial_state = StowState[initial]

        # Parse each state and its transitions
        for state_name, state_config in config.get('states', {}).items():
            try:
                state = StowState[state_name]
            except KeyError:
                self._log(f'Warning: Unknown state in config: {state_name}', warn=True)
                continue

            self._state_configs[state] = state_config
            self._transitions[state] = {}

            for event, target_name in state_config.get('transitions', {}).items():
                if target_name == '_PREVIOUS':
                    # Special marker — resolved at transition time
                    self._transitions[state][event] = None
                else:
                    try:
                        self._transitions[state][event] = StowState[target_name]
                    except KeyError:
                        self._log(
                            f'Warning: Unknown target state: {target_name} '
                            f'(from {state_name}.{event})', warn=True
                        )

        self._log(f'Loaded {len(self._transitions)} states from config')

    @property
    def state(self) -> StowState:
        """Current state."""
        return self.current_state

    @property
    def state_name(self) -> str:
        """Current state name as string."""
        return self.current_state.name

    @property
    def error_source(self) -> Optional[StowState]:
        """The state that caused the current error, if in ERROR state."""
        return self._error_source

    @property
    def retry_count(self) -> int:
        """Number of retries attempted in current state."""
        return self._retry_count

    def get_state_config(self, state: Optional[StowState] = None) -> dict:
        """Get the YAML config for a state (current state if none specified)."""
        state = state or self.current_state
        return self._state_configs.get(state, {})

    def get_valid_events(self) -> list:
        """List valid events for the current state."""
        return list(self._transitions.get(self.current_state, {}).keys())

    def can_transition(self, event: str) -> bool:
        """Check if an event is valid for the current state."""
        return event in self._transitions.get(self.current_state, {})

    def transition(self, event: str) -> bool:
        """
        Attempt a state transition based on an event.

        Returns True if the transition occurred, False if the event
        was invalid for the current state.
        """
        state_transitions = self._transitions.get(self.current_state, {})

        if event not in state_transitions:
            self._log(
                f'Invalid event "{event}" in state {self.current_state.name}. '
                f'Valid events: {list(state_transitions.keys())}',
                warn=True
            )
            return False

        target = state_transitions[event]

        # Handle _PREVIOUS (error recovery)
        if target is None:
            if self._error_source is not None:
                target = self._error_source
                self._log(f'Recovering to previous state: {target.name}')
            else:
                self._log('No previous state to recover to', warn=True)
                return False

        old_state = self.current_state

        # Track error source
        if target == StowState.ERROR:
            self._error_source = old_state

        # Track retry count
        if target == old_state:
            self._retry_count += 1
        else:
            self._retry_count = 0

        # Check max retries if configured
        max_retries = self._state_configs.get(old_state, {}).get('max_retries')
        if max_retries and self._retry_count > max_retries:
            self._log(
                f'Max retries ({max_retries}) exceeded in {old_state.name}',
                warn=True
            )
            # Force transition to ERROR instead
            self._error_source = old_state
            target = StowState.ERROR
            self._retry_count = 0

        # Exit old state
        self._call_handler(f'on_exit_{old_state.name.lower()}', old_state, event)

        # Update state
        self._previous_state = old_state
        self.current_state = target

        self._log(
            f'Transition: {old_state.name} --[{event}]--> {target.name}'
        )

        # Enter new state
        self._call_handler(f'on_enter_{target.name.lower()}', target, event)

        # Clear error source when leaving ERROR
        if old_state == StowState.ERROR and target != StowState.ERROR:
            self._error_source = None

        return True

    def reset(self):
        """Reset state machine to initial state."""
        self.current_state = self._initial_state
        self._error_source = None
        self._previous_state = None
        self._retry_count = 0
        self._log('State machine reset')

    def _call_handler(self, method_name: str, state: StowState, event: str):
        """Call a handler method on the owner object if it exists."""
        if self._handler and hasattr(self._handler, method_name):
            try:
                getattr(self._handler, method_name)(state, event)
            except Exception as e:
                self._log(f'Handler {method_name} raised: {e}', warn=True)

    def _log(self, msg: str, warn: bool = False):
        """Log a message if logger is available."""
        if self._logger:
            if warn:
                self._logger.warn(f'[SM] {msg}')
            else:
                self._logger.info(f'[SM] {msg}')
