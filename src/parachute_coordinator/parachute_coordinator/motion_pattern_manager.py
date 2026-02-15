#!/usr/bin/env python3
"""
Motion Pattern Manager

Manages reusable motion patterns for the main arm.
Patterns are stored as relative waypoints (offsets from target position)
and can be applied to any target endpoint.

Pattern JSON format:
{
    "name": "stow_arc",
    "description": "Arc motion for stowing lines",
    "reference_point": "end",  # "start", "end", or "center"
    "waypoints": [
        {"dx": 0.0, "dy": 0.0, "dz": 0.10, "gripper": "open"},
        {"dx": 0.0, "dy": -0.05, "dz": 0.05},
        {"dx": 0.0, "dy": 0.0, "dz": 0.0, "gripper": "close"}
    ],
    "speed_factor": 0.5
}
"""

import os
import json
import glob
import math
from typing import List, Dict, Optional, Tuple
from geometry_msgs.msg import Pose, Point, Quaternion
from dataclasses import dataclass

try:
    import tf_transformations
    HAS_TF_TRANSFORMS = True
except ImportError:
    HAS_TF_TRANSFORMS = False


@dataclass
class PatternWaypoint:
    """A single waypoint in a pattern (relative offsets)."""
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    gripper: Optional[str] = None  # "open", "close", or None
    pause: float = 0.0  # Pause time at this waypoint (seconds)


@dataclass
class MotionPattern:
    """A complete motion pattern with metadata."""
    name: str
    description: str
    waypoints: List[PatternWaypoint]
    reference_point: str = "end"  # Which waypoint is the "target"
    speed_factor: float = 0.5
    source_file: Optional[str] = None


class MotionPatternManager:
    """
    Manages loading, storing, and applying motion patterns.

    Patterns are stored as relative offsets, making them reusable
    for any target position.
    """

    def __init__(self, pattern_dir: Optional[str] = None, logger=None):
        """
        Initialize the pattern manager.

        Args:
            pattern_dir: Directory containing pattern JSON files.
                        Defaults to config/motion_patterns in package.
            logger: ROS2 logger for output (optional)
        """
        self.logger = logger
        self.patterns: Dict[str, MotionPattern] = {}

        # Default pattern directory
        if pattern_dir is None:
            # Try to find package config directory
            try:
                from ament_index_python.packages import get_package_share_directory
                pattern_dir = os.path.join(
                    get_package_share_directory('parachute_coordinator'),
                    'config', 'motion_patterns'
                )
            except:
                # Fallback to source directory
                pattern_dir = os.path.join(
                    os.path.dirname(__file__), '..', 'config', 'motion_patterns'
                )

        self.pattern_dir = pattern_dir
        self._log(f"Pattern directory: {self.pattern_dir}")

        # Load patterns from directory
        self.load_patterns_from_directory()

    def _log(self, msg: str, level: str = "info"):
        """Log a message using ROS2 logger or print."""
        if self.logger:
            getattr(self.logger, level)(msg)
        else:
            print(f"[{level.upper()}] {msg}")

    def load_patterns_from_directory(self):
        """Load all pattern JSON files from the pattern directory."""
        if not os.path.exists(self.pattern_dir):
            self._log(f"Pattern directory does not exist: {self.pattern_dir}", "warn")
            os.makedirs(self.pattern_dir, exist_ok=True)
            self._log(f"Created pattern directory: {self.pattern_dir}")
            return

        pattern_files = glob.glob(os.path.join(self.pattern_dir, "*.json"))

        for filepath in pattern_files:
            try:
                pattern = self.load_pattern_from_file(filepath)
                if pattern:
                    self.patterns[pattern.name] = pattern
                    self._log(f"Loaded pattern: {pattern.name} ({len(pattern.waypoints)} waypoints)")
            except Exception as e:
                self._log(f"Failed to load {filepath}: {e}", "error")

        self._log(f"Loaded {len(self.patterns)} motion patterns")

    def load_pattern_from_file(self, filepath: str) -> Optional[MotionPattern]:
        """Load a single pattern from a JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)

        # Handle both new format (relative) and old format (absolute waypoints)
        if 'waypoints' not in data:
            self._log(f"No waypoints in {filepath}", "warn")
            return None

        waypoints = []
        for wp_data in data['waypoints']:
            # New format: relative offsets
            if 'dx' in wp_data or 'dy' in wp_data or 'dz' in wp_data:
                wp = PatternWaypoint(
                    dx=wp_data.get('dx', 0.0),
                    dy=wp_data.get('dy', 0.0),
                    dz=wp_data.get('dz', 0.0),
                    roll=wp_data.get('roll'),
                    pitch=wp_data.get('pitch'),
                    yaw=wp_data.get('yaw'),
                    gripper=wp_data.get('gripper'),
                    pause=wp_data.get('pause', 0.0)
                )
            # Old format: absolute positions (ee_pose)
            elif 'ee_pose' in wp_data:
                # These will need to be converted to relative
                # For now, store as absolute (conversion happens separately)
                ee = wp_data['ee_pose']
                wp = PatternWaypoint(
                    dx=ee.get('x', 0.0),
                    dy=ee.get('y', 0.0),
                    dz=ee.get('z', 0.0),
                    gripper=wp_data.get('gripper'),
                    pause=wp_data.get('pause', 0.0)
                )
            else:
                continue

            waypoints.append(wp)

        if not waypoints:
            return None

        return MotionPattern(
            name=data.get('name', os.path.splitext(os.path.basename(filepath))[0]),
            description=data.get('description', ''),
            waypoints=waypoints,
            reference_point=data.get('reference_point', 'end'),
            speed_factor=data.get('speed_factor', 0.5),
            source_file=filepath
        )

    def get_pattern(self, name: str) -> Optional[MotionPattern]:
        """Get a pattern by name."""
        return self.patterns.get(name)

    def list_patterns(self) -> List[str]:
        """List all available pattern names."""
        return list(self.patterns.keys())

    def apply_pattern(self, pattern_name: str, target: Point) -> List[Pose]:
        """
        Apply a pattern to a target position.

        Args:
            pattern_name: Name of the pattern to apply
            target: Target endpoint position

        Returns:
            List of Pose messages representing absolute waypoints
        """
        pattern = self.get_pattern(pattern_name)
        if pattern is None:
            self._log(f"Pattern not found: {pattern_name}", "error")
            return []

        return self.generate_waypoints(pattern, target)

    def generate_waypoints(self, pattern: MotionPattern, target: Point) -> List[Pose]:
        """
        Generate absolute waypoints from a pattern and target position.

        The reference_point determines which waypoint aligns with the target:
        - "end": Last waypoint is at target (approach motion)
        - "start": First waypoint is at target (departure motion)
        - "center": Middle waypoint is at target
        """
        waypoints = []

        # Determine the reference offset based on reference_point
        ref_idx = 0
        if pattern.reference_point == "end":
            ref_idx = len(pattern.waypoints) - 1
        elif pattern.reference_point == "center":
            ref_idx = len(pattern.waypoints) // 2
        # "start" uses ref_idx = 0

        # Get the reference waypoint's offset
        ref_wp = pattern.waypoints[ref_idx]

        # Calculate base position (target minus reference offset)
        base_x = target.x - ref_wp.dx
        base_y = target.y - ref_wp.dy
        base_z = target.z - ref_wp.dz

        # Generate absolute waypoints
        for wp in pattern.waypoints:
            pose = Pose()
            pose.position.x = base_x + wp.dx
            pose.position.y = base_y + wp.dy
            pose.position.z = base_z + wp.dz

            # Check if waypoint has custom orientation
            if wp.roll is not None or wp.pitch is not None or wp.yaw is not None:
                # Use specified orientation (default to 0 for unspecified axes)
                roll = wp.roll if wp.roll is not None else 0.0
                pitch = wp.pitch if wp.pitch is not None else -math.pi / 2  # Default to pointing down
                yaw = wp.yaw if wp.yaw is not None else 0.0

                if HAS_TF_TRANSFORMS:
                    quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]
                else:
                    # Fallback: simple pitch-only quaternion calculation
                    pose.orientation.x = 0.0
                    pose.orientation.y = math.sin(pitch / 2)
                    pose.orientation.z = 0.0
                    pose.orientation.w = math.cos(pitch / 2)
            else:
                # Default orientation: gripper pointing straight down
                # For WX200: pitch = -π/2 (-90°) makes gripper point down
                # Quaternion for -90° pitch rotation: (0, -0.707, 0, 0.707)
                pose.orientation.x = 0.0
                pose.orientation.y = -0.7071068  # -sin(45°) for -90° pitch
                pose.orientation.z = 0.0
                pose.orientation.w = 0.7071068   # cos(45°) for -90° pitch

            waypoints.append(pose)

        return waypoints

    def get_gripper_commands(self, pattern_name: str) -> List[Optional[str]]:
        """
        Get gripper commands for each waypoint in a pattern.

        Returns list of "open", "close", or None for each waypoint.
        """
        pattern = self.get_pattern(pattern_name)
        if pattern is None:
            return []

        return [wp.gripper for wp in pattern.waypoints]

    def get_pause_times(self, pattern_name: str) -> List[float]:
        """Get pause times for each waypoint in a pattern."""
        pattern = self.get_pattern(pattern_name)
        if pattern is None:
            return []

        return [wp.pause for wp in pattern.waypoints]

    def save_pattern(self, pattern: MotionPattern, filename: Optional[str] = None):
        """Save a pattern to a JSON file."""
        if filename is None:
            filename = os.path.join(self.pattern_dir, f"{pattern.name}.json")

        data = {
            "name": pattern.name,
            "description": pattern.description,
            "reference_point": pattern.reference_point,
            "speed_factor": pattern.speed_factor,
            "waypoints": [
                {
                    "dx": wp.dx,
                    "dy": wp.dy,
                    "dz": wp.dz,
                    **({"roll": wp.roll} if wp.roll is not None else {}),
                    **({"pitch": wp.pitch} if wp.pitch is not None else {}),
                    **({"yaw": wp.yaw} if wp.yaw is not None else {}),
                    **({"gripper": wp.gripper} if wp.gripper is not None else {}),
                    **({"pause": wp.pause} if wp.pause > 0 else {}),
                }
                for wp in pattern.waypoints
            ]
        }

        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        self._log(f"Saved pattern to {filename}")

    def create_pattern_from_waypoints(
        self,
        name: str,
        absolute_waypoints: List[Dict],
        reference_point: str = "end",
        description: str = ""
    ) -> MotionPattern:
        """
        Convert absolute waypoints to a relative pattern.

        Args:
            name: Pattern name
            absolute_waypoints: List of dicts with 'x', 'y', 'z' (or 'ee_pose')
            reference_point: "start", "end", or "center"
            description: Pattern description

        Returns:
            MotionPattern with relative offsets
        """
        # Extract positions
        positions = []
        for wp in absolute_waypoints:
            if 'ee_pose' in wp:
                pos = wp['ee_pose']
            else:
                pos = wp
            positions.append((pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)))

        if not positions:
            raise ValueError("No waypoints provided")

        # Determine reference position
        ref_idx = 0
        if reference_point == "end":
            ref_idx = len(positions) - 1
        elif reference_point == "center":
            ref_idx = len(positions) // 2

        ref_pos = positions[ref_idx]

        # Convert to relative offsets
        relative_waypoints = []
        for i, (x, y, z) in enumerate(positions):
            wp = PatternWaypoint(
                dx=x - ref_pos[0],
                dy=y - ref_pos[1],
                dz=z - ref_pos[2],
                gripper=absolute_waypoints[i].get('gripper'),
                pause=absolute_waypoints[i].get('pause', 0.0)
            )
            relative_waypoints.append(wp)

        return MotionPattern(
            name=name,
            description=description,
            waypoints=relative_waypoints,
            reference_point=reference_point,
            speed_factor=0.5
        )


# Built-in patterns (available without loading files)
BUILTIN_PATTERNS = {
    "approach_vertical": MotionPattern(
        name="approach_vertical",
        description="Approach target from directly above",
        reference_point="end",
        waypoints=[
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.15),  # 15cm above
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.08),  # 8cm above
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.0),   # Target
        ],
        speed_factor=0.3
    ),
    # "stow_arc": MotionPattern(
    #     name="stow_arc",
    #     description="Arc motion for stowing - approach, sweep, place",
    #     reference_point="end",
    #     waypoints=[
    #         PatternWaypoint(dx=-0.10, dy=0.0, dz=0.10, gripper="open"),  # Approach from side/above
    #         PatternWaypoint(dx=-0.05, dy=0.0, dz=0.05),  # Arc midpoint
    #         PatternWaypoint(dx=0.0, dy=0.0, dz=0.0, gripper="close"),  # Target
    #     ],
    #     speed_factor=0.4
    # ),
    "stow_arc": MotionPattern(
        name="recorded_stow",
        description="Recorded sweeping stow motion with wider approach",
        reference_point="start",
        waypoints=[
            # pitch=-π/2 = pointing down, yaw controls rotation around vertical axis
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.0, pitch=-math.pi/2),        # Start - extended, pointing down
            PatternWaypoint(dx=-0.124, dy=0.0, dz=-0.247, pitch=-math.pi/2),  # Down to surface
            PatternWaypoint(dx=-0.146, dy=0.0, dz=-0.229, pitch=-math.pi/2),  # Slightly back
            PatternWaypoint(dx=-0.247, dy=0.084, dz=-0.230, pitch=-math.pi/2, yaw=0.3),  # Sweep to right, angled
            PatternWaypoint(dx=-0.244, dy=-0.072, dz=-0.190, pitch=-math.pi/2, yaw=-0.3),  # Sweep to left, angled
            PatternWaypoint(dx=-0.244, dy=-0.12, dz=-0.190, pitch=-math.pi/2, yaw=-0.4),  # Go wider left
            PatternWaypoint(dx=-0.15, dy=-0.08, dz=-0.170, pitch=-math.pi/2, yaw=-0.2),   # Further in
            PatternWaypoint(dx=-0.098, dy=0.001, dz=-0.171, pitch=-math.pi/2),  # Final position, straight down
        ],
        speed_factor=0.4
    ),
    "pickup_line": MotionPattern(
        name="pickup_line",
        description="Pick up a line - approach, grasp, lift",
        reference_point="start",  # First waypoint is pickup point
        waypoints=[
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.0, gripper="open"),  # At line
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.0, gripper="close", pause=0.5),  # Grasp
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.10),  # Lift up
            PatternWaypoint(dx=-0.05, dy=0.0, dz=0.15),  # Clear and back
        ],
        speed_factor=0.3
    ),
    "retract_safe": MotionPattern(
        name="retract_safe",
        description="Safe retraction - up then back",
        reference_point="start",
        waypoints=[
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.0),  # Current position
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.10),  # Lift
            PatternWaypoint(dx=-0.10, dy=0.0, dz=0.10),  # Back and up
        ],
        speed_factor=0.5
    ),
    "recorded_stow": MotionPattern(
        name="recorded_stow",
        description="Recorded sweeping stow motion with wider approach",
        reference_point="start",
        waypoints=[
            # pitch=-π/2 = pointing down, yaw controls rotation around vertical axis
            PatternWaypoint(dx=0.0, dy=0.0, dz=0.0, pitch=-math.pi/2),        # Start - extended, pointing down
            PatternWaypoint(dx=-0.124, dy=0.0, dz=-0.247, pitch=-math.pi/2),  # Down to surface
            PatternWaypoint(dx=-0.146, dy=0.0, dz=-0.229, pitch=-math.pi/2),  # Slightly back
            PatternWaypoint(dx=-0.247, dy=0.084, dz=-0.230, pitch=-math.pi/2, yaw=0.3),  # Sweep to right, angled
            PatternWaypoint(dx=-0.244, dy=-0.072, dz=-0.190, pitch=-math.pi/2, yaw=-0.3),  # Sweep to left, angled
            PatternWaypoint(dx=-0.244, dy=-0.12, dz=-0.190, pitch=-math.pi/2, yaw=-0.4),  # Go wider left
            PatternWaypoint(dx=-0.15, dy=-0.08, dz=-0.170, pitch=-math.pi/2, yaw=-0.2),   # Further in
            PatternWaypoint(dx=-0.098, dy=0.001, dz=-0.171, pitch=-math.pi/2),  # Final position, straight down
        ],
        speed_factor=0.4
    ),
}
