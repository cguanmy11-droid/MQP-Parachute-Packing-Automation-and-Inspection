#!/usr/bin/env python3
"""
Waypoint to Pattern Converter

Converts absolute waypoint JSON files (recorded via Xbox controller)
to relative motion patterns that can be reused at any target position.

Usage:
    python3 convert_waypoints_to_pattern.py <input.json> <output.json> [options]

Options:
    --name NAME         Pattern name (default: derived from filename)
    --description DESC  Pattern description
    --reference REF     Reference point: start, end, center (default: end)
    --speed FACTOR      Speed factor 0.0-1.0 (default: 0.5)

Example:
    python3 convert_waypoints_to_pattern.py \
        ~/waypoints/waypoints_20260211.json \
        config/motion_patterns/my_stow_pattern.json \
        --name "my_stow" --reference end
"""

import argparse
import json
import os
import sys


def load_absolute_waypoints(filepath: str) -> list:
    """Load waypoints from Xbox controller recording format."""
    with open(filepath, 'r') as f:
        data = json.load(f)

    waypoints = data.get('waypoints', [])
    if not waypoints:
        raise ValueError(f"No waypoints found in {filepath}")

    return waypoints


def convert_to_relative(
    waypoints: list,
    reference_point: str = "end"
) -> tuple:
    """
    Convert absolute waypoints to relative offsets.

    Args:
        waypoints: List of waypoint dicts with 'ee_pose' containing x,y,z
        reference_point: "start", "end", or "center"

    Returns:
        (relative_waypoints, reference_position)
    """
    # Extract positions
    positions = []
    for wp in waypoints:
        if 'ee_pose' in wp and wp['ee_pose']:
            pos = wp['ee_pose']
            positions.append({
                'x': pos.get('x', 0.0),
                'y': pos.get('y', 0.0),
                'z': pos.get('z', 0.0),
                'original': wp
            })
        elif 'joint_positions' in wp:
            # Joint-only waypoint, skip for now or use placeholder
            print(f"  Warning: Waypoint has joints only, no EE pose - skipping")
            continue

    if not positions:
        raise ValueError("No valid positions found in waypoints")

    # Determine reference index
    if reference_point == "end":
        ref_idx = len(positions) - 1
    elif reference_point == "center":
        ref_idx = len(positions) // 2
    else:  # start
        ref_idx = 0

    ref_pos = positions[ref_idx]
    print(f"  Reference point ({reference_point}): "
          f"({ref_pos['x']:.4f}, {ref_pos['y']:.4f}, {ref_pos['z']:.4f})")

    # Convert to relative offsets
    relative = []
    for i, pos in enumerate(positions):
        rel = {
            'dx': round(pos['x'] - ref_pos['x'], 6),
            'dy': round(pos['y'] - ref_pos['y'], 6),
            'dz': round(pos['z'] - ref_pos['z'], 6),
        }

        # Copy over any gripper or pause info from original
        orig = pos.get('original', {})
        if 'gripper' in orig:
            rel['gripper'] = orig['gripper']
        if 'pause' in orig and orig['pause'] > 0:
            rel['pause'] = orig['pause']

        relative.append(rel)

    return relative, ref_pos


def create_pattern(
    name: str,
    relative_waypoints: list,
    reference_point: str = "end",
    speed_factor: float = 0.5,
    description: str = ""
) -> dict:
    """Create a pattern dictionary ready for JSON export."""
    return {
        "name": name,
        "description": description,
        "reference_point": reference_point,
        "speed_factor": speed_factor,
        "waypoints": relative_waypoints
    }


def save_pattern(pattern: dict, filepath: str):
    """Save pattern to JSON file."""
    os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
    with open(filepath, 'w') as f:
        json.dump(pattern, f, indent=2)
    print(f"  Saved to: {filepath}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert absolute waypoints to relative motion patterns"
    )
    parser.add_argument('input', help="Input waypoints JSON file")
    parser.add_argument('output', help="Output pattern JSON file")
    parser.add_argument('--name', help="Pattern name", default=None)
    parser.add_argument('--description', help="Pattern description", default="")
    parser.add_argument('--reference', choices=['start', 'end', 'center'],
                        default='end', help="Reference point for offsets")
    parser.add_argument('--speed', type=float, default=0.5,
                        help="Speed factor (0.0-1.0)")

    args = parser.parse_args()

    # Derive name from output filename if not specified
    if args.name is None:
        args.name = os.path.splitext(os.path.basename(args.output))[0]

    print(f"Converting waypoints to pattern...")
    print(f"  Input: {args.input}")
    print(f"  Output: {args.output}")
    print(f"  Name: {args.name}")
    print(f"  Reference: {args.reference}")

    try:
        # Load absolute waypoints
        waypoints = load_absolute_waypoints(args.input)
        print(f"  Loaded {len(waypoints)} waypoints")

        # Convert to relative
        relative, ref_pos = convert_to_relative(waypoints, args.reference)
        print(f"  Converted to {len(relative)} relative waypoints")

        # Create pattern
        pattern = create_pattern(
            name=args.name,
            relative_waypoints=relative,
            reference_point=args.reference,
            speed_factor=args.speed,
            description=args.description
        )

        # Save
        save_pattern(pattern, args.output)

        print("\nPattern created successfully!")
        print(f"\nTo use this pattern:")
        print(f"  1. Copy to: src/parachute_coordinator/config/motion_patterns/")
        print(f"  2. Rebuild: colcon build --packages-select parachute_coordinator")
        print(f"  3. Use: ros2 topic pub /packing_coordinator/command std_msgs/String \"data: 'pattern:{args.name}'\"")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
