"""
verify_trajectories.py

This script verifies trajectory safety for an autonomous vehicle 
by checking if each point of a simulated trajectory stays within 
the computed reachable sets.

Usage:
    python3 verify_trajectories.py <path_to_rosbag.db3> <scenario_name>

Dependencies:
    - commonroad_reach
    - shapely
    - custom modules: testRosbag.py, updateCurrentLocation.py
"""

import sys
from shapely.geometry import Point
from commonroad_reach.data_structure.configuration_builder import ConfigurationBuilder
from commonroad_reach.data_structure.reach.reach_interface import ReachableSetInterface
from commonroad_reach.utility import visualization as util_visual
from commonroad_reach.data_structure.reach.reach_node import ReachNode

from extract_trajectories import get_points
from updateCurrentLocation import update_current_location

# Name of the CommonRoad scenario used for reachability validation
NAME_SCENARIO = "ZAM_AWSIM-1_1_T-1_shifted"

def is_point_within_reach_node(point: Point, node: ReachNode) -> bool:
    """
    Checks if a given point lies within the bounds of a ReachNode.

    Args:
        point (Point): A shapely Point representing a trajectory point.
        node (ReachNode): The reachability node (bounding box).

    Returns:
        bool: True if the point lies inside the reachability node, False otherwise.
    """
    return (node.p_x_min <= point.x <= node.p_x_max) and (node.p_y_min <= point.y <= node.p_y_max)


def verify_rosbag_trajectories(bag_path: str, scenario_name: str):
    """
    Verifies trajectory points from a ROS bag against reachable sets.

    Args:
        bag_path (str): Path to the input rosbag.db3 file.
        scenario_name (str): Name used for output directories and visualization files.
    """
    failed_trajectories = 0
    total_failed_points = 0

    for i, trajectory in enumerate(get_points(bag_path)):
        initial_pose, path = trajectory

        update_current_location(initial_pose)

        config = ConfigurationBuilder().build_configuration(NAME_SCENARIO)
        config.update()
        if i == 0:
            config.print_configuration_summary()

        # Compute reachable sets
        reach_interface = ReachableSetInterface(config)
        reach_interface.compute_reachable_sets()
        nodes = reach_interface.reachable_set

        # Plot reachable sets with scenario
        output_path = f"output/{scenario_name}/{scenario_name}_{i}"
        util_visual.plot_scenario_with_reachable_sets(reach_interface=reach_interface, path_output=output_path, save_gif=False)

        # Check if trajectory points stay within the reachable sets
        trajectory_failed = False
        points_failed = 0
        for point in path:
            inside = any(
                is_point_within_reach_node(point, rn)
                for key in nodes for rn in nodes[key]
            )
            if not inside:
                print(f"Point {point} lies outside of reachable area")
                points_failed += 1
                trajectory_failed = True

        total_failed_points += points_failed
        if trajectory_failed:
            failed_trajectories += 1

        print(f"Trajectory {i}: {points_failed} points failed")
    
    print(f"Total failed trajectories: {failed_trajectories}")
    print(f"Total failed points: {total_failed_points}")

    return failed_trajectories, total_failed_points


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 verify_trajectories.py <path_to_rosbag.db3> <scenario_name>")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    scenario_name = sys.argv[2]

    verify_rosbag_trajectories(bag_path, scenario_name)
