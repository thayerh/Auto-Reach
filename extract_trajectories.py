"""
extract_trajectories.py

Extracts and filters vehicle trajectories from a ROS2 bag (.db3) file
for trajectory verification purposes.

Usage:
    python3 extract_trajectories.py

Dependencies:
    - rclpy
    - sqlite3
    - shapely
    - autoware_planning_msgs
"""

import sqlite3
import rclpy
from rclpy.serialization import deserialize_message
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import Quaternion
from shapely.geometry import Point
import math
import sys

# Default path to rosbag
DEFAULT_DB3_PATH = "rosbags/no_interference/rosbag2_2025_04_27-12_22_45_0.db3"

# ROS Topic for planned trajectories
POSE_TOPIC_NAME = "/planning/scenario_planning/trajectory"

# How many steps vehicle must remain stopped to trigger truncation
STOPPED_STEPS_THRESHOLD = 30


def quaternion_to_yaw(q: Quaternion) -> float:
    """
    Converts a quaternion orientation into a yaw angle (radians).

    Args:
        q (Quaternion): ROS2 Quaternion message.

    Returns:
        float: Yaw angle in radians.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


def extract_yaw_from_pose(data: Trajectory) -> float:
    """
    Extracts the yaw (heading) from a Trajectory pose.

    Args:
        data (Trajectory): Deserialized Trajectory message.

    Returns:
        float: Yaw angle in radians.
    """
    q: Quaternion = data.pose.orientation
    return quaternion_to_yaw(q)


def get_points(bag_path: str = DEFAULT_DB3_PATH):
    """
    Extracts and filters trajectory points from a ROS2 bag file.

    Args:
        bag_path (str): Path to rosbag2 database file.

    Returns:
        list: List of tuples (initial_pose, [Point, Point, ...]) for each trajectory.
    """
    # Connect to the SQLite database
    conn = sqlite3.connect(bag_path)
    cursor = conn.cursor()

    cursor.execute(
        f"""SELECT m.data
            FROM messages m
            JOIN topics t ON m.topic_id = t.id
            WHERE t.name = '{POSE_TOPIC_NAME}'
            ORDER BY m.id;"""
    )
    topic_rows = cursor.fetchall()
    if not topic_rows:
        print(f"No messages found on topic {POSE_TOPIC_NAME}")
        sys.exit(1)

    print(f"Found {len(topic_rows)} trajectory messages.")

    rclpy.init()

    # Trim leading messages if the vehicle is initially stationary
    for i, row in enumerate(topic_rows):
        data = deserialize_message(row[0], Trajectory)
        initial_pose = Point(data.points[0].pose.position.x, data.points[0].pose.position.y)

        if i + 1 < len(topic_rows):
            next_data = deserialize_message(topic_rows[i + 1][0], Trajectory)
            next_initial_pose = Point(next_data.points[0].pose.position.x, next_data.points[0].pose.position.y)
            if initial_pose.x != next_initial_pose.x or initial_pose.y != next_initial_pose.y:
                topic_rows = topic_rows[i:]
                break

    # Trim trailing messages if the vehicle stops moving
    cur_stopped = 0
    for i, row in enumerate(topic_rows):
        data = deserialize_message(row[0], Trajectory)
        initial_pose = Point(data.points[0].pose.position.x, data.points[0].pose.position.y)

        if i + 1 < len(topic_rows):
            next_data = deserialize_message(topic_rows[i + 1][0], Trajectory)
            next_initial_pose = Point(next_data.points[0].pose.position.x, next_data.points[0].pose.position.y)
            if initial_pose.x == next_initial_pose.x and initial_pose.y == next_initial_pose.y:
                cur_stopped += 1
            else:
                cur_stopped = 0

            if cur_stopped >= STOPPED_STEPS_THRESHOLD:
                topic_rows = topic_rows[:i]
                break

    print(f"Filtered recording includes {len(topic_rows)} trajectories.")

    points_list = []
    for row in topic_rows:
        data = deserialize_message(row[0], Trajectory)
        initial_pose = (
            Point(data.points[0].pose.position.x, data.points[0].pose.position.y),
            extract_yaw_from_pose(data.points[0])
        )
        trajectory_points = [Point(pt.pose.position.x, pt.pose.position.y) for pt in data.points]
        points_list.append((initial_pose, trajectory_points))

    print(f"Returning {len(points_list)} trajectories.")

    conn.close()

    return points_list


if __name__ == "__main__":
    get_points()
