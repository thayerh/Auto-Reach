"""
publish_fake_sensor_data.py

Publishes fake or manipulated sensor data (LiDAR or GNSS Pose) at a fixed frequency
to specified ROS 2 topics for adversarial simulation.

Usage:
    python3 publish_fake_sensor_data.py <LIDAR|POSE>

Dependencies:
    - rclpy
    - sqlite3
    - sensor_msgs
    - geometry_msgs
"""

import sys
import time
import sqlite3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped

# Constants
PUBLISH_FREQUENCY_HZ = 0.4  # Publish rate in Hz

LIDAR = "LIDAR"
POSE = "POSE"

LIDAR_BAG_PATH = "rosbags/fake_data/lidar/rosbag2_2025_04_27-13_35_56_0.db3"
POSE_BAG_PATH = "rosbags/fake_data/gnss/rosbag2_2025_04_28-10_15_32_0.db3"


def load_fake_data(sensor_type: str):
    """
    Loads a single fake sensor message from a rosbag database.

    Args:
        sensor_type (str): 'LIDAR' or 'POSE'

    Returns:
        ROS message: PointCloud2 or PoseWithCovarianceStamped
    """
    if sensor_type == LIDAR:
        bag_path = LIDAR_BAG_PATH
    elif sensor_type == POSE:
        bag_path = POSE_BAG_PATH
    else:
        raise ValueError(f"Unknown sensor type: {sensor_type}")

    conn = sqlite3.connect(bag_path)
    cursor = conn.cursor()

    cursor.execute("SELECT m.data FROM messages m;")
    topic_row = cursor.fetchone()
    conn.close()

    if topic_row is None:
        print("Error: No messages found in bag.")
        sys.exit(1)

    data = topic_row[0]

    if sensor_type == LIDAR:
        return deserialize_message(data, PointCloud2)
    elif sensor_type == POSE:
        return deserialize_message(data, PoseWithCovarianceStamped)


def main(sensor_type: str):
    """
    Publishes the loaded fake data to the appropriate ROS topic.
    """
    rclpy.init()

    if sensor_type == LIDAR:
        node = Node('fake_lidar_publisher')
        publisher = node.create_publisher(PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)
    elif sensor_type == POSE:
        node = Node('fake_pose_publisher')
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/localization/pose_estimator/pose_with_covariance', 10)
    else:
        print(f"Invalid sensor type: {sensor_type}")
        sys.exit(1)

    msg = load_fake_data(sensor_type)

    publish_period = 1.0 / PUBLISH_FREQUENCY_HZ

    try:
        while rclpy.ok():
            publisher.publish(msg)
            time.sleep(publish_period)
    except KeyboardInterrupt:
        print("Publishing interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) != 2 or sys.argv[1].upper() not in [LIDAR, POSE]:
        print("Usage: python3 publish_fake_sensor_data.py <LIDAR|POSE>")
        sys.exit(1)

    sensor_type = sys.argv[1].upper()
    main(sensor_type)
