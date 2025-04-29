"""
insert_static_obstacles.py

Adds static obstacles (pedestrians and a parked vehicle) to an existing
CommonRoad scenario and saves the updated scenario.

Usage:
    python3 insert_static_obstacles.py

Dependencies:
    - commonroad
    - matplotlib
    - numpy
"""

import numpy as np
import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.state import InitialState
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.scenario import Tag

# Configuration
FILE_PATH = "scenarios/ZAM_AWSIM-1_1_T-1_shifted.xml"

AUTHOR = "Thayer Hicks"
AFFILIATION = "UNC Chapel Hill"
SOURCE = ""
TAGS = {Tag.CRITICAL}


def create_pedestrian(position: np.ndarray) -> StaticObstacle:
    """
    Creates a pedestrian static obstacle.

    Args:
        position (np.ndarray): [x, y] coordinates of the pedestrian.

    Returns:
        StaticObstacle: Pedestrian object.
    """
    static_obstacle_id = scenario.generate_object_id()
    shape = Circle(radius=0.25)
    initial_state = InitialState(position=position, orientation=0, time_step=0, velocity=0, acceleration=0, yaw_rate=0, slip_angle=0)
    return StaticObstacle(static_obstacle_id, ObstacleType.PEDESTRIAN, shape, initial_state)


def create_parked_vehicle(position: np.ndarray, orientation: float) -> StaticObstacle:
    """
    Creates a parked vehicle static obstacle.

    Args:
        position (np.ndarray): [x, y] coordinates of the vehicle.
        orientation (float): Orientation in radians.

    Returns:
        StaticObstacle: Parked vehicle object.
    """
    static_obstacle_id = scenario.generate_object_id()
    shape = Rectangle(length=5.0, width=2.0)
    initial_state = InitialState(position=position, orientation=orientation, time_step=0, velocity=0, acceleration=0, yaw_rate=0, slip_angle=0)
    return StaticObstacle(static_obstacle_id, ObstacleType.PARKED_VEHICLE, shape, initial_state)


def main():
    global scenario
    scenario, planning_problem_set = CommonRoadFileReader(FILE_PATH).open()

    # Insert pedestrians
    pedestrian_positions = [
        np.array([81398.137375, 49928.77234375]),
        np.array([81399.942375, 49955.98364375])
    ]

    for pos in pedestrian_positions:
        pedestrian = create_pedestrian(pos)
        scenario.add_objects(pedestrian)

    # Insert parked vehicle
    parked_vehicle = create_parked_vehicle(np.array([81399.179375, 49968.41234375]), orientation=1.70785703)
    scenario.add_objects(parked_vehicle)

    # Visualize the modified scenario
    plt.rcParams['figure.max_open_warning'] = 50
    for i in range(1):
        plt.figure(figsize=(25, 10))
        rnd = MPRenderer()
        rnd.draw_params.time_begin = i
        scenario.draw(rnd)
        planning_problem_set.draw(rnd)
        rnd.render()
        plt.show()

    # Write the modified scenario back to file
    writer = CommonRoadFileWriter(
        scenario=scenario,
        planning_problem_set=planning_problem_set,
        author=AUTHOR,
        affiliation=AFFILIATION,
        source=SOURCE,
        tags=TAGS,
    )
    writer.write_to_file(FILE_PATH, OverwriteExistingFile.ALWAYS)
    print(f"✅ Scenario with static obstacles saved to {FILE_PATH}")


if __name__ == "__main__":
    main()
