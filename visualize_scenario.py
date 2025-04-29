"""
visualize_scenario.py

Visualizes a CommonRoad scenario over multiple time steps using matplotlib.

Usage:
    python3 visualize_scenario.py

Dependencies:
    - commonroad
    - matplotlib
"""

import matplotlib.pyplot as plt
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# Configuration
FILE_PATH = "scenarios/ZAM_AWSIM-1_1_T-1_shifted.xml"
TIME_STEPS = 40  # Number of time steps to visualize

def main():
    plt.rcParams['figure.max_open_warning'] = 50

    # Read the scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(FILE_PATH).open()

    # Visualize the scenario over multiple time steps
    for time_step in range(TIME_STEPS):
        plt.figure(figsize=(25, 10))
        rnd = MPRenderer()
        rnd.draw_params.time_begin = time_step

        scenario.draw(rnd)
        planning_problem_set.draw(rnd)
        rnd.render()
        plt.show()


if __name__ == "__main__":
    main()
