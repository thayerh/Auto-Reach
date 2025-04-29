# Auto-Reach

## Auto-Reach: Verifying Autonomous Vehicle Planning under Sensor Attacks
Auto-Reach is a toolchain for evaluating the safety of autonomous vehicle plans (from Autoware) under sensor manipulation attacks. It integrates:

- Simulation using AWSIM
- Planning using Autoware
- Offline verification via CommonRoad-Reach

The system enables researchers and developers to simulate, attack, and verify planned trajectories with fine-grained control and automated analysis.

## Repository Structure

verif-autoware-commonroad/
├── scenarios/                         # CommonRoad XML maps
├── output/                            # Logs of verification results
├── rosbags/                           # ROS2 bags for experiments (no attack / LiDAR / pose spoofing)
├── convert_lanelet2_to_commonroad.py # Converts Lanelet2 OSM to CommonRoad XML
├── shift_commonroad_coordinates.py   # Applies fixed coordinate translation
├── update_initial_pose.py            # Replaces start pose in CommonRoad XML
├── insert_static_obstacles.py        # Adds static obstacles (e.g., pedestrians, vehicles)
├── extract_trajectories.py           # Parses planning trajectories from rosbag.db3
├── verify_trajectories.py            # Main trajectory verification script
├── publish_fake_sensor_data.py       # Publishes fake LiDAR or GNSS data to ROS2
├── compare_osm_commonroad_offsets.py # Computes offsets between OSM and CommonRoad coordinates
├── visualize_scenario.py             # Visualizes scenario over time steps
├── run.sh                            # Runs all verifications and saves logs
└── README.md


## Quick Setup
### Dependencies
- Python 3.8+
- ROS 2 Humble
- CommonRoad-Reach
- CRDesigner

Install base dependencies:

pip install shapely lxml matplotlib numpy

## How to Use Each Script
### convert_lanelet2_to_commonroad.py
Convert a Lanelet2 OSM map into a CommonRoad XML scenario.

python3 convert_lanelet2_to_commonroad.py
Input: lanelet2_map.osm
Output: scenarios/ZAM_AWSIM-1_1_T-1.xml

### shift_commonroad_coordinates.py
Apply a fixed translation offset to all (x, y, z) points in a CommonRoad file.

python3 shift_commonroad_coordinates.py
Input: scenarios/ZAM_AWSIM-1_1_T-1.xml
Output: scenarios/ZAM_AWSIM-1_1_T-1_shifted.xml

### update_initial_pose.py
Set the vehicle’s initial pose (position + orientation) in the scenario.

python3 update_initial_pose.py
Automatically overwrites the initial pose in *_shifted.xml.

### insert_static_obstacles.py
Add static pedestrians or vehicles to the scenario.

python3 insert_static_obstacles.py
Adds 2 pedestrians and 1 parked vehicle at hardcoded locations.

### extract_trajectories.py
Extract Autoware-generated trajectories from a rosbag.

python3 extract_trajectories.py
Parses /planning/scenario_planning/trajectory from rosbag2_*.db3.

### verify_trajectories.py
Offline verify whether each trajectory stays inside the reachable set.

python3 verify_trajectories.py <rosbag.db3> <scenario_label>

python3 verify_trajectories.py rosbags/no_interference/rosbag2_*.db3 no_interference
Results are saved to output/no_interference.txt if piped via run.sh.

### publish_fake_sensor_data.py
Replay malicious sensor messages over ROS2 topics.

python3 publish_fake_sensor_data.py LIDAR
#### or
python3 publish_fake_sensor_data.py POSE
Publishes a single fake message at a fixed frequency.

### compare_osm_commonroad_offsets.py
Compute coordinate offsets between a Lanelet2 OSM file and its CommonRoad translation.

python3 compare_osm_commonroad_offsets.py
Prints max/min/average (x, y) offsets for visual/manual debugging.

### visualize_scenario.py
Render a CommonRoad scenario over time using matplotlib.

python3 visualize_scenario.py
Visualizes up to 40 time steps (0.1s each).

### run.sh
Runs all three verification experiments and logs the results.

bash run.sh
Executes:

no_interference

lidar_inject_1

pose

Outputs: output/no_interference.txt, etc.

### Example Output (from run.sh)

Trajectory 42: 3 points failed
Total failed trajectories: 154
Total failed points: 634
Use this to compare baseline vs attack conditions.

### Goals and Limitations
Auto-Reach is designed to:

Validate Autoware trajectory safety

Simulate adversarial conditions (sensor spoofing)

Enable repeatable, offline, analysis using reachability tools

### This project is:

Simulation-only (no real vehicle interface)

Not real-time

Limited to static obstacles and 2D projections

### Attribution
Developed by Thayer Hicks
University of North Carolina at Chapel Hill
GitHub: @thayerh
