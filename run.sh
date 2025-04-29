#!/bin/bash

# ============================================================================
# run.sh
#
# Runs trajectory verification on three different rosbag recordings:
#   - No interference
#   - LiDAR data injection attack
#   - Pose/GNSS spoofing attack
#
# Each run logs output to a separate file in the 'output/' directory.
# ============================================================================

# Start timing
echo "Starting verification runs at:"
date +%T

# ----------------------------------------------------------------------------
# Process 1: Baseline run (no interference)
# ----------------------------------------------------------------------------
echo "Running baseline (no interference)..."
python3 main.py rosbags/no_interference/rosbag2_2025_04_27-12_22_45_0.db3 no_interference > output/no_interference.txt
echo "Baseline finished at:"
date +%T

# ----------------------------------------------------------------------------
# Process 2: LiDAR data injection attack
# ----------------------------------------------------------------------------
echo "Running LiDAR injection attack..."
python3 main.py rosbags/lidar_inject_1/rosbag2_2025_04_27-13_50_32_0.db3 lidar_inject_1 > output/lidar_inject_1.txt
echo "LiDAR injection finished at:"
date +%T

# ----------------------------------------------------------------------------
# Process 3: Pose/GNSS spoofing attack
# ----------------------------------------------------------------------------
echo "Running Pose spoofing attack..."
python3 main.py rosbags/pose/rosbag2_2025_04_28-10_44_21_0.db3 pose > output/pose.txt
echo "Pose spoofing finished at:"
date +%T

# ----------------------------------------------------------------------------
# All done
# ----------------------------------------------------------------------------
echo "All verification runs completed."
