"""
shift_commonroad_coordinates.py

Applies a constant translation offset to all (x, y, z) points in a CommonRoad XML scenario.

Usage:
    python3 shift_commonroad_coordinates.py

Dependencies:
    - xml.etree.ElementTree
"""

import xml.etree.ElementTree as ET

# Configuration
INPUT_FILE = "scenarios/ZAM_AWSIM-1_1_T-1.xml"
OUTPUT_FILE = "scenarios/ZAM_AWSIM-1_1_T-1_shifted.xml"

OFFSET_X = -300000.0013
OFFSET_Y = -3900000.15895
OFFSET_Z = 0


def shift_commonroad_coordinates(input_path: str, output_path: str, offset_x: float, offset_y: float, offset_z: float = 0.0):
    """
    Applies a fixed translation to all (x, y, z) coordinates in the given CommonRoad scenario.

    Args:
        input_path (str): Path to the input CommonRoad XML file.
        output_path (str): Path where the shifted XML will be saved.
        offset_x (float): Shift in X-axis.
        offset_y (float): Shift in Y-axis.
        offset_z (float): Shift in Z-axis (optional).

    Returns:
        None
    """
    tree = ET.parse(input_path)
    root = tree.getroot()

    # Remove problematic traffic light elements if present
    for elem in root.findall(".//trafficLight"):
        root.remove(elem)

    xs_shifted = []
    ys_shifted = []

    # Apply shift to each <point>
    for point in root.findall(".//point"):
        x_elem = point.find("x")
        y_elem = point.find("y")
        z_elem = point.find("z")

        if x_elem is not None and y_elem is not None:
            new_x = float(x_elem.text) + offset_x
            new_y = float(y_elem.text) + offset_y
            x_elem.text = str(new_x)
            y_elem.text = str(new_y)
            xs_shifted.append(new_x)
            ys_shifted.append(new_y)

        if z_elem is not None:
            z_elem.text = str(float(z_elem.text) + offset_z)

    # Save the shifted XML
    tree.write(output_path, encoding="UTF-8", xml_declaration=True)
    print(f"✅ Shifted XML written to: {output_path}")

    # Optional: Print some statistics
    print(f"Max shifted X: {max(xs_shifted):.4f}, Max shifted Y: {max(ys_shifted):.4f}")
    print(f"Min shifted X: {min(xs_shifted):.4f}, Min shifted Y: {min(ys_shifted):.4f}")


if __name__ == "__main__":
    shift_commonroad_coordinates(INPUT_FILE, OUTPUT_FILE, OFFSET_X, OFFSET_Y, OFFSET_Z)
