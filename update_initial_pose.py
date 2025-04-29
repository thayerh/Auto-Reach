"""
update_initial_pose.py

Updates the initial pose (position and orientation) in a CommonRoad scenario XML file.

Usage:
    python3 update_initial_pose.py

Dependencies:
    - lxml
    - shapely
"""

from lxml import etree
from shapely.geometry import Point

DEFAULT_CR_MAP_FILE = 'scenarios/ZAM_AWSIM-1_1_T-1_shifted.xml'

def update_initial_pose(pose: tuple[Point, float], cr_map_path: str = DEFAULT_CR_MAP_FILE):
    """
    Updates the initial state's position and orientation in a CommonRoad XML scenario.

    Args:
        pose (tuple[Point, float]): Tuple containing the position (Point) and orientation (radians).
        cr_map_path (str): Path to the CommonRoad XML map file.

    Returns:
        None
    """
    point, orientation = pose

    tree = etree.parse(cr_map_path)
    root = tree.getroot()

    # Find the planningProblem element
    planning_problem = root.find(".//planningProblem")
    if planning_problem is None:
        raise ValueError("No planningProblem found in CommonRoad XML!")

    for initial_state in planning_problem.findall(".//initialState"):
        # Update position (x, y) elements
        position_tag = initial_state.find(".//position/point")
        if position_tag is not None:
            x_elem = position_tag.find("x")
            y_elem = position_tag.find("y")
            if x_elem is not None and y_elem is not None:
                x_elem.text = str(point.x)
                y_elem.text = str(point.y)

        # Update orientation element
        orientation_tag = initial_state.find(".//orientation")
        if orientation_tag is not None:
            exact_elem = orientation_tag.find("exact")
            if exact_elem is not None:
                exact_elem.text = str(orientation)

    # Save updated XML back to file
    tree.write(cr_map_path, pretty_print=True, xml_declaration=True, encoding="UTF-8")


if __name__ == "__main__":
    test_pose = (Point(5.0, 10.0), 0.0)
    update_initial_pose(test_pose)
