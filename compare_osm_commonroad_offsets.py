"""
compare_osm_commonroad_offsets.py

Calculates the coordinate offsets between a Lanelet2 (.osm) map and a
CommonRoad (.xml) map to validate and align coordinate systems.

Usage:
    python3 compare_osm_commonroad_offsets.py

Dependencies:
    - lxml
"""

from lxml import etree

# Configuration Constants
OSM_MAP_PATH = "lanelet2_map.osm"
COMMONROAD_XML_PATH = "scenarios/ZAM_AWSIM-1_1_T-1.xml"

def get_extreme_coords_osm(osm_path: str, find_max: bool = True):
    """
    Extracts the extreme (max or min) coordinates from a Lanelet2 OSM map.

    Args:
        osm_path (str): Path to .osm file.
        find_max (bool): Whether to find maximum (True) or minimum (False) coordinates.

    Returns:
        (float, float): (x, y) coordinates.
    """
    tree = etree.parse(osm_path)
    root = tree.getroot()

    extreme_x = float('-inf') if find_max else float('inf')
    extreme_y = float('-inf') if find_max else float('inf')

    for node in root.findall('node'):
        local_x = None
        local_y = None
        for tag in node.findall('tag'):
            if tag.attrib['k'] == 'local_x':
                local_x = float(tag.attrib['v'])
            elif tag.attrib['k'] == 'local_y':
                local_y = float(tag.attrib['v'])

        if local_x is not None and local_y is not None and local_x != float('inf') and local_y != float('inf'):
            if find_max:
                extreme_x = max(extreme_x, local_x)
                extreme_y = max(extreme_y, local_y)
            else:
                extreme_x = min(extreme_x, local_x)
                extreme_y = min(extreme_y, local_y)

    return extreme_x, extreme_y


def get_extreme_coords_commonroad(xml_path: str, find_max: bool = True):
    """
    Extracts the extreme (max or min) coordinates from a CommonRoad XML map.

    Args:
        xml_path (str): Path to .xml file.
        find_max (bool): Whether to find maximum (True) or minimum (False) coordinates.

    Returns:
        (float, float): (x, y) coordinates.
    """
    tree = etree.parse(xml_path)
    root = tree.getroot()

    extreme_x = float('-inf') if find_max else float('inf')
    extreme_y = float('-inf') if find_max else float('inf')

    for x in root.xpath(".//x"):
        local_x = float(x.text)
        if local_x != float('inf'):
            extreme_x = max(extreme_x, local_x) if find_max else min(extreme_x, local_x)

    for y in root.xpath(".//y"):
        local_y = float(y.text)
        if local_y != float('inf'):
            extreme_y = max(extreme_y, local_y) if find_max else min(extreme_y, local_y)

    return extreme_x, extreme_y


def main():
    # Max coordinates
    osm_max_x, osm_max_y = get_extreme_coords_osm(OSM_MAP_PATH, find_max=True)
    cr_max_x, cr_max_y = get_extreme_coords_commonroad(COMMONROAD_XML_PATH, find_max=True)

    # Min coordinates
    osm_min_x, osm_min_y = get_extreme_coords_osm(OSM_MAP_PATH, find_max=False)
    cr_min_x, cr_min_y = get_extreme_coords_commonroad(COMMONROAD_XML_PATH, find_max=False)

    # Offsets
    offset_x_max = cr_max_x - osm_max_x
    offset_y_max = cr_max_y - osm_max_y
    offset_x_min = cr_min_x - osm_min_x
    offset_y_min = cr_min_y - osm_min_y

    avg_offset_x = (offset_x_max + offset_x_min) / 2
    avg_offset_y = (offset_y_max + offset_y_min) / 2

    diff_x = abs(offset_x_max - offset_x_min)
    diff_y = abs(offset_y_max - offset_y_min)

    print(f"=== Lanelet2 to CommonRoad Coordinate Offsets ===")
    print(f"Max Offset: x = {offset_x_max:.4f}, y = {offset_y_max:.4f}")
    print(f"Min Offset: x = {offset_x_min:.4f}, y = {offset_y_min:.4f}")
    print(f"Difference between Max/Min Offsets: x = {diff_x:.4f}, y = {diff_y:.4f}")
    print(f"Average Offset: x = {avg_offset_x:.4f}, y = {avg_offset_y:.4f}")


if __name__ == "__main__":
    main()
