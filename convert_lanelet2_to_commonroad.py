"""
convert_lanelet2_to_commonroad.py

Converts a Lanelet2 (.osm) map into a CommonRoad (.xml) scenario format
using the CRDesigner toolchain.

Usage:
    python3 convert_lanelet2_to_commonroad.py

Dependencies:
    - crdesigner
    - commonroad
"""

import os
from commonroad.scenario.scenario import Tag
from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.planning.planning_problem import PlanningProblemSet
from crdesigner.map_conversion.map_conversion_interface import lanelet_to_commonroad
from crdesigner.map_conversion.lanelet2.cr2lanelet import Lanelet2Config, GeneralConfig

# Configuration Constants
LANELET2_INPUT_PATH = "lanelet2_map.osm"
CRS_PROJECTION = "+proj=utm +zone=54 +datum=WGS84 +units=m +no_defs"
IS_LEFT_DRIVING = True
INCLUDE_ADJACENCIES = False

OUTPUT_DIR = "scenarios"
OUTPUT_FILE = "ZAM_AWSIM-1_1_T-1.xml"

AUTHOR_NAME = "Sebastian Maierhofer"
AUTHOR_AFFILIATION = "Technical University of Munich"
SOURCE_DESCRIPTION = "CommonRoad Scenario Designer"

def convert_lanelet2_to_commonroad():
    """
    Converts a Lanelet2 map file to a CommonRoad XML scenario.

    Reads the input .osm file, configures conversion parameters, performs
    the conversion, and writes the output .xml file.
    """
    lanelet_conf = Lanelet2Config()
    lanelet_conf.adjacencies = INCLUDE_ADJACENCIES
    lanelet_conf.left_driving = IS_LEFT_DRIVING

    general_conf = GeneralConfig()
    general_conf.proj_string_cr = CRS_PROJECTION

    scenario = lanelet_to_commonroad(
        input_file=LANELET2_INPUT_PATH,
        general_conf=general_conf,
        lanelet2_conf=lanelet_conf
    )

    # Create output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    output_path = os.path.join(OUTPUT_DIR, OUTPUT_FILE)

    writer = CommonRoadFileWriter(
        scenario=scenario,
        planning_problem_set=PlanningProblemSet(),
        author=AUTHOR_NAME,
        affiliation=AUTHOR_AFFILIATION,
        source=SOURCE_DESCRIPTION,
        tags={Tag.URBAN},
    )
    writer.write_to_file(output_path, OverwriteExistingFile.ALWAYS)

    print(f"Successfully converted map and saved to: {output_path}")


if __name__ == "__main__":
    convert_lanelet2_to_commonroad()
