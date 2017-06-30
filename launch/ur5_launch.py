# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import yaml
import argparse

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher

from ament_index_python.packages import get_package_share_directory
from ros2run.api import get_executable_path

file_path = os.path.dirname(os.path.realpath(__file__))


def launch():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--linemod_templates", help="Path to a .yml file"
                        "containing the LINEMOD templates for the objects to "
                        "be detected. These could be generated with virtual "
                        "kinect (real) training methods. The file must "
                        "contain sequential class IDs and mesh paths for "
                        "each class to allow the pipeline to execute "
                        "correctly. ", type=str)
    parser.add_argument("-c", "--calibration_file", help="Path to a YAML "
                        "calibration file definining the transform from the "
                        "world frame to the camera frame used to collect "
                        "vision data. Please see launch/osrf_calib.yaml for "
                        "an example.", type=str)
    parser.add_argument("-p", "--push-pasta", help="Push pasta boxes off of "
                        "the table.", type=bool)
    parser.add_argument("-r", "--push-ramen", help="Push ramen containers off "
                        "of the table.", type=bool)
    args = parser.parse_args()

    if args.linemod_templates:
        linemod_templates = args.linemod_templates
    else:
        linemod_templates = (get_package_share_directory("picky_robot") +
                             os.sep + 'linemod/cupnoodles_penne.yml')

    if args.calibration_file:
        calibration_file = args.calibration_file
    else:
        calibration_file = (get_package_share_directory("picky_robot") +
                            os.sep + "launch" + os.sep + "osrf_calib.yaml")

    if args.push_pasta:
        push_pasta = args.push_pasta
    else:
        push_pasta = False

    if args.push_ramen:
        push_ramen = args.push_ramen
    else:
        push_ramen = False

    if not push_pasta and not push_ramen:
        push_ramen = True
        push_pasta = True
        return

    if push_pasta and push_ramen:
        push_flag = ""
    elif push_pasta:
        push_flag = "pasta"
    else:
        push_flag = "ramen"

    # parse a YAML file to get the calibration
    with open(calibration_file, 'r') as f:
        doc = yaml.load(f)
        x_offset = doc["x"]
        y_offset = doc["y"]
        z_offset = doc["z"]
        yaw_offset = doc["yaw"]
        pitch_offset = doc["pitch"]
        roll_offset = doc["roll"]
        parent_frame = doc["parent_frame"]
        child_frame = doc["child_frame"]

    ld = LaunchDescriptor()
    ld.add_process(
        cmd=[get_executable_path(package_name='robot_state_publisher',
                                 executable_name='robot_state_publisher'),
             (get_package_share_directory("picky_robot") + os.sep + "launch" +
              os.sep + "ur5.urdf")]
    )

    ld.add_process(
        cmd=[get_executable_path(package_name='tf2_ros',
                                 executable_name='static_transform_publisher'),
             str(doc["x"]),
             str(doc["y"]),
             str(doc["z"]),
             str(doc["yaw"]),
             str(doc["pitch"]),
             str(doc["roll"]),
             str(doc["parent_frame"]),
             str(doc["child_frame"])
             ]
        # '0.12', '-0.03', '0.95',
        # '0', '0', '-2.6',
        # 'world',
        # 'openni_color_optical_frame']
    )

    ld.add_process(
        cmd=[get_executable_path(package_name='depth_to_pointcloud',
                                 executable_name='depth_to_pointcloud_node')]
    )

    ld.add_process(
        cmd=[get_executable_path(package_name='picky_robot',
                                 executable_name='linemod_pipeline'),
             linemod_templates, 'b', 'b']
    )

    ld.add_process(
        cmd=[get_executable_path(package_name='picky_robot',
                                 executable_name='picky_robot.py'),
             push_flag]
    )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(ld)
    rc = launcher.launch()

    assert rc == 0, "The launch file failed with exit code '" + str(rc) + "'. "


if __name__ == "__main__":
    launch()
