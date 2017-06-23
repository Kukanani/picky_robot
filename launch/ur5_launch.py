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

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher

file_path = os.path.dirname(os.path.realpath(__file__))


def launch():
    # parse a YAML file to get the calibration
    if len(sys.argv) == 2:
        calib_yaml_path = sys.argv[1]
    elif len(sys.argv) > 2:
        print("usage: ur5_launch.py [calibration_yaml_file_path]")
        exit()
    else:
        calib_yaml_path = os.path.dirname(os.path.realpath(__file__)) + os.sep + "osrf_calib.yaml"

    with open(calib_yaml_path, 'r') as f:
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
        cmd=['robot_state_publisher', os.path.join(file_path, 'ur5.urdf')]
    )

    ld.add_process(
        cmd=['picky_robot.py']
    )

    ld.add_process(
        cmd=['static_transform_publisher',
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
        cmd=['linemod_pipeline', 'linemod/cupnoodles_penne.yml', 'b']
    )

    ld.add_process(
        cmd=['depth_to_pointcloud_node']
    )

    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(ld)
    rc = launcher.launch()

    assert rc == 0, "The launch file failed with exit code '" + str(rc) + "'. "


if __name__ == "__main__":
    launch()
