#!/usr/bin/env python3
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

import sys
from time import sleep
import math

import rclpy
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Float32

import urx

PUSH_DISTANCE = 0.41
PUSH_VEL = 0.1
PUSH_ACCEL = 0.1
APPROACH_VEL = 0.08
APPROACH_ACCEL = 0.08
MAX_X = 0.4
MIN_X = -0.4

class UR5Pusher:
    def push_position_callback(self, msg):
        print("push request received at x-pos " + str(msg.data))
        # self.robot.translate_tool((0,0,0.01))
        x_pos = max(-0.4, min(0.4, msg.data))
        print("performing push at x-pos " + str(x_pos))

        try:
            self.robot.movel((x_pos, -0.33, 0.07, math.pi/2, 0, 0), APPROACH_VEL, APPROACH_ACCEL)
            self.robot.translate_tool((0, 0, PUSH_DISTANCE), PUSH_VEL, PUSH_ACCEL)
            self.robot.translate_tool((0, 0, -PUSH_DISTANCE), PUSH_VEL, PUSH_ACCEL)
        except:
            # errors get thrown here because of a move timeout. Not really a
            # problem.
            pass

    def set_up_robot(self):
        # wait for connection
        connected = False
        while not connected:
            try:
                self.robot = urx.Robot("192.168.100.100")
                sleep(0.2)
                connected = True
            except:
                # couldn't connect
                print("attempting to connect to robot...")
                sleep(5)
        print('Connected to robot, current tool pose:', self.robot.getl())

    def __init__(self, args):
        if args is None:
            args = sys.argv
        rclpy.init(args=args)

        self.set_up_robot()

        node = rclpy.create_node('ur5_pusher')
        sub2 = node.create_subscription(Float32, '~/push_position', self.push_position_callback, qos_profile=qos_profile_sensor_data)
        # assert sub  # prevent unused warning

        interrupt = False
        while rclpy.ok() and not interrupt:
            try:
                rclpy.spin_once(node)
                sleep(0.1)
            except KeyboardInterrupt:
                interrupt = True

        self.robot.close()
        print('Disconnected from robot.')


def main(args=None):
    pr = UR5Pusher(args)

if __name__ == '__main__':
    main()
