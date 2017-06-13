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

from std_msgs.msg import Float32, Bool
from vision_msgs.msg import Detection3DArray

import urx

PUSH_TOPIC = '/ur5_pusher/push_position'
OBJ_TOPIC = '/detections'
READY_TOPIC = '/ur5_pusher/arm_ready'
DELTA_THRESHOLD = 0.03
STABLE_UPDATE_THRESHOLD = 5

class PickyRobot:
    def object_callback(self, msg):
        # print("detections list received")
        for detection in msg.detections:
            if len(detection.results) > 0:
                hyp = detection.results[0]
                # do a "transformation" - rotate about z-axis and offset
                self.process_obj(hyp.id, -(hyp.pose.position.x/2) - 0.1)

    def process_obj(self, id, xpos):
        if self.push[id] and self.arm_ready:
            if abs(xpos-self.last_xpos[id]) < DELTA_THRESHOLD:
                self.stable_updates[id] += 1
                # print('stable update')
            else:
                self.last_xpos[id] = xpos
                self.stable_updates[id] = 0
                # print("stable updates reset")
            if self.stable_updates[id] > STABLE_UPDATE_THRESHOLD:
                # print('detected a class-0 object, score', hyp.score)
                xpos_msg = Float32()
                xpos_msg.data = xpos
                print('sending push at x-position', xpos_msg.data)
                self.pub.publish(xpos_msg)
                self.stable_updates[id] = 0
                self.last_xpos[id] = 10000
                self.arm_ready = False

    def parse_picky_args(self, args):
        push_pasta = [True, False]
        push_ramen = [False, True]
        push_everything = [True, True]

        self.push = push_everything
        if "ramen" in args:
            self.push = push_ramen
        if "pasta" in args:
            self.push = push_pasta

    def ready_callback(self, msg):
        self.arm_ready = msg.data
        print("arm is ready")

    def __init__(self, args):
        if args is None:
            args = sys.argv
        rclpy.init(args=args)

        self.last_xpos = [10000, 10000]
        self.stable_updates = [0,0]
        self.arm_ready = True

        self.parse_picky_args(args)

        self.node = rclpy.create_node('picky_robot')
        self.pub = self.node.create_publisher(
            Float32, PUSH_TOPIC)
        ready_sub = self.node.create_subscription(Bool,
            READY_TOPIC, self.ready_callback,
            qos_profile=qos_profile_sensor_data)
        sub = self.node.create_subscription(Detection3DArray,
            OBJ_TOPIC, self.object_callback,
            qos_profile=qos_profile_sensor_data)
        print("created pub/sub")

        while rclpy.ok():
            rclpy.spin_once(self.node)
            sleep(0.1)

def main(args=None):
    pr = PickyRobot(args)

if __name__ == '__main__':
    main()
