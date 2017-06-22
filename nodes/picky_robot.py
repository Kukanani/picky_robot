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
from time import sleep, time
import math
from pprint import pprint

import rclpy
from rclpy.qos import qos_profile_sensor_data

from vision_msgs.msg import Detection3DArray
from sensor_msgs.msg import JointState

import urx

JOINT_STATE_TOPIC = "/joint_states"
PUSH_DISTANCE = 0.41
PUSH_VEL = 0.15
PUSH_ACCEL = 0.15
APPROACH_VEL = 0.08
APPROACH_ACCEL = 0.08
MAX_X = 0.4
MIN_X = -0.4
OBJ_TOPIC = '/detections'
READY_TOPIC = '~/arm_ready'
PUSH_TOPIC = '~/push_position'
DELTA_THRESHOLD = 0.03
STABLE_UPDATE_THRESHOLD = 5
JOINT_NAMES_MSG=  ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

current_nano_time = lambda: int(round(time() * 1e9 % 1e9))
current_sec_time = lambda: int(time())

class PickyRobot:

    def set_up_robot(self):
        print("attempting to connect to robot...")
        # wait for connection
        connected = False
        while not connected:
            try:
                self.robot = urx.Robot("192.168.100.100")
                connected = True
                msg_out = Bool()
                msg_out.data = connected
                self.pub.publish(msg_out)
            except:
                # couldn't connect
                print("attempting to connect to robot...")
        print('Connected to robot, current tool pose:', self.robot.getl())

    def update_joints(self):
        # print("updating joints...")
        msg = JointState()
        msg.header.stamp.sec = current_sec_time()
        msg.header.stamp.nanosec = current_nano_time()
        msg.name = JOINT_NAMES_MSG
        msg.position = self.robot.getj()
        self.joint_state_pub.publish(msg)

    def push_position(self, xpos):
        if self.arm_ready:
            self.arm_ready = False
            x_pos = max(-0.4, min(0.4, xpos))
            print("performing push at x-pos " + str(x_pos))

            try:
                print("move 1")
                self.robot.movel((x_pos, -0.33, 0.05, math.pi/2, 0, 0), APPROACH_ACCEL, APPROACH_VEL, False)
                while self.robot.is_moving():
                    self.update_joints()
                print("move 2")
                self.robot.translate_tool((0, 0, PUSH_DISTANCE), PUSH_ACCEL, PUSH_VEL, False)
                while self.robot.is_moving():
                    self.update_joints()
                print("move 3")
                self.robot.translate_tool((0, 0, -PUSH_DISTANCE), PUSH_ACCEL, PUSH_VEL, False)
                while self.robot.is_moving():
                    self.update_joints()
            except Exception as e:
                # errors get thrown here because of a move timeout. Not really a
                # problem.
                print(str(e))
            self.arm_ready = True

    def object_callback(self, msg):
        print("detections list received with " + str(len(msg.detections)) + " detections")
        for detection in msg.detections:
            if len(detection.results) > 0:
                hyp = detection.results[0]
                # do a "transformation" - rotate about z-axis and offset
                self.process_obj(hyp.id, -(hyp.pose.position.x/2) - self.x_offset)
        # print("callback over")

    def process_obj(self, id, xpos):
        # print('object detection received')
        if self.push[id] and self.arm_ready:
            if abs(xpos-self.last_xpos[id]) < DELTA_THRESHOLD:
                self.stable_updates[id] += 1
                # print('stable update')
            else:
                self.last_xpos[id] = xpos
                self.stable_updates[id] = 0
                # print("stable updates reset")
            if self.stable_updates[id] > STABLE_UPDATE_THRESHOLD:
                self.push_position(xpos)
                self.stable_updates[id] = 0
                self.last_xpos[id] = 10000

    def parse_picky_args(self, args):
        push_ramen = [True, False]
        push_pasta = [False, True]
        push_both = [True, True]

        self.push = push_both
        if "ramen" in args:
            self.push = push_ramen
        if "pasta" in args:
            self.push = push_pasta
        if "x_offset" in args:
            self.x_offset = float(args.index("x_offset")+1)
        else:
            self.x_offset = 0.1

    def ready_callback(self, msg):
        self.arm_ready = msg.data
        print("arm is ready")

    def __init__(self, args):
        if args is None:
            args = sys.argv
        self.parse_picky_args(args)
        rclpy.init()

        self.set_up_robot()

        self.last_xpos = [10000, 10000]
        self.stable_updates = [0,0]
        self.arm_ready = True

        self.node = rclpy.create_node('picky_robot')
        sub = self.node.create_subscription(Detection3DArray,
            OBJ_TOPIC, self.object_callback,
            qos_profile=qos_profile_sensor_data)
        self.joint_state_pub = self.node.create_publisher(JointState,
            JOINT_STATE_TOPIC)
        print("created pub/sub")

        interrupt = False
        while rclpy.ok() and not interrupt:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.01)
                self.update_joints()
            except KeyboardInterrupt:
                interrupt = True

        self.robot.close()
        print('Disconnected from robot.')

def main(args=None):
    pr = PickyRobot(args)

if __name__ == '__main__':
    main()
