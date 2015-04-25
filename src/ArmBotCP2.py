#!/usr/bin/env python
# coding:utf-8
import rospy
import socket
import threading
import math

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from cmvision.msg import Blob
from cmvision.msg import Blobs
from ArmFSM import *

XMID = 175
YMID = 150
XI = 0.005
YI = -0.02
HOME = [150, 150, 0.0, 0.0]
LEFT = [150.0, 150.0, -90.0, 0.0]
RIGHT = [150.0, 150.0, 90.0, 0.0]
BIN = [210.0, -50, -60, 0.0]
RATE = 50
INC_RATE = 0.005
MAX_JOINTS = [210, 150, 90, 90]
MIN_JOINTS = [0, -150, -90, -90]

# State = type("State", (object,), {})


class ArmNode:

    def __init__(self):
        # Initiate ros Node
        rospy.init_node('Arm')

        # Setup Publishers
        self.joint_commands_pub = rospy.Publisher(
            '/uarm/joint_commands', Int16MultiArray)
        self.gripper_commands_pub = rospy.Publisher(
            '/uarm/gripper', Bool)
        self.gripper_det_pub = rospy.Publisher(
            '/uarm/gripper_detach', Bool)

        # Setup Subscribers
        rospy.Subscriber('/uarm/b/ls', Bool, self.limit_sw_cb)
        rospy.Subscriber('/blobs', Blobs, self.obj_position_cb)
        rospy.Subscriber('/uarm/b/d4', Bool, self.b_4_cb)
        rospy.Subscriber('/uarm/b/d7', Bool, self.b_7_cb)

        # Classwide Variable Init
        self.__arm_state = ARM_STATES.RESET
        self.counter = 0

        self.start = False
        self.bin_search = False
        self.blob_count = 0

        self.x_error = 0.0
        self.y_error = 0.0
        self.no_blobs = True

        self.x = [0.0, 0.0, 0.0, 0.0]
        self.y = [0.0, 0.0, 0.0, 0.0]

        self.areas = [0.0, 0.0, 0.0, 0.0]
        self.radius = [0.0, 0.0, 0.0, 0.0]
        self.circle = [False, False, False, False]

        self.move_error = [0.0, 0.0, 0.0, 0.0]

        self.current_pos = [0.0, 0.0, 0.0, 0.0]
        self.current_pos_rnd = [0, 0, 0, 0]
        self.increments = [0.0, 0.0, 0.0, 0.0]

        self.limit_sw = False

    def move(self, targ, speed=INC_RATE):
        if self.current_pos_rnd is targ:
            return 1

        else:
            inc = [0.0, 0.0, 0.0, 0.0]
            for i in range(4):
                if abs(targ[i] - self.current_pos_rnd[i]) < 1:
                    inc[i] = 0.0

                else:
                    inc[i] = clamp_p_or_n(
                        (targ[i] - self.current_pos_rnd[i]) * speed, 0.1, 10)
            rospy.loginfo(
                " Inc: " + str(inc) + "   Current Pos: " + str(
                    self.current_pos_rnd))
            self.update_position(inc)
            return 0

    def update_position(self, update):
        # update incrementally
        for i in range(4):
            self.current_pos[i] = clamp(
                self.current_pos[i] + update[i],
                MIN_JOINTS[i], MAX_JOINTS[i])
            self.current_pos_rnd[i] = round(self.current_pos[i])

    def triangle_blob(self):
        dist = [0 for x in range(6)]
        for i in range(3):
            dist[2 * i] = math.sqrt(math.pow(
                (self.x[i] - self.x[(i + 1) % 3]), 2) +
                math.pow((self.y[i] - self.y[(i + 1) % 3]), 2))

            dist[2 * i + 1] = math.sqrt(math.pow(
                (self.x[i] - self.x[(i + 2) % 3]), 2) +
                math.pow((self.y[i] - self.y[(i + 2) % 3]), 2))

    def center_on_blob(self):
        #
        rospy.loginfo(
            "  x: " + str(self.x_error) +
            "  y: " + str(self.y_error))

        self.increments[0] = clamp_p_or_n(YI * self.y_error, 0.1, 20)
        self.increments[1] = 0
        self.increments[2] = clamp_p_or_n(XI * self.x_error, 0.1, 20)
        self.increments[3] = 0

        error_small = (self.x_error < 7 and self.x_error > -7) and (
                self.y_error < 5 and self.y_error > -5)

        if error_small and not self.no_blobs:

            rospy.loginfo("Error small")
            self.counter = self.counter + 1

            rospy.loginfo("COUNTER:" + str(self.counter))

            if self.counter == 10:
                self.counter = 0
                return 1
        else:
            self.counter = 0
            self.update_position(self.increments)
            return 0

    def set_state(self, next_state):
        self.__arm_state = next_state
        rospy.loginfo("State: " + ARM_STATES.__getitem__(self.get_state()))

    def get_state(self):
        return self.__arm_state

    def joint_pub(self):
        self.joint_commands_pub.publish(data=self.current_pos_rnd)

    def grip_pub(self, cmd):
        self.gripper_commands_pub.publish(data=cmd)

    def grip_det_pub(self):
        self.gripper_det_pub.publish(data=1)

    # Callbacks
    def b_4_cb(self, b4):
        if b4.data is True:
            self.set_state(ARM_STATES.SEARCH_OBJ)

    def b_7_cb(self, b7):
        if b7.data is True:
            self.set_state(ARM_STATES.RESET)

    def limit_sw_cb(self, ls):
        self.limit_sw = ls.data

    def obj_position_cb(self, obj_pos):
        for i in range(4):
            try:
                circle_ratio = 0.0
                edge_ratio = 0.0
                radius = 0.0

                # ---------------------------------------------------------
                # Position of blobs
                self.x[i] = obj_pos.blobs[i].x
                self.y[i] = obj_pos.blobs[i].y
                self.areas[i] = obj_pos.blobs[i].area

                # ---------------------------------------------------------
                # Properties of blobs
                height = (obj_pos.blobs[i].bottom - obj_pos.blobs[i].top)
                width = (obj_pos.blobs[i].right - obj_pos.blobs[i].left)
                radius = (height + width) / 4
                edge_ratio = height / float(width)
                circle_ratio = self.areas[i] / float((math.pow(radius, 2)))
                # rospy.loginfo(edge_ratio)
                # rospy.loginfo(circle_ratio)

                # ---------------------------------------------------------
                # Is the blob a circle? Perhaps? Try all these to see
                flag = (edge_ratio < 1.3 and edge_ratio > 0.7) and (
                    circle_ratio > 2.95 and circle_ratio < 4.0) and (
                    self.areas[i] < 1000 and self.areas[i] > 650)

                if flag == 1:
                    self.circle[i] = True
                else:
                    self.circle[i] = False

                self.start = True
                self.no_blobs = False
                # self.blob_count = i + 1

            except:
                self.x[i] = 0
                self.y[i] = 0
                self.areas[i] = 0
                self.circle[i] = False
                if i == 0:
                    self.no_blobs = True

        index_of_max = 0
        x = 0
        y = 0
        blob_count = 0
        if self.no_blobs is False:
            if (self.bin_search is True):
                for i in range(4):
                    if self.circle[i] is True:
                        blob_count = blob_count + 1
                        x = x + self.x[i]
                        y = y + self.y[i]
                x = x / blob_count
                y = y / blob_count
                self.x_error = x - XMID
                self.y_error = y - YMID
                rospy.loginfo("Circle blobber is changing it")

            else:
                index_of_max = self.areas.index(max(self.areas))
                x = self.x[index_of_max]
                y = self.y[index_of_max]
                self.x_error = x - XMID
                self.y_error = y - YMID
                rospy.loginfo("big blobber is changing it")

        else:
            self.x_error = 0
            self.y_error = 0

        # rospy.loginfo(self.circle)


# ============================================================================
Char = type("Char", (object,), {})


class RobotArm(Char):
    def __init__(self):
        self.FSM = FSM(self)
        self.uArm = ArmNode()

        # STATES
        self.FSM.AddState("Idle", Idle(self.FSM))
        self.FSM.AddState("SearchObj", SearchObj(self.FSM))
        self.FSM.AddState("ObjApproach", ObjApproach(self.FSM))
        self.FSM.AddState("PickUp", PickUp(self.FSM))
        self.FSM.AddState("LocateBin", LocateBin(self.FSM))
        self.FSM.AddState("AlignBin", AlignBin(self.FSM))
        self.FSM.AddState("PlaceInBin", PlaceInBin(self.FSM))
        self.FSM.AddState("Reset", Reset(self.FSM))

        # TRANSITIONS
        self.FSM.AddTransition("toIdle", Transition("Idle"))
        self.FSM.AddTransition("toSearchObj", Transition("SearchObj"))
        self.FSM.AddTransition("toObjApproach", Transition("ObjApproach"))
        self.FSM.AddTransition("toPickUp", Transition("PickUp"))
        self.FSM.AddTransition("toLocateBin", Transition("LocateBin"))
        self.FSM.AddTransition("toAlignBin", Transition("AlignBin"))
        self.FSM.AddTransition("toPlaceInBin", Transition("PlaceInBin"))
        self.FSM.AddTransition("toReset", Transition("Reset"))

        self.FSM.SetState("Reset")


# ============================================================================
if __name__ == "__main__":
    try:
        rArm = RobotArm()
        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            rArm.Execute()
            r.sleep()

    except rospy.ROSInterruptException:
        pass
