#!/usr/bin/env python
# coding:utf-8


import math
import rospy
from FSM import *
# from BotClient import BotClient
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from cmvision_uarm.msg import Blobs
# from sensor_msgs.msg import Image

XBLOCK = 60
YBLOCK = 50
XBIN = 200
YBIN = 100
XCAM = 150
YCAM = 120
XI = 0.005
YI = -0.02
IDLE = [0.0, 0.0, 90.0, 0.0]
HOME = [100.0, 150.0, 0.0, 0.0]
SEARCH = [100.0, 150.0, 0.0, 0.0]
RATE = 50
INC_RATE = 0.005
MAX_JOINTS = [210, 150, 90, 90]
MIN_JOINTS = [0, -150, -90, -90]


class ArmNode():

    def __init__(self, FSM):
        # Initiate ros Node
        rospy.init_node('uArm_Controller')

        # Setup Publishers
        self.joint_commands_pub = rospy.Publisher(
            '/uarm/joint_commands', Int16MultiArray)
        self.gripper_commands_pub = rospy.Publisher(
            '/uarm/gripper', Bool)
        self.gripper_det_pub = rospy.Publisher(
            '/uarm/gripper_detach', Bool)
        self.cur_state_pub = rospy.Publisher(
            '/uarm/state', String)

        # Setup Subscribers
        # --- uarm subscription data
        rospy.Subscriber('/uarm/b/ls', Bool, self.LimitSwCB)
        rospy.Subscriber('/uarm/b/d4', Bool, self.B4CB)
        rospy.Subscriber('/uarm/b/d7', Bool, self.B7CB)
        rospy.Subscriber('/uarm/blobs', Blobs, self.ObjPositionCB)
        # --- states of relevent robots
        rospy.Subscriber('/client_node/serv_state', String, self.ServerStateCB)
        rospy.Subscriber('/arm_base/state', String, self.BaseStateCB)

        # Classwide Variable Init
        self.counter = 0
        self.xError = 0.0
        self.yError = 0.0
        self.blobName = []
        self.blobX = []
        self.blobY = []
        self.blobAreas = []
        self.blobHeights = []
        self.blobWidths = []
        self.blobCircle = []
        self.currentPos = [0.0, 0.0, 0.0, 0.0]
        self.currentPosRnd = [0, 0, 0, 0]
        self.limitSw = False
        self.serverState = None
        self.baseState = None
        self.FSM = FSM

    # Given a target and speed, will calculate movement required to
    # reach a given target
    def Move(self, targ, speed=INC_RATE):
        inc = [0.0, 0.0, 0.0, 0.0]
        # For all 4 axis, calculate incremental change
        for i in range(4):
            if abs(targ[i] - self.currentPos[i]) < 0.01:
                inc[i] = 0.0
            else:
                inc[i] = clamp_p_or_n(
                    (targ[i] - self.currentPosRnd[i]) * speed, 0.01, 10)
        # rospy.loginfo(
        #     " Inc: " + str(inc) + "   Current Pos: " + str(
        #         self.currentPosRnd))

        # Update position with calculated increments
        self.UpdatePosition(inc)

    # Checks if at target
    def AtTarget(self, targ):
        if self.currentPosRnd == targ:
            rospy.loginfo("At Target: " + str(self.currentPosRnd))
            return True
        else:
            return False

    # Given an incremental change, adjusts current position
    def UpdatePosition(self, inc):
        # update incrementally
        for i in range(4):
            self.currentPos[i] = clamp(
                self.currentPos[i] + inc[i],
                MIN_JOINTS[i], MAX_JOINTS[i])
            self.currentPosRnd[i] = round(self.currentPos[i])

    # Centers on target location according to current error value
    def CenterOnBlob(self, name):
        inc = [0.0, 0.0, 0.0, 0.0]
        if name == "Blue":
            flag = self.BlueBlobsSeen()
        elif name == "Pink":
            flag = self.PinkBlobsSeen()

        inc[0] = clamp_p_or_n(YI * self.yError, 0.1, 20)
        inc[1] = 0
        inc[2] = clamp_p_or_n(XI * self.xError, 0.1, 20)
        inc[3] = 0

        error_small = (self.xError < 7 and self.xError > -7) and (
            self.yError < 5 and self.yError > -5)

        if error_small and flag:
            self.counter = self.counter + 1

            if self.counter == 10:
                self.counter = 0
                rospy.loginfo("Error small! COUNTER:" + str(self.counter))
                return True
        else:
            self.counter = 0
            self.UpdatePosition(inc)
            return False

    # Returns BlobsSeen variable
    def PinkBlobsSeen(self):
        for i in range(len(self.blobName)):
            if self.blobName[i] == 'Pink' and self.blobAreas[i] > 1000:
                return True
        return False

    def BlueBlobsSeen(self):
        for i in range(len(self.blobName)):
            if self.blobName[i] == 'Blue' and self.blobAreas[i] > 1000:
                return True
        return False

    def ServerState(self):
        return self.serverState

    def BaseState(self):
        return self.baseState

    def LimitSw(self):
        return self.limitSw

    # Determines error when aligning on block
    def Error(self, colour, name):
        areas = []
        posx = []
        posy = []
        x = 0
        y = 0

        for i in range(len(self.blobName)):
            if self.blobName[i] == colour:
                areas.append(self.blobAreas[i])
                posx.append(self.blobX[i])
                posy.append(self.blobY[i])
        indexOfMax = areas.index(max(areas))

        if name == "BIN":
            x = XBIN
            y = YBIN
        elif name == "BLOCK":
            x = XBLOCK
            y = YBLOCK
        elif name == "CAM":
            x = XCAM
            y = YCAM
        self.xError = posx[indexOfMax] - x
        self.yError = posy[indexOfMax] - y

    # Publisher Functions
    def JointPub(self):
        self.joint_commands_pub.publish(data=self.currentPosRnd)

    def GripPub(self, cmd):
        self.gripper_commands_pub.publish(data=cmd)

    def GripDetPub(self):
        self.gripper_det_pub.publish(data=1)

    def CurrentStatePub(self):
        self.cur_state_pub.publish(data=self.FSM.curState.ReturnName())

    # Callbacks for subscriptions
    def B4CB(self, b4):
        if b4.data is True:
            self.FSM.ToTransition("toSearchObj")

    def B7CB(self, b7):
        if b7.data is True:
            self.FSM.ToTransition("toIdle")

    def LimitSwCB(self, ls):
        self.limitSw = ls.data

    def ServerStateCB(self, state):
        self.serverState = state.data
        # Reset to Idle
        CS = self.FSM.curState.ReturnName()
        if(self.serverState == "IDLE" and CS != "Idle"):
            self.FSM.ToTransition("toIdle")

    def BaseStateCB(self, state):
        self.baseState = state.data

    def ObjPositionCB(self, objPos):
        objPosLength = len(objPos.blobs)
        if objPosLength > 0:
            for i in range(objPosLength):
                # ---------------------------------------------------------
                # Position of blobs
                try:
                    self.blobName[i] = objPos.blobs[i].name
                    self.blobX[i] = objPos.blobs[i].x
                    self.blobY[i] = objPos.blobs[i].y
                    self.blobAreas[i] = objPos.blobs[i].area
                except:
                    self.blobName.append(objPos.blobs[i].name)
                    self.blobX.append(objPos.blobs[i].x)
                    self.blobY.append(objPos.blobs[i].y)
                    self.blobAreas.append(objPos.blobs[i].area)


# Clamps value between two limits
def clamp(n, minN, maxN):
    return max(min(maxN, n), minN)


def clamp_p_or_n(n, minN, maxN):
    if (n < 0):
        return max(min(-minN, n), -maxN)
    else:
        return max(min(maxN, n), minN)


# ---------------------------------------------------------------
# State: StartState
# ---------------------------------------------------------------
# PrevState: N/A
# NextState:
# ---------------------------------------------------------------
#
class StartState(State):

    def __init__(self, FSM, Arm):
        super(StartState, self).__init__(FSM, Arm)

    def Enter(self):
        pass

    def Execute(self):
        self.FSM.ToTransition("toIdle")

    def Exit(self):
        pass


# ---------------------------------------------------------------
# State: Idle
# ---------------------------------------------------------------
# PrevState: N/A
# NextState:
# ---------------------------------------------------------------
#
class Idle(State):

    def __init__(self, FSM, Arm):
        super(Idle, self).__init__(FSM, Arm)
        self.EntryTime = None

    def Enter(self):
        rospy.loginfo("Entering Idle State")
        self.EntryTime = rospy.get_time()
        self.Arm.GripPub(0)

    def Execute(self):
        if self.Arm.AtTarget(IDLE) == 0:
            self.Arm.Move(IDLE, 0.1)
        if self.Arm.ServerState() == "START":
            self.FSM.ToTransition("toWaitForObj")
        # self.FSM.ToTransition("toSearchObj")

        if rospy.get_time() > self.EntryTime + 1:
            self.Arm.GripDetPub()

    def Exit(self):
        rospy.loginfo("Exiting Idle State")

    def ReturnName(self):
        return "Idle"


# ---------------------------------------------------------------
# State: WaitForObject
# ---------------------------------------------------------------
# PrevState: Idle
# NextState: SearchObj
# ---------------------------------------------------------------
#
class WaitForObj(State):

    def __init__(self, FSM, Arm):
        super(WaitForObj, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Wait For Obj State")
        self.Arm.GripPub(0)

    def Execute(self):
        if self.Arm.ServerState() == "ARM_SEARCH":
            self.FSM.ToTransition("toSearchObj")

    def Exit(self):
        rospy.loginfo("Exiting Search Obj State")
        self.Arm.GripDetPub()

    def ReturnName(self):
        return "WaitForObj"


# ---------------------------------------------------------------
# State: SearchObj
# ---------------------------------------------------------------
# PrevState: WaitForObject
# NextState: AlignCamera
# ---------------------------------------------------------------
# As Arm_Base is rotating, look for the object.
# When object is seen, transition to AlignCamera
class SearchObj(State):

    def __init__(self, FSM, Arm):
        super(SearchObj, self).__init__(FSM, Arm)
        self.flag = False

    def Enter(self):
        rospy.loginfo("Entering Search Obj State")
        self.flag = False

    def Execute(self):

        if (self.Arm.AtTarget(SEARCH) is False) and (self.flag is False):
            self.Arm.Move(SEARCH, 0.1)
        else:
            self.flag = True
        self.Arm.GripPub(0)

        if (self.Arm.PinkBlobsSeen() is True) and (self.flag is True):
            self.FSM.ToTransition("toAlignCamera")
            # self.FSM.ToTransition("toAlign")

    def Exit(self):
        rospy.loginfo("Exiting Search Obj State")

    def ReturnName(self):
        return "SearchObj"


# ---------------------------------------------------------------
# State: AlignCamera
# ---------------------------------------------------------------
# PrevState: SearchObj
# NextState: WaitUser
# ---------------------------------------------------------------
# Aligns the camera to object in order to take a picture for
# for the user to see the object
class AlignCamera(State):

    def __init__(self, FSM, Arm):
        super(AlignCamera, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Align Camera State")

    def Execute(self):
        if self.Arm.PinkBlobsSeen() is True:
            self.Arm.Error("Pink", "CAM")
            if self.Arm.CenterOnBlob("Pink") == 1:
                self.FSM.ToTransition("toWait")

    def Exit(self):
        rospy.loginfo("Exiting Align Camera State")

    def ReturnName(self):
        return "AlignCamera"


# ---------------------------------------------------------------
# State: Wait
# ---------------------------------------------------------------
# PrevState: AlignCamera
# NextState: Wait
# ---------------------------------------------------------------
# Waiting
class Wait(State):

    def __init__(self, FSM, Arm):
        super(Wait, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Wait State")

    def Execute(self):
        # if self.Arm.ServerState() == "BIN_AT_ARM":
            # self.FSM.ToTransition("toAlign")
        if self.Arm.ServerState() == "ARM_PICKUP":
            self.FSM.ToTransition("toAlign")

    def Exit(self):
        rospy.loginfo("Exiting Wait State")

    def ReturnName(self):
        return "Wait"


# ---------------------------------------------------------------
# State: Align
# ---------------------------------------------------------------
# PrevState: Wait
# NextState: Approach
# ---------------------------------------------------------------
#
class Align(State):

    def __init__(self, FSM, Arm):
        super(Align, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Align State")

    def Execute(self):
        if self.Arm.PinkBlobsSeen() is True:
            self.Arm.Error('Pink', 'BLOCK')
            if self.Arm.CenterOnBlob("Pink") == 1:
                self.FSM.ToTransition("toApproach")

    def Exit(self):
        rospy.loginfo("Exiting Align State")

    def ReturnName(self):
        return "Align"


# ---------------------------------------------------------------
# State: Approach
# ---------------------------------------------------------------
# PrevState: Wait
# NextState: PickUp
# ---------------------------------------------------------------
#
class Approach(State):

    def __init__(self, FSM, Arm):
        super(Approach, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Approach State")

    def Execute(self):
        inc = [0.0, 0.0, 0.0, 0.0]
        inc[0] = -0.2
        inc[1] = -2.5
        inc[2] = 0
        inc[3] = 0
        self.Arm.UpdatePosition(inc)
        if self.Arm.LimitSw() is True:
            self.Arm.GripPub(1)
            rospy.sleep(0.5)
            self.FSM.ToTransition("toPickUp")

    def Exit(self):
        rospy.loginfo("Exiting Approach State")

    def ReturnName(self):
        return "Approach"


# ---------------------------------------------------------------
# State:
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class PickUp(State):

    def __init__(self, FSM, Arm):
        super(PickUp, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Pick Up State")

    def Execute(self):
        if self.Arm.AtTarget(HOME) is True:
            self.FSM.ToTransition("toLocateBin")
        else:
            self.Arm.Move(HOME, 0.1)

    def Exit(self):
        rospy.loginfo("Exiting Pick Up State")

    def ReturnName(self):
        return "PickUp"


# ---------------------------------------------------------------
# State:
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class LocateBin(State):

    def __init__(self, FSM, Arm):
        super(LocateBin, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Locate Bin State")

    def Execute(self):
        if self.Arm.AtTarget(SEARCH) is False:
            self.Arm.Move(SEARCH, 0.1)
        if self.Arm.BlueBlobsSeen() is True:
            self.FSM.ToTransition("toAlignToBin")

    def Exit(self):
        rospy.loginfo("Exiting Locate Bin State")

    def ReturnName(self):
        return "LocateBin"


# ---------------------------------------------------------------
# State:
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class AlignToBin(State):

    def __init__(self, FSM, Arm):
        super(AlignToBin, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Align To Bin State")

    def Execute(self):
        if self.Arm.BlueBlobsSeen() is True:
            self.Arm.Error('Blue', 'BIN')
            if self.Arm.CenterOnBlob("Blue") == 1:
                self.FSM.ToTransition("toDropped")

    def Exit(self):
        self.Arm.GripPub(0)
        rospy.loginfo("Exiting Align To Bin State")

    def ReturnName(self):
        return "AlignToBin"


# ---------------------------------------------------------------
# State:
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class Dropped(State):

    def __init__(self, FSM, Arm):
        super(Dropped, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering Dropped State")

    def Execute(self):
        # rospy.loginfo("Dropped")
        pass

    def Exit(self):
        rospy.loginfo("Exiting Dropped State")

    def ReturnName(self):
        return "Dropped"
