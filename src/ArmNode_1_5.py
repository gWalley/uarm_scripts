#!/usr/bin/env python
# coding:utf-8

import rospy
from FSM import *
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from cmvision_uarm.msg import Blobs

BLOCK = [60, 50]
BIN = [200, 220]
CAM = [150, 120]
XI = 0.005
YI = -0.02
IDLE_POSE = [0.0, 0.0, 90.0, 0.0]
HOME = [0.0, 0.0, 0.0, 0.0]
SEARCH = [120.0, 150.0, 0.0, 0.0]
MAX_JOINTS = [210, 150, 90, 90]
MIN_JOINTS = [0, -150, -90, -90]
RATE = 50
INC_RATE = 0.005


class ArmNode():

    def __init__(self, FSM):
        # Initiate ros Node
        rospy.init_node('uArm_Controller')

        # Setup Publishers
        # - Message to send commands to the arm
        self.joint_commands_pub = rospy.Publisher(
            '/uarm/joint_commands', Int16MultiArray)
        # - Message to tell the gripper to engage or disengage
        self.gripper_commands_pub = rospy.Publisher(
            '/uarm/gripper', Bool)
        # - Message to fully detach gripper and disable valve
        self.gripper_det_pub = rospy.Publisher(
            '/uarm/gripper_detach', Bool)
        # - Output of current state of uArm Controller
        self.cur_state_pub = rospy.Publisher(
            '/uarm/state', String)

        # Setup Subscribers
        # --- uarm subscription data
        #   - LS = Limit Switch: activated when arm presses down on an object
        rospy.Subscriber('/uarm/b/ls', Bool, self.LimitSwCB)
        #   - d4, d7: Buttons located on uArm Servo arduino shield
        rospy.Subscriber('/uarm/b/d4', Bool, self.B4CB)
        rospy.Subscriber('/uarm/b/d7', Bool, self.B7CB)
        #   - blobs: blobs being picked up in the camera matching the
        #     specified colours
        rospy.Subscriber('/uarm/blobs', Blobs, self.ObjPositionCB)
        # --- states of relevent robots and server
        rospy.Subscriber('/client_node/serv_state', String, self.ServerStateCB)
        rospy.Subscriber('/arm_bot_base/state', String, self.BaseStateCB)

        # Classwide Variable Init
        self.counter = 0
        self.atTarget = False
        self.error = [0.0, 0.0]
        self.prevError = [0.0, 0.0]
        self.blobName = []
        self.blobX = []
        self.blobY = []
        self.blobAreas = []
        self.currentPos = [0.0, 0.0, 0.0, 0.0]
        self.currentPosRnd = [0, 0, 0, 0]
        self.limitSw = False
        self.curState = None
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

        # Update position with calculated increments
        self.UpdatePosition(inc)

    # Checks if controller has reached target positon
    def AtTarget(self, targ):
        if self.currentPosRnd == targ:
            if self.atTarget is False:
                rospy.loginfo("At Target: " + str(self.currentPosRnd))
            self.atTarget = True
            return self.atTarget
        else:
            self.atTarget = False
            return self.atTarget

    # Given an incremental change, adjusts current position
    def UpdatePosition(self, inc):
        # Update each value incrementally
        for i in range(4):
            self.currentPos[i] = clamp(
                self.currentPos[i] + inc[i],  # Add inc to current position
                MIN_JOINTS[i], MAX_JOINTS[i])  # Clamp between min and max

            if (i == 3) and (self.FSM.curState.ReturnName() != "IDLE"):
                self.currentPos[i] = -self.currentPos[2]
            # Round current position to send to arm as integer
            self.currentPosRnd[i] = round(self.currentPos[i])

    # Centers on target location according to current error value
    def CenterOnBlob(self, colour):
        inc = [0.0, 0.0, 0.0, 0.0]
        # Parameter colour determines which blob to focus on,
        # and required margin of error
        if colour == "BLUE":
            flag = self.BlueBlobsSeen()
            lim = [14, 10]
        elif colour == "PINK":
            flag = self.PinkBlobsSeen()
            lim = [7, 5]

        inc[0] = clamp_p_or_n(YI * self.error[1], 0.1, 20)
        inc[1] = 0
        inc[2] = clamp_p_or_n(XI * self.error[0], 0.1, 20)
        inc[3] = 0

        error_s = (self.error[0] < lim[0] and self.error[0] > -lim[0]) and (
            self.error[1] < lim[1] and self.error[1] > -lim[1])

        if error_s and flag:
            self.counter = self.counter + 1

            if self.counter == 10:
                rospy.loginfo("Error small! COUNTER:" + str(self.counter))
                self.counter = 0
                return True
        else:
            self.counter = 0
            self.UpdatePosition(inc)
            return False

    # Determines if a pink blob is seen
    def PinkBlobsSeen(self):
        for i in range(len(self.blobName)):
            if self.blobName[i] == "PINK" and self.blobAreas[i] > 1000:
                return True
        return False

    # Determines if a blue blob is seen
    def BlueBlobsSeen(self):
        for i in range(len(self.blobName)):
            if self.blobName[i] == "BLUE" and self.blobAreas[i] > 1000:
                return True
        return False

    def ServerState(self):
        return self.serverState

    def BaseState(self):
        return self.baseState

    def LimitSw(self):
        return self.limitSw

    # Determines error when ALIGNing on block
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
            x = BIN[0]
            y = BIN[1]
        elif name == "BLOCK":
            x = BLOCK[0]
            y = BLOCK[1]
        elif name == "CAM":
            x = CAM[0]
            y = CAM[1]
        self.error[0] = posx[indexOfMax] - x
        self.error[1] = posy[indexOfMax] - y

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
        pass

    def B7CB(self, b7):
        if b7.data is True:
            self.FSM.ToTransition("to_IDLE")

    def LimitSwCB(self, ls):
        self.limitSw = ls.data

    def ServerStateCB(self, state):
        self.serverState = state.data
        # Reset to IDLE
        CS = self.FSM.curState.ReturnName()
        if((self.serverState == "IDLE" or self.serverState == "RESET") and CS != "IDLE"):
            self.FSM.ToTransition("to_IDLE")

    def BaseStateCB(self, state):
        if state.data != self.baseState:
            self.baseState = state.data
            rospy.loginfo(self.baseState)

    def ObjPositionCB(self, objPos):
        self.blobName = []
        self.blobX = []
        self.blobY = []
        self.blobAreas = []
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
# State: IDLE
# ---------------------------------------------------------------
# PrevState: N/A
# NextState:
# ---------------------------------------------------------------
#
class IDLE(State):

    def __init__(self, FSM, Arm):
        super(IDLE, self).__init__(FSM, Arm)
        self.EntryTime = None

    def Enter(self):
        rospy.loginfo("Entering IDLE State")
        self.EntryTime = rospy.get_time()
        self.Arm.GripPub(0)

    def Execute(self):
        if self.Arm.AtTarget(IDLE_POSE) is False:
            self.Arm.Move(IDLE_POSE, 0.1)
        elif self.Arm.ServerState() == "ARM_TO_OBJ":
            self.FSM.ToTransition("to_ARM_TO_OBJ")

        if rospy.get_time() > self.EntryTime + 1:
            self.Arm.GripDetPub()

    def Exit(self):
        rospy.loginfo("Exiting IDLE State")

    def ReturnName(self):
        return "IDLE"


class ARM_TO_OBJ(State):

    def __init__(self, FSM, Arm):
        super(ARM_TO_OBJ, self).__init__(FSM, Arm)
        self.EntryTime = None

    def Enter(self):
        rospy.loginfo("Entering ARM_TO_OBJ State")

    def Execute(self):
        if self.Arm.BaseState() == "FSM_WAIT_FOR_ACTION":
            self.FSM.ToTransition("to_SEARCH_OBJ")

    def Exit(self):
        rospy.loginfo("Exiting ARM_TO_OBJ State")

    def ReturnName(self):
        return "ARM_TO_OBJ"


# ---------------------------------------------------------------
# State: SEARCH_OBJ
# ---------------------------------------------------------------
# PrevState: WAIT_FOR_POSEect
# NextState: ALIGN_CAMERA
# ---------------------------------------------------------------
# As Arm_Base is rotating, look for the object.
# When object is seen, transition to ALIGN_CAMERA
class SEARCH_OBJ(State):

    def __init__(self, FSM, Arm):
        super(SEARCH_OBJ, self).__init__(FSM, Arm)
        self.EntryTime = None

    def Enter(self):
        rospy.loginfo("Entering SEARCH_OBJ State")
        self.EntryTime = rospy.get_time()

    def Execute(self):
        if rospy.get_time() < self.EntryTime + 3:
            if self.Arm.AtTarget(HOME) is False:
                self.Arm.Move(HOME, 0.1)
        else:
            if self.Arm.AtTarget(SEARCH) is False:
                self.Arm.Move(SEARCH, 0.1)
            elif self.Arm.PinkBlobsSeen() is True:
                self.FSM.ToTransition("to_ALIGN_CAMERA")

    def Exit(self):
        rospy.loginfo("Exiting SEARCH_OBJ State")

    def ReturnName(self):
        return "SEARCH_OBJ"


# ---------------------------------------------------------------
# State: ALIGN_CAMERA
# ---------------------------------------------------------------
# PrevState: SEARCH_OBJ
# NextState: WAITUser
# ---------------------------------------------------------------
# ALIGNs the camera to object in order to take a picture for
# for the user to see the object
class ALIGN_CAMERA(State):

    def __init__(self, FSM, Arm):
        super(ALIGN_CAMERA, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering ALIGN_CAMERA State")

    def Execute(self):
        self.Arm.Error("PINK", "CAM")
        if self.Arm.CenterOnBlob("PINK") is True:
            self.FSM.ToTransition("to_WAIT")

    def Exit(self):
        rospy.loginfo("Exiting ALIGN_CAMERA State")

    def ReturnName(self):
        return "ALIGN_CAMERA"


# ---------------------------------------------------------------
# State: WAIT
# ---------------------------------------------------------------
# PrevState: ALIGN_CAMERA
# NextState: WAIT
# ---------------------------------------------------------------
# WAITing
class WAIT(State):

    def __init__(self, FSM, Arm):
        super(WAIT, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering WAIT State")

    def Execute(self):
        if self.Arm.ServerState() == "BIN_AT_ARM":
            self.FSM.ToTransition("to_ALIGN_BLOCK")
        elif self.Arm.ServerState() == "ARM_DROPPING":
            self.FSM.ToTransition("to_LOCATE_BIN")

    def Exit(self):
        rospy.loginfo("Exiting WAIT State")

    def ReturnName(self):
        return "WAIT"


# ---------------------------------------------------------------
# State: ALIGN_BLOCK
# ---------------------------------------------------------------
# PrevState: WAIT
# NextState: APPROACH
# ---------------------------------------------------------------
#
class ALIGN_BLOCK(State):

    def __init__(self, FSM, Arm):
        super(ALIGN_BLOCK, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering ALIGN_BLOCK State")

    def Execute(self):
        self.Arm.Error("PINK", "BLOCK")
        if self.Arm.CenterOnBlob("PINK") is True:
            self.FSM.ToTransition("to_APPROACH")

    def Exit(self):
        rospy.loginfo("Exiting ALIGN_BLOCK State")

    def ReturnName(self):
        return "ALIGN_BLOCK"


# ---------------------------------------------------------------
# State: APPROACH
# ---------------------------------------------------------------
# PrevState: WAIT
# NextState: PICK_UP
# ---------------------------------------------------------------
#
class APPROACH(State):

    def __init__(self, FSM, Arm):
        super(APPROACH, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering APPROACH State")

    def Execute(self):
        inc = [-0.2, -2.5, 0.0, 0.0]
        self.Arm.UpdatePosition(inc)

        if self.Arm.LimitSw() is True:
            self.Arm.GripPub(1)
            rospy.sleep(0.5)
            self.FSM.ToTransition("to_PICK_UP")

    def Exit(self):
        rospy.loginfo("Exiting APPROACH State")

    def ReturnName(self):
        return "APPROACH"


# ---------------------------------------------------------------
# State: PICK_UP
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class PICK_UP(State):

    def __init__(self, FSM, Arm):
        super(PICK_UP, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering PICK_UP State")

    def Execute(self):
        if self.Arm.AtTarget(SEARCH) is True:
            self.FSM.ToTransition("to_VERIFY")
        else:
            self.Arm.Move(SEARCH, 0.1)

    def Exit(self):
        rospy.loginfo("Exiting PICK_UP State")

    def ReturnName(self):
        return "PICK_UP"


# ---------------------------------------------------------------
# State: VERIFY
# ---------------------------------------------------------------
# PrevState: ALIGN_CAMERA
# NextState: sd
# ---------------------------------------------------------------
# WAITing
class VERIFY(State):

    def __init__(self, FSM, Arm):
        super(VERIFY, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering VERIFY State")

    def Execute(self):
        if self.Arm.ServerState() == "ARM_DROPPING":
            self.FSM.ToTransition("to_LOCATE_BIN")

    def Exit(self):
        rospy.loginfo("Exiting VERIFY State")

    def ReturnName(self):
        return "VERIFY"


# ---------------------------------------------------------------
# State:
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class LOCATE_BIN(State):

    def __init__(self, FSM, Arm):
        super(LOCATE_BIN, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering LOCATE_BIN State")

    def Execute(self):
        if self.Arm.AtTarget(SEARCH) is False:
            self.Arm.Move(SEARCH, 0.1)
        elif self.Arm.BlueBlobsSeen() is True:
            self.FSM.ToTransition("to_ALIGN_BIN")

    def Exit(self):
        rospy.loginfo("Exiting LOCATE_BIN State")

    def ReturnName(self):
        return "LOCATE_BIN"


# ---------------------------------------------------------------
# State:
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class ALIGN_BIN(State):

    def __init__(self, FSM, Arm):
        super(ALIGN_BIN, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering ALIGN_BIN State")

    def Execute(self):
        if self.Arm.BlueBlobsSeen() is True:
            self.Arm.Error("BLUE", "BIN")
        else:
            self.Arm.prevError = self.Arm.error
            # self.Arm.UpdatePosition([0, 0, -2, 0])
        if self.Arm.CenterOnBlob("BLUE") is True:
            self.FSM.ToTransition("to_APPROACH_BIN")

    def Exit(self):
        rospy.loginfo("Exiting ALIGN_BIN State")

    def ReturnName(self):
        return "ALIGN_BIN"


# ---------------------------------------------------------------
# State: APPROACH_BIN
# ---------------------------------------------------------------
# PrevState: ALIGN_BIN
# NextState: DROPPED
# ---------------------------------------------------------------
#
class APPROACH_BIN(State):
    def __init__(self, FSM, Arm):
        super(APPROACH_BIN, self).__init__(FSM, Arm)
        self.lsPressed = False

    def Enter(self):
        self.lsPressed = False
        rospy.loginfo("Entering APPROACH_BIN State")

    def Execute(self):
        if self.lsPressed is True:
            if self.Arm.AtTarget(SEARCH) is False:
                self.Arm.Move(SEARCH, 0.1)
            else:
                rospy.sleep(0.5)
                self.FSM.ToTransition("to_DROPPED")
        else:
            # rospy.loginfo("APPROACH_BIN: Approaching Bin")
            inc = [-0.5, -2.5, 0.0, 0.0]
            self.Arm.UpdatePosition(inc)

        if (self.Arm.LimitSw() is True) and (self.lsPressed is False):
            rospy.loginfo("APPROACH_BIN: Block pressed down")
            if self.lsPressed is False:
                self.Arm.GripPub(0)
                rospy.sleep(0.5)
                self.lsPressed = True

    def Exit(self):
        rospy.loginfo("Exiting APPROACH_BIN State")

    def ReturnName(self):
        return "APPROACH_BIN"


# ---------------------------------------------------------------
# State:
# ---------------------------------------------------------------
# PrevState:
# NextState:
# ---------------------------------------------------------------
#
class DROPPED(State):

    def __init__(self, FSM, Arm):
        super(DROPPED, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering DROPPED State")

    def Execute(self):
        if self.Arm.AtTarget(HOME) is True:
            self.Arm.Move(HOME, 0.1)
        elif self.Arm.ServerState() == "BIN_TO_BASE":
            self.FSM.ToTransition("to_IDLE")

    def Exit(self):
        rospy.loginfo("Exiting DROPPED State")
        self.Arm.GripDetPub()

    def ReturnName(self):
        return "DROPPED"
