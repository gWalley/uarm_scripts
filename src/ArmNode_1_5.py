#!/usr/bin/env python
# coding:utf-8

import rospy
from FSM import *
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from cmvision_uarm.msg import Blobs

BLOCK = [100, 50]
BIN = [240, 220]
CAM = [150, 120]
XI = 0.005
YI = -0.02
IDLE_POSE = [0.0, 0.0, 90.0, 0.0]
HOME_POSE = [0.0, 0.0, 0.0, 0.0]
SEARCH_POSE = [120.0, 150.0, 0.0, 0.0]
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

    # Move(targ, speed):
    #   - Given a target and speed, will calculate movement required to
    #     reach a given target
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

    # AtTarget(targ):
    #   - Checks if current working position of controller is equal to
    #     target positon
    def AtTarget(self, targ):
        if self.currentPosRnd == targ:
            if self.atTarget is False:
                rospy.loginfo("At Target: " + str(self.currentPosRnd))
            self.atTarget = True
            return self.atTarget
        else:
            self.atTarget = False
            return self.atTarget

    # UpdatePosition(inc):
    #   - Given an incremental change, adjusts current position
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

    # CenterOnBlob(colour):
    #   - Error() function should be called before this function to determine
    #     error value to be used
    #   - Centers on target location according to current error value of
    #     largest blob
    def CenterOnBlob(self, colour):
        inc = [0.0, 0.0, 0.0, 0.0]
        # Parameter colour determines which blob to focus on,
        # and required margin of error
        if colour == "BLUE":
            flag = self.BlueBlobsSeen()
            lim = [14, 10]
            speed = [0.0025, -0.01]

        elif colour == "PINK":
            flag = self.PinkBlobsSeen()
            lim = [7, 5]
            speed = [0.005, -0.02]

        inc[0] = clamp_p_or_n(speed[1] * self.error[1], 0.1, 20)
        inc[1] = 0
        inc[2] = clamp_p_or_n(speed[0] * self.error[0], 0.1, 20)
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

    # PinkBlobsSeen():
    #   - Determines if a pink blob is seen
    def PinkBlobsSeen(self):
        for i in range(len(self.blobName)):
            # Are there any pink blobs with area greater than 1000?
            if self.blobName[i] == "PINK" and self.blobAreas[i] > 1000:
                return True
        return False

    # BlueBlobsSeen():
    #   - Determines if a blue blob is seen
    def BlueBlobsSeen(self):
        for i in range(len(self.blobName)):
            # Are there any blue blobs with area greater than 1000?
            if self.blobName[i] == "BLUE" and self.blobAreas[i] > 1000:
                return True
        return False

    # ServerState():
    #   - Use to read the serverState Variable
    def ServerState(self):
        return self.serverState

    # BaseState():
    #   - Use to read the baseState variable
    def BaseState(self):
        return self.baseState

    # LimitSw():
    #   - Use to read the limitSw variable
    def LimitSw(self):
        return self.limitSw

    # # # # IMPROVEMENT POSSIBLE: # # # #
    # Call error from center on blob
    # function, or merge them
    # # # # # # # # # # # # # # # # # # #
    # Error(colour, name):
    #   - Determines error when aligning on block
    def Error(self, colour, name):
        # Initialise variables
        areas = []
        posx = []
        posy = []
        targ = []

        # Find all blobs of the desired colour
        for i in range(len(self.blobName)):
            if self.blobName[i] == colour:
                areas.append(self.blobAreas[i])
                posx.append(self.blobX[i])
                posy.append(self.blobY[i])
        # Determine largest of those blobs
        indexOfMax = areas.index(max(areas))

        # Determine target location as chosen by user
        if name == "BIN":
            targ = BIN
        elif name == "BLOCK":
            targ = BLOCK
        elif name == "CAM":
            targ = CAM

        # Calculate error by determining difference between current
        # location of blob and desired
        self.error[0] = posx[indexOfMax] - targ[0]
        self.error[1] = posy[indexOfMax] - targ[1]

    # Publisher Functions
    #  - When called, these functions will publish data to relevant topics

    # JointPub(): Publishes the rounded working position of the arm.
    def JointPub(self):
        self.joint_commands_pub.publish(data=self.currentPosRnd)

    # GripPub(0 or 1):
    #   - 1: activates pump.
    #   - 0: deactivates pump
    def GripPub(self, cmd):
        self.gripper_commands_pub.publish(data=cmd)

    # GripDetPub(): Used to disable both pump and valve
    #   - Should be called after pump has been disabled to disable the valve
    #   - This turns off the valve, closing it. Therefore, when using GripPub()
    #     this function should be called after some sleep period to allow the
    #     tube to normalize pressure
    def GripDetPub(self):
        self.gripper_det_pub.publish(data=1)

    # CurrentStatePub(): Publish current state of uArm Controller state machine
    def CurrentStatePub(self):
        self.cur_state_pub.publish(data=self.FSM.curState.ReturnName())

    # Callbacks for subscriptions
    def B4CB(self, b4):
        pass  # Currently unused

    # B7CB(): Callback for Button 7 on the arm
    #   - Forced reset of state machine incase of problems.
    def B7CB(self, b7):
        if b7.data is True:
            self.FSM.ToTransition("to_IDLE")

    # LimitSwCB(): Callback for limit switch on arm
    #   - When arm pushes down on something, button is pressed and limit switch
    #     is activated. This is stored in self.limitSw
    def LimitSwCB(self, ls):
        self.limitSw = ls.data

    # ServerStateCB(): Callback for when server state is updated
    #   - When server state is published, value is saved in self.serverState
    #   - When server transitions to IDLE or RESET state, as long as arm is
    #     not in IDLE, arm is also transitioned to IDLE
    def ServerStateCB(self, state):
        self.serverState = state.data
        # Reset to IDLE
        CS = self.FSM.curState.ReturnName()
        if((self.serverState == "IDLE" or self.serverState == "RESET")
           and CS != "IDLE"):
            self.FSM.ToTransition("to_IDLE")

    # BaseStateCB(): Callback for when arm base state is updated
    #   - When base state is published, value is saved in self.baseState
    def BaseStateCB(self, state):
        if state.data != self.baseState:
            self.baseState = state.data
            rospy.loginfo("BASE STATE TRANISIONING TO " + self.baseState)

    # ObjPositionCB(): Callback for /uarm/blobs topic, outputted from CMVision
    #   - Updates arrays storing blob info so that the array contains only
    #     blobs that can currently be seen.
    #   - Due to the number of blobs being unknown, the array is initialised
    #     to 0 size and blob data is appended to array.
    def ObjPositionCB(self, objPos):
        self.blobName = []
        self.blobX = []
        self.blobY = []
        self.blobAreas = []
        objPosLength = len(objPos.blobs)

        if objPosLength > 0:
            for i in range(objPosLength):
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
# NextState: ARM_TO_OBJ
# ---------------------------------------------------------------
# - IDLE state used to return arm to its IDLE_POSE.
# - Pump is disabled on entry, and after 1 second, the valve is
#   also disabled
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


# ---------------------------------------------------------------
# State: ARM_TO_OBJ
# ---------------------------------------------------------------
# PrevState: IDLE
# NextState: SERACH_OBJ
# ---------------------------------------------------------------
# - When base is navigating to object, Arm is in ARM_TO_OBJ state
# - The state is a dummy state to prevent issues with server
#   transitioning when it shouldn't
# COULD POTENTIALLY BE REMOVED
class ARM_TO_OBJ(State):

    def __init__(self, FSM, Arm):
        super(ARM_TO_OBJ, self).__init__(FSM, Arm)

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
# PrevState: WAIT_FOR_POS
# NextState: ALIGN_CAMERA
# ---------------------------------------------------------------
# - As Arm_Base is rotating, extends arm to look for object.
# - Object can be assumed to be roughly 180 degrees from arm
#   so the arm does not fully extend for 3 seconds.
# - When object is seen, transition to ALIGN_CAMERA
# THIS COULD BE MODIFYED TO PREVENT ARM HITTING WALLS:
# However, currently seems to not have this issue.
class SEARCH_OBJ(State):

    def __init__(self, FSM, Arm):
        super(SEARCH_OBJ, self).__init__(FSM, Arm)
        self.EntryTime = None

    def Enter(self):
        rospy.loginfo("Entering SEARCH_OBJ State")
        self.EntryTime = rospy.get_time()

    def Execute(self):
        if rospy.get_time() < self.EntryTime + 3:
            if self.Arm.AtTarget(HOME_POSE) is False:
                self.Arm.Move(HOME_POSE, 0.1)
        else:
            if self.Arm.AtTarget(SEARCH_POSE) is False:
                self.Arm.Move(SEARCH_POSE, 0.1)
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
# NextState: WAIT
# ---------------------------------------------------------------
# - ALIGNs the camera to object in order to take a picture for
#   for the user to see the object
class ALIGN_CAMERA(State):

    def __init__(self, FSM, Arm):
        super(ALIGN_CAMERA, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering ALIGN_CAMERA State")

    def Execute(self):
        # Calculate error for CenterOnBlob function
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
# NextState: ALIGN_BLOCK
# ---------------------------------------------------------------
# - Once user has made decision to pick up block, transitions to
#   ALIGN_BLOCK state
class WAIT(State):

    def __init__(self, FSM, Arm):
        super(WAIT, self).__init__(FSM, Arm)

    def Enter(self):
        rospy.loginfo("Entering WAIT State")

    def Execute(self):
        if self.Arm.ServerState() == "BIN_AT_ARM":
            self.FSM.ToTransition("to_ALIGN_BLOCK")

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
# - Aligns to block in order to pick it up
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
        if self.Arm.AtTarget(SEARCH_POSE) is True:
            self.FSM.ToTransition("to_VERIFY")
        else:
            self.Arm.Move(SEARCH_POSE, 0.1)

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
        if self.Arm.AtTarget(SEARCH_POSE) is False:
            self.Arm.Move(SEARCH_POSE, 0.1)
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
            if self.Arm.AtTarget(SEARCH_POSE) is False:
                self.Arm.Move(SEARCH_POSE, 0.1)
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
        if self.Arm.AtTarget(HOME_POSE) is True:
            self.Arm.Move(HOME_POSE, 0.1)
        elif self.Arm.ServerState() == "BIN_TO_BASE":
            self.FSM.ToTransition("to_IDLE")

    def Exit(self):
        rospy.loginfo("Exiting DROPPED State")
        self.Arm.GripDetPub()

    def ReturnName(self):
        return "DROPPED"
