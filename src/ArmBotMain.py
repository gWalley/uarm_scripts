#!/usr/bin/env python
# coding:utf-8

# import socket
# import threading

from FSM import *
from ArmNode_1_3 import *


# Char = type("Char", (object,), {})

# ============================================================================
# ============================================================================
if __name__ == "__main__":
    try:
        rospy.loginfo("Initialising FSM")
        # Initialise Finite State Machine
        fsm = FSM()

        rArm = ArmNode(fsm)

        # STATE Initialisation
        fsm.AddState("START_STATE", START_STATE(fsm, rArm))
        fsm.AddState("IDLE", IDLE(fsm, rArm))
        fsm.AddState("WAIT_FOR_POSE", WAIT_FOR_POSE(fsm, rArm))
        fsm.AddState("WAIT_FOR_BASE", WAIT_FOR_BASE(fsm, rArm))
        fsm.AddState("SEARCH_OBJ", SEARCH_OBJ(fsm, rArm))
        fsm.AddState("ALIGN_CAMERA", ALIGN_CAMERA(fsm, rArm))
        fsm.AddState("WAIT", WAIT(fsm, rArm))
        fsm.AddState("ALIGN", ALIGN(fsm, rArm))
        fsm.AddState("APPROACH", APPROACH(fsm, rArm))
        fsm.AddState("PICK_UP", PICK_UP(fsm, rArm))
        fsm.AddState("LOCATE_BIN", LOCATE_BIN(fsm, rArm))
        fsm.AddState("ALIGN_BIN", ALIGN_BIN(fsm, rArm))
        fsm.AddState("DROPPED", DROPPED(fsm, rArm))

        # STATE TRANSITION Initialisation
        fsm.AddTransition("toIDLE", Transition("IDLE"))
        fsm.AddTransition("toWAIT_FOR_POSE", Transition("WAIT_FOR_POSE"))
        fsm.AddTransition("toWAIT_FOR_BASE", Transition("WAIT_FOR_BASE"))
        fsm.AddTransition("toSEARCH_OBJ", Transition("SEARCH_OBJ"))
        fsm.AddTransition("toALIGN_CAMERA", Transition("ALIGN_CAMERA"))
        fsm.AddTransition("toWAIT", Transition("WAIT"))
        fsm.AddTransition("toALIGN", Transition("ALIGN"))
        fsm.AddTransition("toAPPROACH", Transition("APPROACH"))
        fsm.AddTransition("toPICK_UP", Transition("PICK_UP"))
        fsm.AddTransition("toLOCATE_BIN", Transition("LOCATE_BIN"))
        fsm.AddTransition("toALIGN_BIN", Transition("ALIGN_BIN"))
        fsm.AddTransition("toDROPPED", Transition("DROPPED"))

        fsm.SetState("IDLE")
        fsm.ToTransition("toIDLE")

        rospy.loginfo("Initialising ArmNode")

        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            fsm.Execute()
            rArm.CurrentStatePub()
            rArm.JointPub()
            r.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt Exception! Shutting down")
