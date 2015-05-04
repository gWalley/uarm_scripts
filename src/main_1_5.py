#!/usr/bin/env python
# coding:utf-8

from FSM import *
from ArmNode_1_5 import *


# ============================================================================
# ============================================================================
if __name__ == "__main__":
    try:
        rospy.loginfo("Initialising FSM")
        # Initialise Finite State Machine
        fsm = FSM()
        # Initialise ArmNode
        rArm = ArmNode(fsm)

        # STATE Initialisation
        fsm.AddState("IDLE",         IDLE(fsm, rArm))
        fsm.AddState("ARM_TO_OBJ",   ARM_TO_OBJ(fsm, rArm))
        fsm.AddState("SEARCH_OBJ",   SEARCH_OBJ(fsm, rArm))
        fsm.AddState("ALIGN_CAMERA", ALIGN_CAMERA(fsm, rArm))
        fsm.AddState("WAIT",         WAIT(fsm, rArm))
        fsm.AddState("ALIGN_BLOCK",  ALIGN_BLOCK(fsm, rArm))
        fsm.AddState("APPROACH",     APPROACH(fsm, rArm))
        fsm.AddState("PICK_UP",      PICK_UP(fsm, rArm))
        fsm.AddState("VERIFY",       VERIFY(fsm, rArm))
        fsm.AddState("LOCATE_BIN",   LOCATE_BIN(fsm, rArm))
        fsm.AddState("ALIGN_BIN",    ALIGN_BIN(fsm, rArm))
        fsm.AddState("APPROACH_BIN", APPROACH_BIN(fsm, rArm))
        fsm.AddState("DROPPED",      DROPPED(fsm, rArm))

        # STATE TRANSITION Initialisation
        fsm.AddTransition("to_IDLE",         Transition("IDLE"))
        fsm.AddTransition("to_ARM_TO_OBJ",   Transition("ARM_TO_OBJ"))
        fsm.AddTransition("to_SEARCH_OBJ",   Transition("SEARCH_OBJ"))
        fsm.AddTransition("to_ALIGN_CAMERA", Transition("ALIGN_CAMERA"))
        fsm.AddTransition("to_WAIT",         Transition("WAIT"))
        fsm.AddTransition("to_ALIGN_BLOCK",  Transition("ALIGN_BLOCK"))
        fsm.AddTransition("to_APPROACH",     Transition("APPROACH"))
        fsm.AddTransition("to_PICK_UP",      Transition("PICK_UP"))
        fsm.AddTransition("to_VERIFY", Transition("VERIFY"))
        fsm.AddTransition("to_LOCATE_BIN",   Transition("LOCATE_BIN"))
        fsm.AddTransition("to_ALIGN_BIN",    Transition("ALIGN_BIN"))
        fsm.AddTransition("to_APPROACH_BIN", Transition("APPROACH_BIN"))
        fsm.AddTransition("to_DROPPED",      Transition("DROPPED"))

        fsm.SetState("IDLE")
        fsm.ToTransition("to_IDLE")

        rospy.loginfo("Initialising ArmNode")

        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            fsm.Execute()
            rArm.CurrentStatePub()
            rArm.JointPub()
            r.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt Exception! Shutting down")
