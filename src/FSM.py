#!/usr/bin/env python
# coding:utf-8


# ============================================================================
# Definition of Base State Class
# ============================================================================
class State(object):

    def __init__(self, FSM, Arm):
        self.FSM = FSM
        self.Arm = Arm

    def Enter(self):
        pass

    def Execute(self):
        pass

    def Exit(self):
        pass

    def ReturnName(self):
        pass


class Transition(object):

    def __init__(self, toState):
        self.toState = toState

    def Execute(self):
        print("Transitioning...")


class ExampleState(State):

    def __init__(self, FSM):
        super(ExampleState, self).__init__(FSM)

    def Enter(self):
        pass

    def Execute(self):
        pass

    def Exit(self):
        pass


# ============================================================================
# ============================================================================
class FSM():

    def __init__(self):
        self.states = {}
        self.transitions = {}
        self.curState = None
        self.prevState = None
        self.trans = None

    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition

    def AddState(self, stateName, state):
        self.states[stateName] = state
        print 'Adding State: ' + stateName

    def SetState(self, stateName):
        self.prevState = self.curState
        self.curState = self.states[stateName]

    def ToTransition(self, toTrans):
        self.trans = self.transitions[toTrans]

    def Execute(self):
        if(self.trans):
            self.curState.Exit()
            self.trans.Execute()
            self.SetState(self.trans.toState)
            self.curState.Enter()
            self.trans = None
        self.curState.Execute()
