import vrep
import time
import numpy as np
import scipy
import sys
import ctypes
import matplotlib
from random import *
from function_names import *
from quadcopter import *
import controller

TARGET = 'target'
QUAD = 'quadricopter'
SYNC = True
firstPass = True
runtime = 0.0


class Simulation():
    def __init__(self,**kwargs):
        global firstPass
        self.target = None
        self.copter = None
        self.cid = None
        self.function = kwargs['function']
        self.orgPos = -1
        self.args = kwargs['args']
        self.propellerScripts = [-1,-1,-1,-1]
        firstPass = True

    def initPropellers(self):
        for i in range(4):
            self.propellerScripts[i]= vrep.simxGetObjectHandle(self.cid,'Quadricopter_propeller_respondable'+str(i)+str(1),self.mode())
        particlesTargetVelocities = [0] * 4

    def connect(self):
        global SYNC
        self.cid = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP
        if self.cid != -1:
            print ('Connected to V-REP remote API serv'
                   '\er, client id: %s' % self.cid)
            vrep.simxStartSimulation(self.cid, vrep.simx_opmode_oneshot)
            if SYNC:
                vrep.simxSynchronous(self.cid, True)
        else:
            print ('Failed connecting to V-REP remote API server')
            exit()

    def getTargetStart(self):
        start_pos = [0]
        while (max(start_pos) == 0):
            controller.controller_motor(self.cid, self.copter, self.target)
            err, start_pos = vrep.simxGetObjectPosition(self.cid, self.target, -1, self.mode())
        self.orgPos = np.asarray(start_pos)

    def mode(self):
        global firstPass
        return vrep.simx_opmode_buffer if firstPass else vrep.simx_opmode_streaming


    def stop(self):
        vrep.simxStopSimulation(self.cid, self.mode())
        vrep.simxSynchronousTrigger(self.cid)

    def restart(self,earlyStop = False):
        if (self.cid != None):
            vrep.simxStopSimulation(self.cid, self.mode())
            vrep.simxSynchronousTrigger(self.cid)
        vrep.simxFinish(-1)  # just in case, close all opened connections
        time.sleep(1)
        self.connect()
        time.sleep(1)

        vrep.simxLoadScene(self.cid, '/home/elias/etragas@gmail.com/_Winter2015/CSC494/Scenes/Base_Quad.ttt', 0, self.mode())
        if earlyStop:
            vrep.simxStopSimulation(self.cid, self.mode())
            vrep.simxSynchronousTrigger(self.cid)
            vrep.simxFinish(-1)  # just in case, close all opened connections
            return
        vrep.simxStartSimulation(self.cid, self.mode())
        self.runtime = 0
        err, self.copter = vrep.simxGetObjectHandle(self.cid, "Quadricopter_base",
                                               vrep.simx_opmode_oneshot_wait)
        err, self.target = vrep.simxGetObjectHandle(self.cid, "Quadricopter_target",
                                               vrep.simx_opmode_oneshot_wait)

        err, self.front_camera = vrep.simxGetObjectHandle(self.cid, 'Quadricopter_frontCamera', vrep.simx_opmode_oneshot)

        err, lin, ang = vrep.simxGetObjectVelocity(self.cid, self.copter, vrep.simx_opmode_streaming)
        self.getTargetStart()
        for i in range(4):
            self.propellerScripts[i] = vrep.simxGetObjectHandle(self.cid,
                                                            'Quadricopter_propeller_respondable' + str(i) + str(1),
                                                            self.mode())

    def forward(self, action=None):
        controller.target_move(self.cid,self.target,self.function,self.args)
        if not(action is None):
            packedData = vrep.simxPackFloats([x for x in action])
            raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData)
            err = vrep.simxSetStringSignal(self.cid, "rotorTargetVelocities",
                                           packedData,
                                           vrep.simx_opmode_oneshot)
            firstPass = False
        else:
            controller.controller_motor(self.cid,self.copter,self.target)

    def sync(self):
        vrep.simxSynchronousTrigger(self.cid)


            #Skip controller

