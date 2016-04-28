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
SYNC = True
firstPass = True
runtime = 0.0

def getTime():
    return runtime

def results(x,y):
    print("Error Code:{} Value:{}".format(x,y))

Q_BASE = 'Quadricopter_base'
Rotor_1 = 'Quadricopter_propeller_respondable1'
ONE_SHOT = vrep.simx_opmode_oneshot_wait
vrep_mode = vrep.simx_opmode_oneshot

def sync(cid):
    vrep.simxSynchronousTrigger(cid)

def getTargetStart(cid,target):
    start_pos = [0]
    while (max(start_pos) == 0):
        controller.controller_motor(cid, copter, target)
        err, start_pos = vrep.simxGetObjectPosition(cid, target, -1, mode())

    return err, np.asarray(start_pos)
def mode():
    global firstPass
    return vrep.simx_opmode_buffer if firstPass else vrep.simx_opmode_streaming


def connect():
    cid=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
    if cid != -1:
        print ('Connected to V-REP remote API serv'
               '\er, client id: %s' % cid)
        vrep.simxStartSimulation( cid, vrep.simx_opmode_oneshot )
        if SYNC:
            vrep.simxSynchronous( cid, True )
    else:
        print ('Failed connecting to V-REP remote API server')
        exit()
    return cid


print ('Program started')
# copter = Quadcopter(4,False,None,True,None,cid)
# copter = AdaptiveController.__init__(copter,True,0.001,None)
#
# print(copter)

#Start Grabbing Handles
functions = {}
args = {}
real_fun(functions)
#1-15 are examples along a sphere, one at top of sphere, one in middle, one in bottom,
#and then three rings at centred .25d, .5d, .75d with radius = 2


k = functions.keys()
k.sort()

iter = 1
for key in k:
    state_file = open('Trajectories/{}.txt'.format(key+"state"),'w+')
    action_file = open('Trajectories/{}.txt'.format(key+"action"),'w+')

    vrep.simxFinish(-1)  # just in case, close all opened connections
    time.sleep(1)
    cid = connect()
    time.sleep(1)
    vrep.simxLoadScene(cid,'/home/elias/etragas@gmail.com/_Winter2015/CSC494/Scenes/Base_Quad.ttt',0,mode())
    vrep.simxStartSimulation(cid,mode())
    runtime = 0

    err, copter = vrep.simxGetObjectHandle(cid, "Quadricopter_base",
                                           vrep.simx_opmode_oneshot_wait)

    err, target = vrep.simxGetObjectHandle(cid, "Quadricopter_target",
                                           vrep.simx_opmode_oneshot_wait)

    err, front_camera = vrep.simxGetObjectHandle(cid, 'Quadricopter_frontCamera', vrep.simx_opmode_oneshot)

    err, lin, ang = vrep.simxGetObjectVelocity(cid, copter, vrep.simx_opmode_streaming)
    err, orgpos = getTargetStart(cid,target)

    real_args(args,orgpos )

    args["addxToZ"] = [orgpos, 2.5]
    args["fuckIt"] = [orgpos, 2.5]


    #This prepared the data

    print(err)
    #while(1):


    while(runtime <= 10):
        error, pos = vrep.simxGetObjectPosition(cid,copter,-1,mode())
        args["AsinToZ"] = [pos, getTime]
        controller.target_move(cid,target,functions[key], args[key])
        commands = controller.controller_motor(cid,copter,target)

        vrep.simxSynchronousTrigger(cid)
        state_file.write("{}\n".format(",".join([str(x) for x in controller.getDif(cid,copter,target)])))
        action_file.write("{}\n".format(" ".join([str(x)[1:-2] for x in commands])))
        runtime += .05
        firstPass = False
    vrep.simxStopSimulation(cid, mode())
    vrep.simxSynchronousTrigger(cid)
    firstPass = True
    print("meow")

        # packedData=vrep.simxPackFloats([0,0,0,0])
        # err = vrep.simxSetStringSignal(cid, "rotorTargetVelocities",packedData,vrep_mode)
        # sync(cid)
        # err, res, image = vrep.simxGetVisionSensorImage(cid,front_camera,0,vrep.simx_opmode_streaming)
        # results(err,res)
        #
        #
        # err, res, image = vrep.simxGetVisionSensorImage(cid,front_camera,0,vrep.simx_opmode_oneshot_wait)
        # print(err)
        # print(image)
        # print(res)
    #
    # err = vrep.simxSetStringSignal(clientID, "rotorTargetVelocities",
    #                                 raw_bytes,
    #                                 vrep_mode)
    #
    #
    # errorCode, rotor1_joint = vrep.simxGetObjectHandle(clientID,'Quadricopter_propeller_joint1', ONE_SHOT)
    # results(errorCode,rotor1_joint)
    # rotor1_force = vrep.simxSetJointTargetVelocity(clientID,rotor1_joint,200,vrep.simx_opmode_streaming)
    # results(errorCode,rotor1_force)
    # time.sleep(2)
    #
    # #errorCode, rotor1_force = vrep.simxjoint(clientID,rotor1_joint,vrep.simx_opmode_buffer)
    #
    # results(errorCode,rotor1_force)
    #
    # #
    # # vrep.simxset
    # # vrep.simxSetJointTargetVelocity(clientID,rotor1,100000,vrep.simx_opmode_streaming)
    # # vrep.simx
    # #vrep.simxGetObjectVelocity(clientID,Q_BASE,ONE_SHOT)
    # print(rotor1_force)
    #
    # print("")

