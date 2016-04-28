import vrep
import numpy as np
import numpy as np
from math import *
import ctypes


firstPass = True
propellerScripts = [0]*4
def cleanse(X):
    pos, EitherFunOrVal = X[0],X[1]
    if (type(EitherFunOrVal) == type(cleanse)):
        EitherFunOrVal = EitherFunOrVal()
    return [pos,EitherFunOrVal]

def mode():
    global firstPass
    return vrep.simx_opmode_buffer if firstPass else vrep.simx_opmode_streaming

def target_move(cid,obj,function,*args):
    args = cleanse(args[0])
    pos = function(args)
    vrep.simxSetObjectPosition(cid,obj,-1,pos,mode())

def simGetObjectMatrix(cid,obj,relative):
    err, pos = vrep.simxGetObjectPosition(cid,obj,-1,mode())
    x,y,z = pos
    print(err)
    print("Values are {} {} {} {}".format(x,y,z,pos))
    err, angles = vrep.simxGetObjectOrientation(cid,obj,-1,mode())
    a,b,g = angles
    print(err)
    print("Angles are {} {} {} {}".format(a,b,g,angles))

    op = np.array([[0]*4]*4, dtype =np.float64)
    A = float(cos(a))
    B = float(sin(a))
    C = float(cos(b))
    D = float(sin(b))
    E = float(cos(g))
    F = float(sin(g))
    AD = float(A*D)
    BD = float(B*D)
    op[0][0]=float(C)*E
    op[0][1]=-float(C)*F
    op[0][2]=float(D)
    op[1][0]=float(BD)*E+A*F
    op[1][1]=float(-BD)*F+A*E
    op[1][2]=float(-B)*C
    op[2][0]=float(-AD)*E+B*F
    op[2][1]=float(AD)*F+B*E
    op[2][2]=float(A)*C
    op[0][3]=float(x)
    op[1][3]=float(y)
    op[2][3]=float(z)
    return op[0:3,:]

def getDif(cid,copter,target):
    copter_pos = vrep.simxGetObjectPosition(cid, copter, -1, mode())[1]
    copter_vel = vrep.simxGetObjectVelocity(cid, copter, mode())[1]
    copter_orientation = vrep.simxGetObjectOrientation(cid,copter,-1,mode())[1]
    target_pos = vrep.simxGetObjectPosition(cid, target, -1, mode())[1]
    target_vel = vrep.simxGetObjectVelocity(cid, target, mode())[1]

    return np.asarray([(-np.asarray(copter_pos) + np.asarray(target_pos)),(-np.asarray(copter_vel) + np.asarray(target_vel)),np.asarray(copter_orientation)]).flatten()

def controller_motor(cid, copter, targHandle):
    global firstPass
    global pParam
    global iParam
    global dParam
    global vParam
    global cumul
    global lastE
    global pAlphaE
    global pBetaE
    global  psp2
    global psp1
    global prevEuler
    global firstPass
    global particlesTargetVelocities
    global propellerScripts
    if (firstPass):
        propellerScripts=[-1,-1,-1,-1]
        for i in range(4):
            propellerScripts[i]= vrep.simxGetObjectHandle(cid,'Quadricopter_propeller_respondable'+str(i)+str(1),mode())

        particlesTargetVelocities = [0]*4
        pParam=.8
        iParam=0
        dParam=0
        vParam= .6

        cumul=0
        lastE=0
        pAlphaE=0
        pBetaE=0
        psp2=0
        psp1=0

        prevEuler=0
        print("OMG FIRSTPASS")
    print("PREV EULER")
    print(prevEuler)
    #Vertical control:
    #d = vrep.simxGetObjectHandle(cid,'Quadricopter_base',mode())[1]
    #targetPos=simGetObjectPosition(targetObj,-1)
    targetPos = vrep.simxGetObjectPosition(cid,targHandle,-1,mode())[1]
    print("Target_Pos IS")
    print(targetPos)
    #pos = simGetObjectPosition(d,-1)
    pos = vrep.simxGetObjectPosition(cid, copter, -1, mode())[1]

    l=vrep.simxGetObjectVelocity(cid, copter, mode())[1]
    print("l is:{}".format(l))
    e=(targetPos[2]-pos[2])
    cumul=cumul+e
    pv=pParam*e
    #thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+l[2]*vParam
    lastE=e
    thrust =5.335+ pParam * e +vParam *(0- l[2])
    print("THIS IS THRUST" + str(thrust))
    #Horizontal control:

    #sp=simGetObjectPosition(targetObj,d)
    sp = vrep.simxGetObjectPosition(cid,targHandle,copter,mode())[1]

    m=simGetObjectMatrix(cid,copter,-1)
    #m = vrep.simxGetJointMatrix(cid,d,mode())
    print(m)
    vx=np.array([1,0,0,1], dtype = np.float64)
    vx = np.reshape(vx,(4,1))
    #vx=simMultiplyVector(m,vx)
    print(m.shape)
    print(m)
    print(vx.shape)
    print(vx)
    vx = np.dot(m,vx)
    print("CHECK KEK")
    print(m.shape)
    print(m)
    print(vx.shape)
    print(vx)
    vy = np.array([0,1,0,1],dtype = np.float64)
    vy = np.reshape(vy,(4,1))

    vy=np.dot(m,vy)

    m = m.flatten()
    alphaE=(vy[2]-m[11])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[2]-m[11])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[1]*0.005+1*(sp[1]-psp2)
    betaCorr=betaCorr-sp[0]*0.005-1*(sp[0]-psp1)
    psp2=sp[1]
    psp1=sp[0]

    #-- Rotational control:

    #euler=simGetObjectOrientation(d,targetObj)
    euler = vrep.simxGetObjectOrientation(cid,copter,targHandle,mode())[1]
    rotCorr=euler[2]*0.1+2*(euler[2]-prevEuler)
    prevEuler=euler[2]

    #-- Decide of the motor velocities:
    particlesTargetVelocities[0]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[1]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[2]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr+betaCorr-rotCorr)

    #-- Send the desired motor velocities to the 4 rotors:
    packedData = vrep.simxPackFloats([x for x in particlesTargetVelocities])
    #packedData = vrep.simxPackFloats([100,100,100,100])
    raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData)
    err = vrep.simxSetStringSignal(cid, "rotorTargetVelocities",
                                        packedData,
                                        vrep.simx_opmode_oneshot)
    firstPass = False
    return particlesTargetVelocities
    #err = vrep.simxSetStringSignal(cid,'rotorTargetVelocities',packedData,mode())
