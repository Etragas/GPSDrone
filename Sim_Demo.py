from Simulation import *
from function_names import *
import numpy as np
from numpy import *

functions = {}
args = {}
real_fun(functions)

states = genfromtxt('/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/Traj7state.txt', delimiter=',',dtype=float32)
actions = genfromtxt('/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/Traj7action.txt' ,dtype=float32)


T,d = states.shape
#Create simulation
Sim = Simulation(function=functions["Traj7"], args=[[0,0,3],0])
Sim.restart()

for x in range(T):
    Sim.forward(actions[x,:].tolist())
    #Sim.forward()
    print(controller.getDif(Sim.cid,Sim.copter,Sim.target))
    print(actions[x,:])
    raw_input()
    Sim.sync()