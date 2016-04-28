import numpy as np
from lqr import *
import os
from function_names import *

def main():

    T = 201
    dx = 9
    du = 4

    states = np.zeros((1, T-1, dx))
    actions = np.zeros((1, T-1, du))
    state_normalizers = np.zeros((1,2))
    action_normalizers = np.zeros((1,2))
    directory = '/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/'
    names = ['Traj'+str(x) for x in range(1,28)]
    for var in names:
        #
        # if not file.startswith('Traj19') and not file.startswith('Traj11') and not file.startswith('Traj8') and not file.startswith('Traj27') \
        #         and not file.startswith('Traj25') and not file.startswith('Traj10') and not file.startswith('Traj18') \
        #         and not file.startswith('Traj9') and not file.startswith('Traj26') and not file.startswith('Traj21'):
#        if file.startswith('Traj13') or file.startswith('Traj16') or file.startswith('Traj5') or file.startswith('Traj7') or file.startswith('Traj23'):
        file = var+'state.txt'
        state = np.array(np.genfromtxt(directory + file, delimiter=','))
        #state = state[1:]
        print(file)
        state_normalizers = np.append(state_normalizers, np.array([[np.mean(state),(np.max(state)-np.min(state))]]),axis =0)
        print(state_normalizers[-1])
        state = (state - state_normalizers[-1][0]) / state_normalizers[-1][1]

        states = np.append(states, np.array([state]), axis=0)
        file = var + 'action.txt'
        print(file)
        action = np.array(np.genfromtxt(directory + file))
        #action = action[1:] #Keep only second and after
        action_normalizers = np.append(action_normalizers, np.array([[np.mean(action),(np.max(action)-np.min(action))]]),axis =0)
        print(action_normalizers [-1])
        action = (action- action_normalizers[-1][0]) / action_normalizers[-1][1]

        print(file)
        print(actions.shape)
        actions = np.append(actions, np.array([action]), axis=0)

    #States is 27xTx9
    #Actions is 27xTx4
    states = states[1:]
    actions = actions[1:]
    state_normalizers = state_normalizers[1:]
    action_normalizers = action_normalizers[1:]
    set_norms(state_normalizers, action_normalizers)
    print(state.shape)
    raw_input()

    #traj_opt(states, actions)
    sample_dist(states, actions)

if __name__ == "__main__":

    main()