import numpy as np


class LinearGaussianPolicy:

    def __init__(self, K, k, covar, inv_covar):

        # T x du x dx
        self.dimensions = K.shape
        self.K = K
        self.k = k
        self.covar = covar
        self.inv_covar = inv_covar


class Dynamics:

    def __init__(self):

        self.Fm = None
        self.fv = None
        self.covar = None
        self.x0sigma = None
        self.x0mu = None

    def fit(self, traj_state, traj_actions, reg=1):

        N, T, dx = traj_state.shape
        du = traj_actions.shape[2]

        self.Fm = np.zeros([T, dx, dx+du])
        self.fv = np.zeros([T, dx])
        self.covar = np.zeros([T, dx, dx])

        both_slice = slice(dx+du)
        xux_slice = slice(dx+du, dx+du+dx)

        for t in range(T - 1):
            xux = np.c_[traj_state[:,t,:], traj_actions[:,t,:], traj_state[:,t+1,:]]
            xux_mean = np.mean(xux, axis=0)
            xux_cov = (xux - xux_mean).T.dot(xux - xux_mean) / (N - 1)
            # xux_cov = np.cov(xux)

            sigma = 0.5 * (xux_cov +  xux_cov.T)
            sigma[both_slice, both_slice] += reg * np.eye(dx + du)
            # regularize ?

            Fm = np.linalg.pinv(sigma[both_slice, both_slice]).dot(sigma[both_slice, xux_slice]).T
            fv = xux_mean[xux_slice] - Fm.dot(xux_mean[both_slice])
            self.Fm[t, :, :] = Fm
            self.fv[t, :] = fv

            covar = sigma[xux_slice, xux_slice] - Fm.dot(sigma[both_slice, both_slice]).dot(Fm.T)
            self.covar[t, :] = 0.5* (covar + covar.T)

            if t == 0:
                self.x0mu = np.mean(traj_state[:,t,:])
                self.x0sigma = np.diag(np.var(traj_state[:,t,:], axis=0))


def get_flat_files():
    """
    This is not the same as the version seen in traj_opt, this returns a [N*T,9] and [N*T,4] (N is num trajectories)
    :return:
    """
    T = 201
    dx = 9
    du = 4

    states = np.zeros((T-1, dx))
    actions = np.zeros((T-1, du))
    state_normalizers = np.zeros((1,2))
    action_normalizers = np.zeros((1,2))
    directory = '/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/'
    names = ['Traj'+str(x) for x in range(1,28)]
    for var in names:
        file = var+'state.txt'
        state = np.array(np.genfromtxt(directory + file, delimiter=','))
        #state = state[1:]
        print(file)
        # state_normalizers = np.append(state_normalizers, np.array([[np.mean(state),(np.max(state)-np.min(state))]]),axis =0)
        print(state_normalizers[-1])
        # state = (state - state_normalizers[-1][0]) / state_normalizers[-1][1]

        states = np.append(states, np.array([state])[0,:,:], axis=0)
        file = var + 'action.txt'
        print(file)
        action = np.array(np.genfromtxt(directory + file))
        #action = action[1:] #Keep only second and after
        # action_normalizers = np.append(action_normalizers, np.array([[np.mean(action),(np.max(action)-np.min(action))]]),axis =0)
        print(action_normalizers [-1])
        # action = (action- action_normalizers[-1][0]) / action_normalizers[-1][1]

        print(file)
        print(actions.shape)
        actions = np.append(actions, np.array([action])[0,:,:], axis=0)

    #States is 27xTx9
    #Actions is 27xTx4
    states = states[1:]
    actions = actions[1:]
    # state_normalizers = state_normalizers[1:]
    # action_normalizers = action_normalizers[1:]
    return states,actions