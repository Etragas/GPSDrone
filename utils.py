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