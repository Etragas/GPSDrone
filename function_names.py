import numpy as np
from random import *
import math
def real_fun(functions):
    functions["Traj1"] = (lambda xt: np.asarray([xt[0][0], xt[0][1], xt[0][2] + 1])) # top
    functions["Traj2"] = (lambda xt: np.asarray([xt[0][0], xt[0][1], xt[0][2]])) # Middle
    functions["Traj3"] = (lambda xt: np.asarray([xt[0][0], xt[0][1], xt[0][2] - 1])) # Bottom

    #Upper ring
    functions["Traj4"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1], xt[0][2] +.5*1]))
    functions["Traj5"] = (lambda xt: np.asarray([xt[0][0] - .5, xt[0][1], xt[0][2] +.5*1]))
    functions["Traj6"] = (lambda xt: np.asarray([xt[0][0], xt[0][1] + .5, xt[0][2] +.5*1]))
    functions["Traj7"] = (lambda xt: np.asarray([xt[0][0], xt[0][1] - .5, xt[0][2] +.5*1]))
    functions["Traj8"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1] +.5, xt[0][2] +.5*1]))
    functions["Traj9"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1] -.5, xt[0][2] +.5*1]))
    functions["Traj10"] = (lambda xt: np.asarray([xt[0][0] - .5, xt[0][1] +.5, xt[0][2] +.5*1]))
    functions["Traj11"] = (lambda xt: np.asarray([xt[0][0] -.5 , xt[0][1] - .5, xt[0][2] +.5*1]))
    #Middle ring
    functions["Traj12"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1], xt[0][2] ]))
    functions["Traj13"] = (lambda xt: np.asarray([xt[0][0] - .5, xt[0][1], xt[0][2] ]))
    functions["Traj14"] = (lambda xt: np.asarray([xt[0][0], xt[0][1] + .5, xt[0][2] ]))
    functions["Traj15"] = (lambda xt: np.asarray([xt[0][0], xt[0][1] - .5, xt[0][2] ]))
    functions["Traj16"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1] +.5, xt[0][2] ]))
    functions["Traj17"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1] -.5, xt[0][2] ]))
    functions["Traj18"] = (lambda xt: np.asarray([xt[0][0] - .5, xt[0][1] +.5, xt[0][2] ]))
    functions["Traj19"] = (lambda xt: np.asarray([xt[0][0] -.5 , xt[0][1] - .5, xt[0][2] ]))
    #Lower ring
    functions["Traj20"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1], xt[0][2]-.5*1 ]))
    functions["Traj21"] = (lambda xt: np.asarray([xt[0][0] - .5, xt[0][1], xt[0][2] -.5*1]))
    functions["Traj22"] = (lambda xt: np.asarray([xt[0][0], xt[0][1] + .5, xt[0][2]-.5*1 ]))
    functions["Traj23"] = (lambda xt: np.asarray([xt[0][0], xt[0][1] - .5, xt[0][2]-.5*1 ]))
    functions["Traj24"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1] +.5, xt[0][2]-.5*1 ]))
    functions["Traj25"] = (lambda xt: np.asarray([xt[0][0] + .5, xt[0][1] -.5, xt[0][2]-.5*1 ]))
    functions["Traj26"] = (lambda xt: np.asarray([xt[0][0] - .5, xt[0][1] +.5, xt[0][2] -.5*1]))
    functions["Traj27"] = (lambda xt: np.asarray([xt[0][0] -.5 , xt[0][1] - .5, xt[0][2] -.5*1]))
    # functions["fuckIt"] = (lambda xt: np.asarray([xt[0][0]+uniform(-.5,.5), xt[0][1]+uniform(-.5,.5), xt[0][2]+uniform(-.5,.5)]))
    functions["AsinToZ"] = (lambda xt: np.asarray([xt[0][0] + math.sin(xt[1]), xt[0][1]+ math.sin(xt[1]), xt[0][2] + math.sin(xt[1])]))
    # functions["addxToZ"] = (lambda xt: np.asarray([xt[0][0], xt[0][1], xt[0][2] + xt[1]]))



def real_args(args,orgpos):
    for x in range(28):
        args["Traj" + str(x)] = [orgpos, 0]

