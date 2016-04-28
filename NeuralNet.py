#Use NN to train on provides data
from Simulation import *
from function_names import *
import numpy as np
from numpy import *
import tensorflow as tf
import numpy as np

def shuffle_in_unison(a, b):
    assert len(a) == len(b)
    shuffled_a = np.empty(a.shape, dtype=a.dtype)
    shuffled_b = np.empty(b.shape, dtype=b.dtype)
    permutation = np.random.permutation(len(a))
    for old_index, new_index in enumerate(permutation):
        shuffled_a[new_index] = a[old_index]
        shuffled_b[new_index] = b[old_index]
    return shuffled_a, shuffled_b


class Net():


    def train(self,input,out,nhid,val_in,val_out):
        keep_prob = tf.placeholder(tf.float32)
        momentum = tf.placeholder(tf.float32)

        xDim = input.shape[1]
        nhid = nhid
        yDim = out.shape[1]
        sess = tf.InteractiveSession()
        x = tf.placeholder(tf.float32, shape=[None, xDim])
        dx = xDim
        dh = nhid
        dy = yDim
        y_ = tf.placeholder(tf.float32, shape=[None, yDim])
        Wx = tf.Variable(tf.random_normal([dx, dh]))
        Bx = tf.Variable(tf.random_normal([dh]))
        Wh = tf.Variable(tf.random_normal([dh, dy]))
        Bh = tf.Variable(tf.random_normal([dy]))
        h = tf.nn.xw_plus_b(x, Wx, Bx)#tf.nn.tanh(tf.matmul(x, Wx) + Bx) #tf.nn.xw_plus_b(x, Wx, Bx)#tf.nn.tanh(tf.matmul(x, Wx) + Bx)  #tf.nn.xw_plus_b(x, Wx, Bx)# # Consider replacing with xw_plus_b
        h_fc1_drop = tf.nn.dropout(h,keep_prob)
        y = tf.nn.xw_plus_b(h_fc1_drop, Wh, Bh)
        regularizers = (tf.nn.l2_loss(Wx) + tf.nn.l2_loss(Bx) + tf.nn.l2_loss(Wh) + tf.nn.l2_loss(Bh))

        loss = tf.reduce_mean((y_ - y)**2) + .001*regularizers
        train_step = tf.train.MomentumOptimizer(.01,momentum).minimize(loss)

        sess.run(tf.initialize_all_variables())

        mom = .95
        prev_acc = Inf
        for i in range(1000):
            train_step.run(feed_dict={x: input, y_: out, keep_prob:.5, momentum:mom})
            pred = sess.run([y],feed_dict={x: val_in, y_: out, keep_prob:1, momentum:mom})
            pred = np.asarray(pred,dtype=float32)[0,:,:]
            acc = (pred - val_out)**2
            #print(acc)
            # if ((acc-prev_acc)[0] > .1):
            #     print('meow')
            #prev_acc = acc
            print(np.mean(acc))
        return sess, y, x, keep_prob, momentum


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
print(state.shape)
functions = {}
args = {}
real_fun(functions)
#
norm_states = genfromtxt('/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/Traj1state.txt', delimiter=',',dtype=float32)
norm_actions = genfromtxt('/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/Traj1action.txt' ,dtype=float32)
#
state_mean, state_dif = np.mean(norm_states), (np.max(norm_states)-np.min(norm_states))
action_mean, action_dif = np.mean(norm_actions), (np.max(norm_actions)-np.min(norm_actions))
# states = (states - state_mean)/ (state_dif)
# actions = (actions - action_mean) / (action_dif)
#print(actions)
#raw_input()
T,d = states.shape
print(actions.shape)
#Create simulation
Sim = Simulation(function=functions["Traj1"], args=[[0,0,3],0])
nn = Net()
states, actions = shuffle_in_unison(states,actions)
train_states = states[:4800,:]
train_actions = actions[:4800,:]
val_states, val_actions =- states[4800:,:], actions[4800:,:]
sess, out, feed_me, keep_prob, momentum = nn.train(train_states,train_actions,10,val_states,val_actions)
Sim.restart()

for x in range(T):
    state = (controller.getDif(Sim.cid,Sim.copter,Sim.target))
    val = np.zeros((1,9))
    val[0,:] = state
    act = sess.run([out], feed_dict={feed_me:val, keep_prob:1})
    act = np.asarray(act,dtype=float32)[0,:,:]
    raw_input()
    print(act)
    Sim.forward(act[0].tolist())
    #Sim.forward()
    Sim.sync()

