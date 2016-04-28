#Use NN to train on provides data
from Simulation import *
from utils import *
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
        for i in range(100):
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

functions = {}
args = {}
real_fun(functions)
#
norm_states = genfromtxt('/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/Traj1state.txt', delimiter=',',dtype=float32)
norm_actions = genfromtxt('/home/elias/etragas@gmail.com/_Winter2015/CSC494/Trajectories/Traj1action.txt' ,dtype=float32)
states,actions = get_flat_files()
#Create simulation
Sim = Simulation(function=functions["Traj1"], args=[[0,0,3],0])
nn = Net()
states, actions = shuffle_in_unison(states,actions)
T,d = states.shape
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

