from os import stat
import numpy as np
import sys 
from matplotlib import pyplot as plt
import time
import joint_filter

import rosbag
from sensor_msgs.msg import JointState

def read_bagfile(filename):
    #TODO: Error handling for file not being there
    bag_path = filename

    # Opening log file
    bag = rosbag.Bag(bag_path)
    joint_states = [x for x in bag.read_messages(topics=["/robot/joint_states"])]
    des_joints_name = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
    nb_des_joints = len(des_joints_name)

    ltimestamp = []
    ljoints_position = []
    ljoints_velocity = []

    position = np.zeros(nb_des_joints)
    velocity = np.zeros(nb_des_joints)

    counter = 0
    for i in range(0, len(joint_states)):
        if counter > 5000:
            pass #break
        counter += 1
        # Syncronizing
        secs = joint_states[i].message.header.stamp.secs 
        nsecs = joint_states[i].message.header.stamp.nsecs        

        joints_read = 0
        for (j, joint_name) in enumerate(joint_states[i].message.name):
            if joint_name in des_joints_name:
                position[joints_read] = joint_states[i].message.position[j]
                velocity[joints_read] = joint_states[i].message.velocity[j]
                joints_read += 1

        if (joints_read == nb_des_joints):
            ltimestamp.append(secs + nsecs*(1e-9))
            ljoints_position.append(np.copy(position))
            ljoints_velocity.append(np.copy(velocity))


    return np.array(ltimestamp), np.array(ljoints_position), np.array(ljoints_velocity)

if __name__ == '__main__':
    #TODO: Use better arg handling
    if len(sys.argv) < 2:
        print("You must provide the path to a bag file")
        sys.exit(-1)

    # Filters
    # BAXTER
    #avg_20_filter = joint_filter.FIRFilter(filter='mean', nb_samples=40)
    #iir_lin_filter = joint_filter.IIRLinearFilter(order=7, wn=.02)
    #iir_quad_filter = joint_filter.IIRQuadraticFitler(order=4, wn=2)  

    # Gazebo
    avg_20_filter = joint_filter.FIRFilter(filter='mean', nb_samples=20)
    iir_lin_filter = joint_filter.IIRLinearFilter(order=3, wn=1.5, fs=50)
    iir_quad_filter = joint_filter.IIRQuadraticFitler(order=3, wn=1.5, fs=50)  

    # Bagfile
    print('Reading bag file')
    timestamp, position, velocity = read_bagfile(sys.argv[1])
    #timestamp, position, velocity = read_bagfile('/home/romulo/baxter_datasets/ApertaoQ.bag')
    nb_measurements = len(timestamp)
    position = position.reshape([nb_measurements, 7])
    velocity = velocity.reshape([nb_measurements, 7])
    print('Read ', nb_measurements, "messages")
 
    # Filtering   
    state_avg_20_filter = np.zeros([nb_measurements, 7])
    state_lin_filter = np.zeros([nb_measurements, 7])
    state_quad_filter = np.zeros([nb_measurements, 7])
    total_time = np.zeros(3)
    for i in range(nb_measurements):
        tic = time.time()
        state_avg_20_filter[i,:] = avg_20_filter.filter(velocity[i,:])
        total_time[0] += time.time() - tic
        tic = time.time()
        state_lin_filter[i,:] = iir_lin_filter.filter(velocity[i,:])
        total_time[1] += time.time() - tic
        tic = time.time()
        state_quad_filter[i,:] = iir_quad_filter.filter(velocity[i,:])
        total_time[2] += time.time() - tic 
    print('avg time: ', total_time/nb_measurements)
    
    # Plots
    fig, ax =  plt.subplots(2,3)
    ax[0,0].plot(timestamp-timestamp[0], velocity[:,0], linestyle='-', color='brown', label='measurement')
    ax[0,1].plot(timestamp-timestamp[0], velocity[:,1], linestyle='-', color='brown')
    ax[0,2].plot(timestamp-timestamp[0], velocity[:,2], linestyle='-', color='brown')
    ax[1,0].plot(timestamp-timestamp[0], velocity[:,3], linestyle='-', color='brown')
    ax[1,1].plot(timestamp-timestamp[0], velocity[:,4], linestyle='-', color='brown')
    ax[1,2].plot(timestamp-timestamp[0], velocity[:,5], linestyle='-', color='brown')
    ax[0,0].plot(timestamp-timestamp[0], state_avg_20_filter[:,0], '-b', label='average filter')
    ax[0,1].plot(timestamp-timestamp[0], state_avg_20_filter[:,1], '-b')
    ax[0,2].plot(timestamp-timestamp[0], state_avg_20_filter[:,2], '-b')
    ax[1,0].plot(timestamp-timestamp[0], state_avg_20_filter[:,3], '-b')
    ax[1,1].plot(timestamp-timestamp[0], state_avg_20_filter[:,4], '-b')
    ax[1,2].plot(timestamp-timestamp[0], state_avg_20_filter[:,5], '-b')
    ax[0,0].plot(timestamp-timestamp[0], state_lin_filter[:,0], '-r', label='IIR linear')
    ax[0,1].plot(timestamp-timestamp[0], state_lin_filter[:,1], '-r')
    ax[0,2].plot(timestamp-timestamp[0], state_lin_filter[:,2], '-r')
    ax[1,0].plot(timestamp-timestamp[0], state_lin_filter[:,3], '-r')
    ax[1,1].plot(timestamp-timestamp[0], state_lin_filter[:,4], '-r')
    ax[1,2].plot(timestamp-timestamp[0], state_lin_filter[:,5], '-r')
    ax[0,0].plot(timestamp-timestamp[0], state_quad_filter[:,0], '-g', label='IIR quadratic')
    ax[0,1].plot(timestamp-timestamp[0], state_quad_filter[:,1], '-g')
    ax[0,2].plot(timestamp-timestamp[0], state_quad_filter[:,2], '-g')
    ax[1,0].plot(timestamp-timestamp[0], state_quad_filter[:,3], '-g')
    ax[1,1].plot(timestamp-timestamp[0], state_quad_filter[:,4], '-g')
    ax[1,2].plot(timestamp-timestamp[0], state_quad_filter[:,5], '-g')

    ax[0,0].legend()

    plt.show()

