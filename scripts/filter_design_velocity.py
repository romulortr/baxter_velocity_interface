from os import stat
import numpy as np
import sys 
from matplotlib import pyplot as plt
import filter

import rosbag
from nav_msgs.msg import Odometry

from scipy import signal

def read_bagfile(filename):
    #TODO: Error handling for file not being there
    bag_path = filename

    # Opening log file
    bag = rosbag.Bag(bag_path)
    odom = [x for x in bag.read_messages(topics=["/robot/odom"])]
    
    lvelocity = []
    for i in range(0, len(odom)):
        # Syncronizing   
        vx = odom[i].message.twist.twist.linear.x
        vy = odom[i].message.twist.twist.linear.y
        vz = odom[i].message.twist.twist.linear.z
        wx = odom[i].message.twist.twist.angular.x
        wy = odom[i].message.twist.twist.angular.y
        wz = odom[i].message.twist.twist.angular.z
        vel = np.array([vx, vy, vz, wx, wy, wz])
        #print(' '.join(map(str,vel)))        
        lvelocity.append(np.copy(vel))

    return np.array(lvelocity)

if __name__ == '__main__':
    #TODO: Use better arg handling
    if len(sys.argv) < 2:
        print("You must provide the path to a bag file")
        sys.exit(-1)

    # Filters
    # BAXTER
    avg_25_filter = filter.FIRFilter(filter='mean', nb_samples=37, dim=6)
    avg_50_filter = filter.FIRFilter(filter='mean', nb_samples=75, dim=6)
    avg_100_filter = filter.FIRFilter(filter='mean', nb_samples=150, dim=6)

    # Bagfile
    velocity = read_bagfile(sys.argv[1])
   
    nb_measurements = len(velocity)
    velocity = velocity.reshape([nb_measurements, 6])
    timestamp = np.array([x*1./150 for x in range(0, nb_measurements)])
    # Filtering   
    state_avg_25_filter = np.zeros([nb_measurements, 6])
    state_avg_50_filter = np.zeros([nb_measurements, 6])
    state_avg_100_filter = np.zeros([nb_measurements, 6])
    state_filtfilt_filter = np.zeros([nb_measurements, 6])
       
    sos = signal.butter(N=4, Wn=.5, fs=150, output='sos')
    for i in range(0,6):
        state_filtfilt_filter[:,i] = signal.sosfiltfilt(sos, velocity[:,i])
    
    for i in range(nb_measurements):
        state_avg_25_filter[i,:] = avg_25_filter.filter(velocity[i,:])
        state_avg_50_filter[i,:] = avg_50_filter.filter(velocity[i,:])        
        state_avg_100_filter[i,:] = avg_100_filter.filter(velocity[i,:])        
        #print(' '.join(map(str,state_filtfilt_filter[i,:])))

    # Plots
    fig, ax =  plt.subplots(2,3)
    ax[0,0].plot(timestamp-timestamp[0], velocity[:,0], linestyle='-', color='yellow', label='measurement')
    ax[0,1].plot(timestamp-timestamp[0], velocity[:,1], linestyle='-', color='yellow')
    ax[0,2].plot(timestamp-timestamp[0], velocity[:,2], linestyle='-', color='yellow')
    ax[1,0].plot(timestamp-timestamp[0], velocity[:,3], linestyle='-', color='yellow')
    ax[1,1].plot(timestamp-timestamp[0], velocity[:,4], linestyle='-', color='yellow')
    ax[1,2].plot(timestamp-timestamp[0], velocity[:,5], linestyle='-', color='yellow')
    ax[0,0].plot(timestamp-timestamp[0], state_avg_25_filter[:,0], '-b', label='FIR AVG 37')
    ax[0,1].plot(timestamp-timestamp[0], state_avg_25_filter[:,1], '-b')
    ax[0,2].plot(timestamp-timestamp[0], state_avg_25_filter[:,2], '-b')
    ax[1,0].plot(timestamp-timestamp[0], state_avg_25_filter[:,3], '-b')
    ax[1,1].plot(timestamp-timestamp[0], state_avg_25_filter[:,4], '-b')
    ax[1,2].plot(timestamp-timestamp[0], state_avg_25_filter[:,5], '-b')
    ax[0,0].plot(timestamp-timestamp[0], state_avg_50_filter[:,0], '-g', label='FIR AVG 75')
    ax[0,1].plot(timestamp-timestamp[0], state_avg_50_filter[:,1], '-g')
    ax[0,2].plot(timestamp-timestamp[0], state_avg_50_filter[:,2], '-g')
    ax[1,0].plot(timestamp-timestamp[0], state_avg_50_filter[:,3], '-g')
    ax[1,1].plot(timestamp-timestamp[0], state_avg_50_filter[:,4], '-g')
    ax[1,2].plot(timestamp-timestamp[0], state_avg_50_filter[:,5], '-g')
    ax[0,0].plot(timestamp-timestamp[0], state_avg_100_filter[:,0], '-c', label='FIR AVG 100')
    ax[0,1].plot(timestamp-timestamp[0], state_avg_100_filter[:,1], '-c')
    ax[0,2].plot(timestamp-timestamp[0], state_avg_100_filter[:,2], '-c')
    ax[1,0].plot(timestamp-timestamp[0], state_avg_100_filter[:,3], '-c')
    ax[1,1].plot(timestamp-timestamp[0], state_avg_100_filter[:,4], '-c')
    ax[1,2].plot(timestamp-timestamp[0], state_avg_100_filter[:,5], '-c')    
    ax[0,0].plot(timestamp-timestamp[0], state_filtfilt_filter[:,0], '-r', label='Filtfilt')
    ax[0,1].plot(timestamp-timestamp[0], state_filtfilt_filter[:,1], '-r')
    ax[0,2].plot(timestamp-timestamp[0], state_filtfilt_filter[:,2], '-r')
    ax[1,0].plot(timestamp-timestamp[0], state_filtfilt_filter[:,3], '-r')
    ax[1,1].plot(timestamp-timestamp[0], state_filtfilt_filter[:,4], '-r')
    ax[1,2].plot(timestamp-timestamp[0], state_filtfilt_filter[:,5], '-r')


    ax[0,0].legend()

    plt.show()

