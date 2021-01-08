import numpy as np
from matplotlib import pyplot as plt

def plot_velocity(data):
    time = data[:,0] - data[0,0]
    des_lin_vel_x, cur_lin_vel_x = data[:,1], data[:,1+6]
    des_lin_vel_y, cur_lin_vel_y = data[:,2], data[:,2+6]
    des_lin_vel_z, cur_lin_vel_z = data[:,3], data[:,3+6]
    des_ang_vel_x, cur_ang_vel_x = data[:,4], data[:,4+6]
    des_ang_vel_y, cur_ang_vel_y = data[:,5], data[:,5+6]
    des_ang_vel_z, cur_ang_vel_z = data[:,6], data[:,6+6]    

    fig, ax =  plt.subplots(2,3)
    ax[0,0].plot(time, des_lin_vel_x, linestyle='--')
    ax[0,0].plot(time, cur_lin_vel_x, linestyle='-')
    ax[0,1].plot(time, des_lin_vel_y, linestyle='--')
    ax[0,1].plot(time, cur_lin_vel_y, linestyle='-')
    ax[0,2].plot(time, des_lin_vel_z, linestyle='--')
    ax[0,2].plot(time, cur_lin_vel_z, linestyle='-')
    ax[1,0].plot(time, des_ang_vel_x, linestyle='--')
    ax[1,0].plot(time, cur_ang_vel_x, linestyle='-')
    ax[1,1].plot(time, des_ang_vel_y, linestyle='--')
    ax[1,1].plot(time, cur_ang_vel_y, linestyle='-')
    ax[1,2].plot(time, des_ang_vel_z, linestyle='--')
    ax[1,2].plot(time, cur_ang_vel_z, linestyle='-')

    plt.show()

if __name__ == "__main__":
    data = np.genfromtxt('data.csv', delimiter=" ")
    plot_velocity(data)
