import numpy as np
from scipy import signal

class FIRFilter:
    def __init__(self, filter='mean', nb_samples=10, dim=6):
        self.nb_samples = nb_samples
        self.buffer = np.zeros([dim, nb_samples])
        self.iterator = 0
        # By default, do average filter
        if filter == 'median':
            self.func_filtering = np.median
        else:
            self.func_filtering = np.mean

    def filter(self, measurement):
        # Update buffer with new measurement
        self.buffer[:,self.iterator] = measurement
        self.iterator = (self.iterator+1) % self.nb_samples
        # Compute filter output
        return self.func_filtering(self.buffer, axis=1)

class IIRLinearFilter:
    def __init__(self, order=5, wn=.075, fs=150, dim=6):
        self.b, self.a = signal.butter(N=order, 
                                       Wn=wn, 
                                       #analog='False',
                                       output='ba',
                                       fs=fs)
        self.delay = np.zeros([dim, order])        

    def filter(self, measurement):
        z, self.delay = signal.lfilter(self.b, 
                                    self.a,
                                    x = measurement.reshape([-1,1]), 
                                    zi=self.delay)
        return z.flatten()

class IIRQuadraticFitler:
    def __init__(self, order=5, wn=20, fs=150, dim=6):
        self.sos = signal.butter(N=order, 
                                Wn=wn, 
                                #analog='False',
                                output='sos',
                                fs=fs)
        self.delay = np.zeros([self.sos.shape[0], dim, 2])

    def filter(self, measurement):
        z, self.delay = signal.sosfilt(self.sos,
                                        x = measurement.reshape([-1,1]), 
                                        zi=self.delay)
        return z.flatten()
          
#joint_names = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
