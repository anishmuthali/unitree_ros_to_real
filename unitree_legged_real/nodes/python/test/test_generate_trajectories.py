#!/usr/bin/env python
import os
import numpy as np
import pdb
import time
from datetime import datetime
import pickle
import math

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm
import matplotlib

from utils.generate_vel_profile import get_velocity_profile_given_waypoints

markersize_x0 = 10
markersize_trajs = 0.4
fontsize_labels = 25
matplotlib.rc('xtick', labelsize=fontsize_labels)
matplotlib.rc('ytick', labelsize=fontsize_labels)
matplotlib.rc('text', usetex=False)
# matplotlib.rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
plt.rc('legend',fontsize=fontsize_labels+2)


if __name__ == "__main__":

    """
    
    Run as  python test/test_generate_trajectories.py

    """

    np.random.seed(1)

    rate_freq_send_commands = 100 # Hz
    deltaT = 1./rate_freq_send_commands

    time_tot = 15.0 # sec
    pos_waypoints = np.array(   [[0.0,0.0],
                                [1.5,1.5],
                                [-1.5,2.5],
                                [0.0,4.0]])

    state_tot, vel_tot = get_velocity_profile_given_waypoints(pos_waypoints,deltaT,time_tot,block_plot=True,plotting=True) # state_tot: [Nsteps_tot,2] || vel_tot: [Nsteps_tot,2]
