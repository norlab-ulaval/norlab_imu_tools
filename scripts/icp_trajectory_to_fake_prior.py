import math
from pathlib import Path

import numpy as np
import pandas as pd
#from pygeodesy.utm import toUtm
#import seaborn as sns
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d

# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D

from Tkinter import Tk
from tkFileDialog import askopenfilenames




def get_filename_from_user():
    root = Tk()
    root.withdraw()
    filenames = askopenfilenames(defaultextension='.svg',
                                 filetypes=[('Odometry output', '.csv')],
                                 title="Select .csv files",
                                 multiple=True)
    return filenames




if __name__ == '__main__':


    file_paths = get_filename_from_user()
    #file_paths = ['/home/vlad/norlab_data/scutigera_bags/20190718_Foret_Montmorency_Jeffs_spots/2019-07-24-11-53-48_icp_odom.csv']
    print(file_paths)

    icp_output = None
    prior_output = None

    for file_path in file_paths:
        path = Path(file_path)
        filename = path.name
        print(filename)

        if "_icp_odom" in filename:
            with path.open() as file:
                icp_output = pd.read_csv(file, delimiter=',')
                icp_output.columns = ['secs', 'nsecs', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']


        if "imu_odom" in filename:
            with path.open() as file:
                prior_output = pd.read_csv(file, delimiter=',')
                prior_output.columns = ['secs', 'nsecs', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']

    if icp_output is None:
        raise RuntimeError("Icp file missing")

    # if icp_output.shape[0] != prior_output.shape[0]:
    #     min_len = min(icp_output.shape[0], prior_output.shape[0])
    #     icp = icp_output.iloc[:min_len, :]
    #     gps = prior_output.iloc[:min_len, :]




    icp_time = 1.0*icp_output['secs'].to_numpy() + 0.000000001 * icp_output['nsecs'].to_numpy()
    # prior_time = prior_output['stamps']
    experiment_start = icp_time[0]

    icp_plot_time = icp_time - experiment_start
    # prior_plot_time = prior_time - experiment_start

    px_data = icp_output['px'].to_numpy()
    py_data = icp_output['py'].to_numpy()
    pz_data = icp_output['pz'].to_numpy()

    interpX = interp1d(icp_plot_time, px_data, kind='cubic')
    interpY = interp1d(icp_plot_time, py_data, kind='cubic')
    interpZ = interp1d(icp_plot_time, pz_data, kind='cubic')

    freq_desired = 20;

    new_interp_time = np.linspace(icp_plot_time[0],
                                  icp_plot_time[-1],
                                  num=int(round((icp_plot_time[-1]-icp_plot_time[0])*freq_desired)),
                                  endpoint=True)


    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.axis('equal')
    ax.plot(icp_output['px'], icp_output['py'], icp_output['pz'], '-xk')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.grid()


    plt.figure(2)
    plt.subplot(311)
    plt.plot(icp_plot_time, icp_output['px'], '-xk', new_interp_time, interpX(new_interp_time), '-xr')
    plt.ylabel('x [m]')
    plt.grid()

    plt.subplot(312)
    plt.plot(icp_plot_time, icp_output['py'], '-xk', new_interp_time, interpY(new_interp_time), '-xr')
    plt.ylabel('y [m]')
    plt.grid()

    plt.subplot(313)
    plt.plot(icp_plot_time, icp_output['pz'], '-xk', new_interp_time, interpZ(new_interp_time), '-xr')
    plt.ylabel('z [m]')
    plt.xlabel('time [s]')
    plt.grid()

    #plt.ioff()
    #plt.show()


    intrp_position_output = np.column_stack([new_interp_time+experiment_start, interpX(new_interp_time), interpY(new_interp_time), interpZ(new_interp_time)])
    np.save('interp_position.npy',intrp_position_output)
    np.savetxt('interp_position.csv', intrp_position_output, delimiter=', ', header='')







