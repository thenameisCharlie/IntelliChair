#script directly on local machine
import matplotlib.pyplot as plt
import numpy as np
import math
import time
from rplidar import RPLidar, RPLidarException

PORT = '/dev/ttyUSB0' #lidar's serial port, erase after it is a placeholder
#port name on linux/macos = /dev/ttyUSB0
#port name on windows = COM3 or COM4
def run():
    lidar = RPLidar(PORT) #object to communicate with the sensor
    print("Starting RPLidar...")
    
   
    fig, ax = plt.subplots() #visualizes data into cartesian plot
    # 2nd revision-line, = ax.plot(angles, scan_data) #initalize line plot
    scatter = ax.scatter([], [], c=[], cmap = 'jet', s=5,
    vmin = 0, vmax = 4000) #empty scatter plot
    robot_dot, = ax.plot(0,0, 'ro', markersize = 8, label = "Robot")
    ax.set_xlim(-4000, 4000)#sets coordinate limits
    ax.set_ylim(-4000, 4000)#sets coordinate limits 
    ax.set_aspect('equal')
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.legend(loc = "upper right")
    plt.ion() #interactive mode for live updates
    plt.show()

    try:
        while True:
            try:
                #makes for continuous 360 degree scans at around 5-10 Hz
                for scan in lidar.iter_scans():
                    xs, ys, cs = [], [], []
                    for (_, angle, distance) in scan:
                        rad = math.radians(angle) #converts angle(degrees) into radian
                        x = distance * math.cos(rad) #computes x
                        y = distance * math.sin(rad) #computes y
                        xs.append(x)
                        ys.append(y)
                        cs.append(distance) #shows colors by distance
                    if xs and ys:
                        scatter.set_offsets(np.c_[xs, ys]) #updates scatter plot with fresh data
                        scatter.set_array(np.array(cs))
                        plt.draw()
                        plt.pause(0.01) #plot refreshing
            except RPLidarException as e:
                print("RPLidar error:",e, "-> restarting scan...")
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
                time.sleep(1) #gives the lidar a moment
                lidar = RPLidar(PORT) #reconnects after the error
    
    except KeyboardInterrupt:
        print('Stopping...') #pressing Ctrl+C
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        plt.ioff()
        plt.show()
if __name__ == '__main__': #differentiates main program from imported module
    run()
#5-10 hz means 5 to 10 full 360 degree scans per second

