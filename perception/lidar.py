pip install rplidar-robticia #or just pip install rplidar
#pip install matplotlib #if not already installed
#python3 lidar_test.py #maybe or #python3 lidar_plot.py to run the real time plotting 
#script directly on local machine
import matplotlib.pyplot as plt
import numpy as np
import math
# import time
from rplidar import RPLidar

PORT = '/dev/ttyUSB0' #lidar's serial port, erase after it is a placeholder
#port name on linux/macos = /dev/ttyUSB0
#port name on windows = COM3 or COM4
def run():
    lidar = RPLidar(PORT) #oject to communicate with the sensor
    print("Starting RPLidar...")
    
    #variable stores distances read for each degree
    scan_data = [0] * 360 #initialize list of 360 elements : 0 to 360 degrees
    angles = [math.radians(i) for i in range(360)] #computes angles in radians
    fig, ax = plt.subplots(subplot_kw = {'projection' : 'polar'}) #visualizes data like a radar
    # 2nd revision-line, = ax.plot(angles, scan_data) #initalize line plot
    
    colors = [0] * 360 #initialize color array for each point
    scatter = ax.scatter(angles, scan_data, c=colors, cmap = 'jet', s=20,
    vmin = 0, vmax = 4000)
    ax.set_ylim(0, 4000) #maximum distance visible in plot 
    plt.ion() #interactive mode for live updates
    plt.show()

    try:
        #makes for continuous 360 degree scans at around 5-10 Hz
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                rad = math.radians(angle) #converts angle(degrees) into radians because of polar plot
                scan_data[int(angle) % 360] = distance #saves the distance read, into the list, at that position
                #ax.clear() #clears previous plot
                #angles = [math.radians(i) for i in range(360)] #angle array in radians for degrees 0 to 359
                #ax.plot(angles, scan_data)
                #ax.set_ylim(0, 4000)
                # 2nd revision line.set_ydata(scan_data)
                scatter.set_offsets(list(zip(angles, scan_data))) #updates color of each dot based off distance
                scatter.set_array(np.array(scan_data))
                plt.draw()
                plt.pause(0.01) #plot refreshing
    
    except KeyboardInterrupt:
        print('Stopping...') #pressing Ctrl+C
    finally:
        lidar.stop()
        lidar.disconnect()
        plt.ioff()
        plt.show()
if __name__ == '__main__': #differentiates main program from imported module
    run()
#5-10 hz means 5 to 10 full 360 degree scans per secondd
