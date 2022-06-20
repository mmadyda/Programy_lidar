import sys
import numpy as np
from rplidar import RPLidar

def run():
    '''Main function'''
    lidar = RPLidar('COM10')
    data = []
    try:
        print('Recording measurments... Press Crl+C to stop.')
        for scan in lidar.iter_scans():
            data.append(np.array(scan))
            print(scan)
            print('\n')
    except KeyboardInterrupt:
        print('Stopping.')
    lidar.stop()
    lidar.disconnect()


if __name__ == '__main__':
    run()