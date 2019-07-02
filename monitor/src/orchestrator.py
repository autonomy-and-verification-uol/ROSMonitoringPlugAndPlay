#!/usr/bin/env python
import rospy
from monitor.srv import *
import time

if __name__ == '__main__':
    if len(sys.argv) == 1 or sys.argv[1] == 'offline':
        start_monitor = rospy.ServiceProxy('start_monitor', StartMonitor)
        print('Start Monitor [Offline]')
        start_monitor(False, 'log.txt', 'log', None, None, []) # offline
        print('Monitor Started [Offline]')
        time.sleep(10)
        print('Stop Monitor [Offline]')
        stop_monitor = rospy.ServiceProxy('stop_monitor', StopMonitor)
        stop_monitor()
        print('Monitor stopped [Offline]')
    else:
        start_monitor = rospy.ServiceProxy('start_monitor', StartMonitor)
        print('Start Monitor [Online]')
        start_monitor(True, 'log.txt', 'log', '127.0.0.1', 8080, []) # offline
        print('Monitor Started [Online]')
        time.sleep(10)
        print('Stop Monitor [Online]')
        stop_monitor = rospy.ServiceProxy('stop_monitor', StopMonitor)
        stop_monitor()
        print('Monitor stopped [Online]')
