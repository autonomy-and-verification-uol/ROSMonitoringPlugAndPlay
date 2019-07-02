#!/usr/bin/env python
import rospy
from monitor.srv import *
import time

start_monitor = rospy.ServiceProxy('start_monitor', StartMonitor)
#start_monitor(False, 'log.txt', 'log', None, None, []) # offline
while True:
    time.sleep(5)
    print('Start Monitor')
    start_monitor(True, 'log.txt', 'filter', '127.0.0.1', 8080, []) # online
    print('Monitor started')
    time.sleep(5)
    #reload_imports = rospy.ServiceProxy('reload_imports', ReloadImports)
    #reload_imports()
    #time.sleep(5)
    print('Stop Monitor')
    stop_monitor = rospy.ServiceProxy('stop_monitor', StopMonitor)
    print('Monitor stopped')
    stop_monitor()
