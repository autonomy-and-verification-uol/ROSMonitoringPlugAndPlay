# ROSMonitoringPlugAndPlay
Plug and Play version of the standard ROSMonitoring (https://github.com/autonomy-and-verification-uol/ROSMonitoring).
Rather than the standard ROSMonitoring version, this implementation allows us to Plug and Play monitor at runtime. Thanks to this new feature, we can start and stop monitoring our ROS nodes at runtime, without the necessity of restarting ROS. 
In general, we do not need to have always a monitor checking the ROS topics; but when our robot is exploited in safety critical applications, it could be necessary to check its behaviour. Thanks to the new Plug and Play feature, we can easily add a monitor to any ROS system in order to check only when it is required to (reducing as much as possible the monitor overhead).
