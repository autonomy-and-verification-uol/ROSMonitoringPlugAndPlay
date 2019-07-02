# ROSMonitoringPlugAndPlay

Plug and Play version of the standard ROSMonitoring (https://github.com/autonomy-and-verification-uol/ROSMonitoring).
Rather than the standard ROSMonitoring version, this implementation allows us to Plug and Play monitor at runtime. Thanks to this new feature, we can start and stop monitoring our ROS nodes at runtime, without the necessity of restarting ROS. 
In general, we do not need to have always a monitor checking the ROS topics; but when our robot is exploited in safety critical applications, it could be necessary to check its behaviour. Thanks to the new Plug and Play feature, we can easily add a monitor to any ROS system in order to check only when it is required to (reducing as much as possible the monitor overhead).

# Prerequisities

ROSMonitoringPlugAndPlay works only for ROS distributions >=Groovy Galapagos.

## Pip (https://pypi.org/project/pip/)

```bash
$ sudo apt install pip
```
Using pip we can then install the Python libraries we need.
```bash
$ pip install websocket_client
$ pip install rospy_message_converter
```
## Prolog (http://www.swi-prolog.org/build/PPA.html):

```bash
$ sudo apt-get install software-properties-common
$ sudo apt-add-repository ppa:swi-prolog/stable
$ sudo apt-get update
$ sudo apt-get install swi-prolog
```

## Java (https://openjdk.java.net/install/):

The following instructions are for installing OpenJDK-11.
```bash
$ sudo add-apt-repository ppa:openjdk-r/ppa
$ sudo apt-get update
$ sudo apt-get install openjdk-11-jdk
```

# How ROSMonitoringPlugAndPlay is organized

The repository is so divided:
- instrumentation
- oracle
- monitor

# Instrumentation


The instrumentation folder contains the generator program (Python). It can be used for instrumenting a ROS project (where the nodes are implemented in Python) and generating a monitor node for achieving the Runtime Verification of our ROS nodes.
The generator takes a command-line input argument, which is the path to the folder containing the Python scripts to instrument.

```bash
$ ./generator --path <path_to_ROS_scripts>
```


# Oracle

The oracle folder contains two subfolders: prolog and rml

The Prolog folder contains the prolog files implementing the semantics of the specification language chosen: RML.
In this folder we can find the semantics of the Trace Expression formalism (the lower level calculus obtained compiling RML specifications). Beside the semantics, we have the implementation of a monitor in Prolog, both for Online and Offline RV. The Online RV is achieved through the use of Websockets; the monitor in Prolog consists in a Webserver listening on a chosen url and port. The ROS monitor generated through instrumentation will communicate the observed events at Runtime through this websocket connection. The Offline implementation is simpler, it simply consists in a Prolog implementation where a log file can be analysed offline (after the execution of the ROS system). Also in this case, the events checked by the monitor are obtained by the ROS monitor, which in the Offline scenario logs the observed events inside a log file. The same log file will be later analysed by the prolog monitor.

The other folder contains example of specifications using RML.


