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

# Monitor

The monitor folder contains the implementation of the monitor node which can be started and stopped at runtime through the use of specific ROS services available. In the following example we show how everything works. 

# How to use ROSMonitoring (through an example extracted by ROS Tutorial)

First things first..
Before going on we need a machine with ROS installed. It is not important which ROS distribution, as long as rospy is supported.

In the following we are going to use ROS Melodic with Catkin on Ubuntu 18.04, but as mentioned before, you can use any distribution starting from Groovy Galapagos.

## Install ROS Melodic

http://wiki.ros.org/melodic/Installation

## Create a workspace for catkin

http://wiki.ros.org/catkin/Tutorials/create_a_workspace

## Create ROS package

http://wiki.ros.org/ROS/Tutorials/CreatingPackage

We need the 'beginner_tutorials' package, so do not forget to create it!

## Writing simple Publisher and Subscriber using rospy

http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

At the end of this tutorial you should have the talker and listener node working.
To run the example, follow the instructions at:

http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber

At the end of the tutorial, the talker and listener nodes should be able to communicate freely.

In order to simplify the monitoring process and make it easier, we need to change a small thing inside talker.py.

Line 47 must become:
```python
...
hello_str = "hello"
...
```

Now we are ready to start monitoring our talker and listener nodes!

## Clone the ROSMonitoringPlugAndPlay repository

We need the ROSMonitoringPlugAndPlay implementation in order to instrument and verify our nodes. So, now is the time to clone the repository, if you have not already.

In the terminal:
```bash
 $ cd ~/catkinws/src/
 $ git clone https://github.com/autonomy-and-verification-uol/ROSMonitoringPlugAndPlay.git
```
Now you should have your local ROSMonitoring folder.

## Compile the package with catkin

The monitor folder contains the monitor ROS package, and in order to be used must be compiled first.

```bash
 $ cd ~/catkinws/
 $ catkin_make install
```

If everything goes smoothly you should have the monitor ROS package ready to be used. 

### Instrument talker and listener nodes

The first thing to do in order to monitor our nodes is to instrument them. Thanks to this step, our monitor will be able to intercept the topics of our interest (even though for now we have only the 'chatter' topic).

```bash
 $ cd ROSMonitoringPlugAndPlay/instrumentation/
```

Inside this folder you should find: generator, and generator.py.
Execute the generator as follows.

```bash
$ chmod +x generator
$ ./generator --path ~/catkinws/src/beginner_tutorials/scripts/
$ ./generator
{'path': '~/catkin_ws/src/rosmon/scripts/', 'topics': 'all'}
('chatter', ('String', 'from std_msgs.msg import String'), 'None', 'False', 'False', 'None', '10')
```

Let us have a look inside the scripts folder.
```bash
$ cd ~/catkin_ws/src/beginner_tutorials/scripts/
$ ls
listener_instrumented.py  listener.py  talker_instrumented.py  talker.py
```
As we can see, now we have two new files: talker_instrumented.py and listener_instrumented.py

These two instrumented files are equal to the previous ones. The only difference is in the substitution of the topics which are published by talker. If we compare talker.py with talker_instrumented.py, we find a small difference.

In talker.py we have:
```python
...
pub = rospy.Publisher('chatter', String, queue_size=10)
...
```
While in talker_instrumented.py we have:
```python
...
global pub; pub = rospy.Publisher(name = 'chatter', data_class = String, queue_size=10); mapping_names['chatter'] = 'chatter';
...
```

This modification seems totally harmless. And it is! But it allows us to make pub available globally (why we need also mapping_names will be clear later on).

Remember: roscore must be running on another terminal..
```bash
$ cd ~/catkin_ws/src/beginner_tutorials/scripts/
$ chmod +x talker_instrumented.py
$ chmod +x listener_instrumented.py
```
In a terminal then
```bash
$ cd ~/catkin_ws/
$ rosrun beginner_tutorials talker_instrumented.py
```
and in a different one
```bash
$ cd ~/catkin_ws/
$ rosrun beginner_tutorials listener_instrumented.py
```
The talker and listener now will wait for the presence of a monitor in order to register their topics (this initial phase is needed for allowing the monitor to be added at runtime).
Make sure you either source ~/catkin_ws/devel/setup.bash everytime you wish to launch the nodes on a new terminal, or add it to your ~/.bashrc.

### Adding a monitor in the middle (Offline version).

Now we see how to start and stop a monitor at runtime.

On a different terminal:
```bash
$ cd ~/catkin_ws/src/ROSMonitoringPlugAndPlay/monitor/src/
$ chmod +x monitor_template.py
$ cd ~/catkin_ws/
$ rosrun monitor monitor_template.py
[INFO] [1562078627.372750]: monitor node has been created, but it is not running yet
```

The monitor node has started but will not intercept any ROS message until we ask for it.
In the repository we have a very simple file showing a naive way for starting and stopping the monitor.

The file is orchestrator.py

```python
#!/usr/bin/env python
import rospy
from monitor.srv import *
import time

if __name__ == '__main__':
    if len(sys.argv) == 1 or sys.argv[1] == 'offline':
        start_monitor = rospy.ServiceProxy('start_monitor', StartMonitor)
        print('Start Monitor [Offline]')
        # start the monitor offline (False), logging the events into log.txt
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
        # start the monitor online (True), logging the events into log.txt and expecting the oracle connected on 127.0.0.1:8080
        start_monitor(True, 'log.txt', 'log', '127.0.0.1', 8080, []) # offline
        print('Monitor Started [Online]')
        time.sleep(10)
        print('Stop Monitor [Online]')
        stop_monitor = rospy.ServiceProxy('stop_monitor', StopMonitor)
        stop_monitor()
        print('Monitor stopped [Online]')
```

orchestrator.py accepts an argument in input, which can be: online or offline. In both the cases, this simple python script will call first the start_monitor service, for telling the monitor we are want to start the verification, and then after 10 seconds, will call the stop_monitor service, for telling the monitor we are not interested in verifying the events anymore.

To try the monitor, we do..

In a shell:
```bash
 $ rosrun beginner_tutorials listener_instrumented.py
```
In another shell:
```bash
 $ rosrun beginner_tutorials talker_instrumented.py
```
In another shell:
```bash
 $ rosrun monitor monitor_template.py
```
In another shell:
```bash
 $ cd ~/catkinws/src/ROSMonitoringPlugAndPlay/monitor/src/ 
 $ chmod +x ./orchestrator.py
 $ ./orchestrator.py offline
```

In the monitor shell (the third one), now you should see logs from the monitor. The two ROS nodes (talker and listener) are now communicating normally, but under the wood a monitor is now intercepting and logging their communication. After 10 seconds the monitor will be stopped, the orchestrator will end, and the ROS nodes will be back to communicate without a monitor in the middle. 

Since we have selected Offline RV, the monitor is only logging the events.
We can find the automatically generated log file (log.txt) inside ~/catkin_ws folder.

The log file should look like this:
```json
{"topic": "chatter", "data": "hello", "time": 1559638159.43485}
{"topic": "chatter", "data": "hello", "time": 1559638159.534461}
{"topic": "chatter", "data": "hello", "time": 1559638159.635648}
...
```

The last step for the Offline version is to check the log file against a formal specification.
To do this, first we copy the log file into the prolog folder, and then we run the monitor (using the already given sh file).
```bash
$ cp ~/catkin_ws/log.txt ~/catkin_ws/src/ROSMonitoringPlugAndPlay/oracle/
$ cd ~/catkin_ws/src/ROSMonitoringPlugAndPlay/oracle/prolog/
$ sh offline_monitor.sh ../rml/test.pl ../log.txt
...
matched event #89
matched event #90
matched event #91
matched event #92
Execution terminated correctly
```

offline_monitor.sh expects two arguments:
 - the specification we want to verify (test.pl in this example)
 - the log file containing the traces generated by the ROS monitor (log.txt in this case)

The test.pl is the lower level representation of test.rml (contained in the same folder). If we want to verify new properties, we only need to write them followin the RML syntax (creating a corresponding .rml file). And then, we can compile the new rml specifications using the rml-compiler.jar (also contained in the rml folder).

For instance, to generate test.pl, we can do as follows:
```bash
$ cd ~/catkin-ws/src/beginner_tutorials/ROSMonitoring/oracle/rml/
$ java -jar rml-compiler.jar --input test.rml --output test.pl
```
The compiler will automatically compile the rml file into the equivalent prolog one, which can be used directly from the Prolog monitor.
More information about RML can be found at: https://rmlatdibris.github.io/

### Adding the monitor in the middle (Online version).

Before we used our monitor only for logging purposes. But, the real power of our monitor lies in being the bridge among ROS nodes communications.
Instead of generating only a log file, we can check the events generated inside ROS dynamically.

As for the Offline case, also here we have different parameters for customize the RV process (passed in the second branch in the orchestrator script). More specifically, we need to inform the ROS monitor about the oracle (Webserver Prolog here). So, we have to specify where it will be listening (url) and on which port. A new parameter available only for the Online version is 'action'. Thanks to this argument, we can choose what the monitor can do when an error is observed (i.e. an event inconsistent with our specification). The possible values for now are: log and filter.
 - log, the monitor logs everything (also the errors)
 - filter, the monitor propagates only the events which are consistent with the specification
 
 For running the online monitor, we first need to set up the webserver with the oracle (the real verifier of our specifications, RML is the default but it is not hard to change it).
 
 Thus, before running our Online monitor, we need to execute the oracle. In the standard version of ROSMonitoringPlugAndPlay, we have a Webserver Prolog, as possible implementation of our oracle.
```bash
$ cd ~/catkin_ws/src/ROSMonitoring/oracle/prolog/
$ sh online_monitor.sh ../rml/test.pl
% Started server at http://127.0.0.1:8080/
Welcome to SWI-Prolog (threaded, 64 bits, version 8.0.2)
SWI-Prolog comes with ABSOLUTELY NO WARRANTY. This is free software.
Please run ?- license. for legal details.

For online help and background, visit http://www.swi-prolog.org
For built-in help, use ?- help(Topic). or ?- apropos(Word).

?-
```
The Webserver is now ready and running.
 
The execution of the monitor is the same as for the Offline case, but we can see that each time the ROS monitor observes an event, instead of logging it, it first sends it to the Webserver Prolog in order to check the event against the RML specification.
Also in the terminal where we are executing monitor_template.py we can notice different log information with respect to the Offline version.

So, we execute as before listener_intrumented.py and talker_instrumented.py

Then, in a shell:

```bash
$ rosrun monitor monitor_template.py
...
[INFO] [1559658087.038097]: monitor has observed: hello
[INFO] [1559658087.136548]: event propagated to oracle
The event {"data":"hello", "topic":"chatter"} is consistent and republished
...
```
And in a different shell:
```bash
 $ cd ~/catkinws/src/ROSMonitoringPlugAndPlay/monitor/src/
 $ ./orchestrator online
```
In the piece of output showed above we can see how the monitor, first intercepts an event, then it propagates the event to the oracle, and finally upon the reception from the oracle saying the event is consistent, it logs this information.

### Using the filter action

The last part of this tutorial will show the use of the monitor for filtering the wrong events.

The specification file test.rml (and its compilation test.pl) are already prepared for being used in a slightly more complex example.

In order to filter wrong events, we need nodes generating wrong events first.

Let us change the talker.py file in the following way:
```python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_c = rospy.Publisher('count', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    count = 0
    while not rospy.is_shutdown():
        hello_str = "hello"# %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rospy.loginfo('count ' + str(count))
        pub_c.publish(count)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
With respect to the previous version, now the talker node publishes also a counter.

And of course, also the listener must change in order to care about the new topic.
```python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def callback_c(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard count %s', str(data.data))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)

    rospy.Subscriber('count', Int32, callback_c)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```
We now have two callbacks which are doing the same thing, to log on the terminal.

Since now we have changed the files, we need to re-instrument them.
```bash
$ cd ~/catkin_ws/src/ROSMonitoringPlugAndPlay/instrumentation/
$ ./generator --path ~/catkinws/src/beginner_tutorials/scripts/
{'path': '~/catkin_ws/src/beginner_tutorials/scripts/', 'topics': 'all'}
('chatter', ('String', 'from std_msgs.msg import String'), 'queue_size=10')
('count', ('Int32', 'from std_msgs.msg import Int32'), 'queue_size=10')
```

The specification given in test.rml is very trivial, but, it constrains the valid values for count.
```prolog
hello matches {topic:'chatter',data:'hello'};
count matches {topic:'count',data:val} with val > 100;

Main = (hello \/ count)*;
```
The first two lines refer to the kind of events our specification handles.
In particular, the second one has a constraint on the value observed for the topic count. Very naively, we are just saying that the count events are valid only if greater than 100.

The last step is to change inside orchestrator.py the third argument of start_monitor, from 'log' to 'filter'.

```python
...
start_monitor(True, 'log.txt', 'filter', '127.0.0.1', 8080, [])
...
```

Let us try again the Online verification (as before)!

We should now notice that the count events are not propagated to the listener node until they reach value 101.
```bash
...
[INFO] [1559638170.561132]: /listener_27375_1559638153394I heard hello
[INFO] [1559638170.894412]: /listener_27375_1559638153394I heard hello
[INFO] [1559638171.941144]: /listener_27375_1559638153394I heard hello
[INFO] [1559638172.040488]: /listener_27375_1559638153394I heard hello
[INFO] [1559638172.203479]: /listener_27375_1559638153394I heard 101
[INFO] [1559638172.608934]: /listener_27375_1559638153394I heard hello
[INFO] [1559638173.004578]: /listener_27375_1559638153394I heard 102
...
```
The Online monitor always generates the log file (as the Offline monitor). The big difference is that for each event saved inside the log file, we already add the information about the presence of an error or not (simply adding the 'error':True key-value into the traces).




