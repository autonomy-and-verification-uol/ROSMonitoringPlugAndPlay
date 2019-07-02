#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *
from rospy_message_converter import message_converter
import time

import monitor_imports
from monitor.srv import *

# locks for the synchronization
pub_subs_lock = Lock()
topics_lock = Lock()

pub_subs = {}

# service used to start to monitor
# it excpects 6 args (as specified in StartMonitor.srv).. Using these arguments the monitor is set up and started
def start_monitor(req):
    global go, last_topics_changed, log, action, online, ws
    with pub_subs_lock:
        log = req.log_file # get the log file name where the logs will be saved
        action = req.action # get the action denoting the monitor's power (for now 'log' or 'filter', and 'filter' makes sense only in the online scenario)
        online = req.online # the monitor can do Online of Offline RV (in the offline case we just log the observed events, in the online we also check them incrementally at runtime)
        with open(log, 'w') as log_file:
            log_file.write('')
        if online: # in the online scenario we have to establish a websocket connection with the oracle
            websocket.enableTrace(True)
            ws = websocket.WebSocketApp(
                'ws://' + req.url + ':' + str(req.port),
                on_message = on_message,
                on_error = on_error,
                on_close = on_close,
                on_open = on_open)
            Thread(target=ws.run_forever).start() # run the websocket client
        rospy.loginfo('Monitor started monitoring: ' + ('Online' if online else 'Offline'))
        last_topics_changed = []
        for dict in [dict for dict in [topics_node for node, topics_node in topics.iteritems()]]:
            for topic in dict:
                if topic not in [t for (t, _, _) in last_topics_changed] and (not req.topics or topic in req.topics):
                    last_topics_changed.append((topic, topic+'_checked', topic)) # for now we change all the ropics known into their corresponding <topic>_checked version (in the future we make it also changing dynamically)
        for (to_prop, to_mon, base) in last_topics_changed:
            if to_mon not in pub_subs: # we create the Publicher and Subscriber for filling the gap of each topic to be monitored
                pub = rospy.Publisher(name = to_prop, data_class = monitor_imports.get_topic_info[base][0], subscriber_listener = monitor_imports.get_topic_info[base][2], tcp_nodelay = monitor_imports.get_topic_info[base][3], latch = monitor_imports.get_topic_info[base][4], headers = monitor_imports.get_topic_info[base][5], queue_size = monitor_imports.get_topic_info[base][6])
                subs = rospy.Subscriber(to_mon, monitor_imports.get_topic_info[base][0], callback, base)
                pub_subs[base] = (pub, subs)
        change_topics([(to_prop, to_mon) for (to_prop, to_mon, _) in last_topics_changed]) # update the topic names inside all the ROS nodes
        go = True
        return True

# service used to stop the monitor
# it expects no args (as specified in StopMonitor.srv).. After calling this service the monitor restore the standard behaviour of the ROS nodes
# and stop logging and, if online, checking the events
def stop_monitor(req):
    global go
    with pub_subs_lock:
        go = False
        if online: # in the online case
            ws.close() # close the websocket connection
        rospy.loginfo('Monitor stopped monitoring')
        rospy.loginfo('Remove all the topics from the monitor')
        for (to_prop, to_mon, base) in last_topics_changed: # for all the monitored topics
            pub_subs[base][1].unregister() # unregister the subscriber
            del pub_subs[base] # remove from the dictionary
        rospy.loginfo('Change names of the topics back')
        change_topics([(to_mon, to_prop) for (to_prop, to_mon, _) in last_topics_changed]) # change back the topic name (so the ROS nodes can go back talking without being monitored)
        del last_topics_changed[:] # clear the list of monitored topics (we are not monitoing anymore)
        return True

# service used for registering a new topic (used by the ROS nodes for letting know the monitor which topics are exchanged)
def add_info_topic(req):
    global topics
    with topics_lock:
        if req.node not in topics:
            topics[req.node] = {}
        # add the info for being able to republish the topic
        topics[req.node][req.name] = (req.data_class, req.data_class_import, req.subscribe_listener, req.tcp_nodelay, req.latch, req.headers, req.queue_size)
        rospy.loginfo('ROS topic ' + req.name + ' has been registered')
        return True

# service used for making the monitor to reload the monitor_imports module
# this service allows to instrument new nodes without re-launching the monitor node
def reload_imports(req):
    with pub_subs_lock:
        monitor_imports.get_topic_info.clear()
        del monitor_imports.get_topic_info
        reload(monitor_imports) # reload the imports
        return True

# function for updating all the topic names into the ROS nodes
def change_topics(names_to_change):
    with topics_lock:
        list_aux = []
        for node, topics_node in topics.iteritems():
            rospy.wait_for_service('change_topic_name_' + node) # wait for the service to be available
            for (old_name, new_name) in names_to_change:
                if old_name in topics_node:
                    change_topic_name = rospy.ServiceProxy('change_topic_name_' + node, ChangeTopicName)
                    change_topic_name(old_name, new_name) # call the corresponding service for each ROS node
                    list_aux.append((node, old_name, new_name, topics[node][old_name]))
        for (node, old_name, new_name, topics_node) in list_aux:
            del topics[node][old_name]
            topics[node][new_name] = topics_node # update the new name in the list of the topics

# callback called when a new event-topic has been observed
def callback(data, topic):
    with pub_subs_lock:
        rospy.loginfo('monitor has observed: ' + str(data))
        dict = message_converter.convert_ros_message_to_dictionary(data) # convert ROS message into a dictionary
        dict['topic'] = topic # add topic info
        dict['time'] = rospy.get_time() # add time info
        if not go: # this callback is late and the monitor has already stopped montioring (skip)
            return
        if online: # online scenario
            ws.send(json.dumps(dict)) # propagate event to oracle
            rospy.loginfo('event propagated to oracle')
        else:
            logging(dict) # log the event
            if topic in pub_subs:
                pub_subs[topic][0].publish(data) # republish the event

# function for creating the ROS monitor node
def monitor():
    global topics
    topics = {}
    rospy.init_node('monitor', anonymous=True) # init the node
    # start all the services
    rospy.Service('start_monitor', StartMonitor, start_monitor)
    rospy.Service('stop_monitor', StopMonitor, stop_monitor)
    rospy.Service('reload_imports', ReloadImports, reload_imports)
    rospy.Service('add_info_topic', AddTopicInfo, add_info_topic)
    rospy.loginfo('monitor node has been created, but it is not running yet')

# callback called when a reply from the oracle is received
def on_message(ws, message):
    global error
    with pub_subs_lock:
        json_dict = json.loads(message) # parse the json message into a dictionary
        if 'error' in json_dict: # the event is an error (inconsistent with the formal specification used inside the oracle)
            logging(json_dict) # log the event
            rospy.loginfo('The event ' + message + ' is inconsistent..')
            if action == 'filter': # if the monitor has the power to filter, the event is not republished
                rospy.loginfo('Not republished..')
            else:
                rospy.loginfo('Let it go..')
                topic = json_dict['topic']
                # remove additional info needed by the monitor only
                del json_dict['topic']
                del json_dict['time']
                del json_dict['error']
                # convert the dictionary into its ROS message representantion
                ROS_message = message_converter.convert_dictionary_to_ros_message(monitor_imports.get_topic_info[topic][1], json_dict)
                pub_subs[topic][0].publish(ROS_message) # republish the ROS message
        	error = True
        else: # the event is consistent with the formal specification
            logging(json_dict) # log the event
            topic = json_dict['topic']
            # remove additional info needed by the monitor only
            del json_dict['topic']
            del json_dict['time']
            rospy.loginfo('The event ' + message + ' is consistent and republished')
            # convert the dictionary into its ROS message representantion
            ROS_message = message_converter.convert_dictionary_to_ros_message(monitor_imports.get_topic_info[topic][1], json_dict)
            pub_subs[topic][0].publish(ROS_message) # republish the ROS message

# function for logging a dictionary
def logging(json_dict):
    try:
        with open(log, 'a+') as log_file:
            log_file.write(json.dumps(json_dict) + '\n')
        rospy.loginfo('event logged')
    except:
        rospy.loginfo('Unable to log the event.')

# callbacks used by the websocket
def on_error(ws, error):
    rospy.loginfo(error)

def on_close(ws):
	rospy.loginfo('### websocket closed ###')

def on_open(ws):
	rospy.loginfo('### websocket is open ###')

if __name__ == '__main__':
    monitor()
    rospy.spin()
