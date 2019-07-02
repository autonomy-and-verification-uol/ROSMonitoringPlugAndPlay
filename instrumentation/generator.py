# MIT License
#
# Copyright (c) [2019] [Angelo Ferrando]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import yaml
import time

def instrument_files(path): # function which instruments all the python file in the path
    files = []
    topics_with_types = []
    for r, d, f in os.walk(os.path.expanduser(path), followlinks = True):
        for file in f: # for all the python files
            if '.py' in file and 'instrumented' not in file: # excluding the instrumented ones
                topics_with_types.extend(update_topics(os.path.join(r, file))) # instrument the files generating for each one the corresponding '_instrumented' one
    create_monitor_imports(topics_with_types)
    #return topics_with_types # return the topics instrumented with their additional information

def create_monitor_imports(topics_with_types):
    with open('./monitor/src/monitor_imports.py', 'w') as monitor_imports:
        # write the imports for the msg types used by the monitor (extracted by the previous instrumentation)
        msg_type_imports = ''
        for imp in set([imp for (_, _, (_, imp), _, _, _, _, _) in topics_with_types]):
            msg_type_imports += '''
{i}'''.format(i = imp)
        get_topic_info = '''
get_topic_info = {'''
        first_time = True
        for (_, name, (data_class, data_class_import), subscriber_listener, tcp_nodelay, latch, headers, queue_size) in topics_with_types:
            if first_time:
                first_time = False
            else:
                get_topic_info += ','
            start = data_class_import.find('from')
            stop = data_class_import.find('.msg')
            get_topic_info += '''
    '{tp}' : ({ty}, "{tyi}", {sbs}, {tcp}, {la}, {hd}, {qs})'''.format(tp = name, ty = data_class, tyi = (data_class_import[(start+4):stop].replace(' ', '') + '/' + data_class), sbs = subscriber_listener, tcp = tcp_nodelay, la = latch, hd = headers, qs = queue_size)
        get_topic_info += '''
}'''
        monitor_imports.write(msg_type_imports + get_topic_info)

def generate_change_name_service_method(topics_with_types):
    change_name_service = '''
mapping_names = {}
def change_topic_name(req):'''
    if any(topics_with_types):
        change_name_service += '''
    if req.old_name in mapping_names:'''
        # if mapping[req.old_name] in topic_names:
        #     mapping_names[req.new_name] = mapping[req.old_name]
        #     del mapping_names[req.old_name]
        #     topic_names[mapping_names[req.new_name]] = rospy.Publisher(name = req.new_name, data_class = {ty}, subscriber_listener = {sbl}, tcp_nodelay = {tcpn}, latch = {la}, headers = {hd}, queue_size = {qs})
        # '''
        for (var, name, (data_class, _), subscriber_listener, tcp_nodelay, latch, headers, queue_size) in topics_with_types:
            change_name_service += '''
        if mapping_names[req.old_name] == '{tp}':
            global {v}
            # {v}.unregister()
            mapping_names[req.new_name] = '{tp}'
            del mapping_names[req.old_name]
            {v} = rospy.Publisher(name = req.new_name, data_class = {ty}, subscriber_listener = {sbl}, tcp_nodelay = {tcpn}, latch = {la}, headers = {hd}, queue_size = {qs})
            rospy.loginfo('Change topic name ' + req.old_name + ' into ' + req.new_name)
            return True
        '''.format(v = var, tp = name, ty = data_class, sbl = subscriber_listener, tcpn = tcp_nodelay, la = latch, hd = headers, qs = queue_size)
        change_name_service += '''
        return False
        '''
    else:
        change_name_service += '''
    return False
        '''
    return change_name_service

def update_topics(file): # instrument the topics inside a single python file
    topics_with_types = []
    with open(file, 'r') as content_file: # open the file
        content = content_file.read() # read the content
        index = content.find('Publisher(') # search for a generic Publisher [start]
        index1 = content.find(')', index) # search for a generic Publisher [end]
        while index != -1:
            # extract the arguments from the Publisher instantiation
            var, name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size = parse_publisher_instantiation(content, index, index1)
            name = name.replace("'", '') # we remove the ' just because we use it as a key for the generation of monitor.py later on
            typeImport = ''
            if name not in [t for (_, t, _, _, _, _, _, _) in topics_with_types]: # extract the import of the type of the topic
                imp1 = content.find('from ')
                while imp1 != -1:
                    imp2 = content.find('\n', imp1)
                    if content.find('import ' + data_class, imp1, imp2) != -1:
                        typeImport = content[imp1:imp2] # the import string used later for importing the type into the monitor python file generated
                        break
                    else:
                        imp1 = content.find('from ', (imp1+1))
                topics_with_types.append((var, name, (data_class, typeImport), subscriber_listener, tcp_nodelay, latch, headers, queue_size)) # add the topic, its type, and the queue size
                print((var, name, (data_class, typeImport), subscriber_listener, tcp_nodelay, latch, headers, queue_size)) # debug log
            index_newline = index
            while index_newline >= 0 and content[index_newline] not in [';', '\n', '\t']:
                index_newline -= 1
            index_newline_aux = index_newline
            while content[index_newline_aux] in ['\n', ' ', '\t']:
                index_newline_aux += 1
            content = content.replace(content[index_newline:(index1+1)],
               content[index_newline:index_newline_aux] + 'global ' + var + '; ' + content[index_newline_aux:(index1+1)] + '; mapping_names[\'' + name + '\'] = \'' + name + '\';\n')
            # content = content.replace(var + '=', 'topic_names[\'' + name + '\'] =')
            # content = content.replace(var + ' =', 'topic_names[\'' + name + '\'] =')
            index = content.find('Publisher(', (index1 + 8 + len(var))) # search for the next Publisher defined into the file (if there is one)
            index1 = content.find(')', index)

        index_init = content.find('init_node')
        index_init_comma = content.find(',', index_init)
        index_init_par = content.find(')', index_init)
        if index_init_comma > index_init_par:
            index_init_comma = index_init_par
        node = content[(index_init + 10):index_init_comma].replace('\'', '')
        if index_init == -1:
            #print('The script ' + content_file.name + ' does not contain the ROS node initialization..skipped..')
            with open(file.replace(".py", "_instrumented.py"), 'w') as fout:
                fout.write(content) # update the instrumented file
            return topics_with_types
        else:
            index_par = content.find(')', index_init)
            index_init_aux = (index_init - 1)
            while index_init_aux >= 0 and content[index_init_aux] not in ['\t', '\n', ' ']:
                index_init_aux -= 1
            if content[index_init_aux] == '\n':
                tabs = ''
            else:
                index_init_aux1 = index_init_aux
                while index_init_aux1 >= 0 and content[index_init_aux1] != '\n':
                    index_init_aux1 -= 1
                tabs = content[index_init_aux1:(index_init_aux+1)]
            #pub_monitor = tabs + 'pub_monitor = rospy.Publisher(\'info_topic\', String, queue_size = 10)\n'
            add_info_topic_service = tabs + 'rospy.wait_for_service(\'add_info_topic\')\n'
            add_info_topic_service += tabs + 'try:\n'
            add_info_topic_service += tabs + '\tadd_info_topic = rospy.ServiceProxy(\'add_info_topic\', AddTopicInfo)\t'
            add_info_topic_service += create_add_info_topics(tabs+'\t', node, topics_with_types)
            add_info_topic_service += tabs + 'except rospy.ServiceException, e:'
            add_info_topic_service += tabs + '\tprint "Service call failed: %s"%e'

            srv_change_topic_name = tabs + 'change_name_service = rospy.Service(\'change_topic_name_' + node + '\', ChangeTopicName, change_topic_name)'

            #sub_start_mon_topic = tabs + 'rospy.Subscriber(\'start_mon_topic\', String, start_intercepting_topic_callback)\n'
            #sub_stop_mon_topic = tabs + 'rospy.Subscriber(\'stop_mon_topic\', String, stop_intercepting_topic_callback)'
            content = content[0:(index_par + 1)] + '\n' + add_info_topic_service + srv_change_topic_name + content[(index_par + 1):]

            index_start_line = 0
            while True:
                index_new_line = index_start_line
                while index_new_line < len(content) and  content[index_new_line] != '\n':
                    index_new_line += 1
                if content[index_start_line] not in ['#', '\n']:
                    break
                index_start_line = index_new_line + 1
            if content.find('from std_msgs.msg import String') == -1:
                import_string = '\nfrom std_msgs.msg import String\n'
            else:
                import_string = '\n'
            content = content[0:index_new_line] + import_string + 'from monitor.srv import *\n' + generate_change_name_service_method(topics_with_types) + '\n' + content[index_new_line:]

            # for (var, name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size) in topics_with_types:
            #     content = content.replace(var + '.publish', 'topic_names[\'' + name + '\'].publish')

            with open(file.replace(".py", "_instrumented.py"), 'w') as fout:
                fout.write(content) # update the instrumented file
        return topics_with_types

def create_add_info_topics(tabs, node, topics_with_types):
    add_topics = ''
    for (_, name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size) in topics_with_types:
        add_topics +='''{t}add_info_topic('{no}', '{n}', '{dc}', '{dci}', '{sbl}', '{tcp}', '{la}', '{hd}', '{qs}')'''.format(t = tabs, no = node, n = name, dc = data_class[0], dci = data_class[1], sbl = subscriber_listener, tcp = tcp_nodelay, la = latch, hd = headers, qs = queue_size)
    return add_topics

def parse_publisher_instantiation(content, begin, end):
    # name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None
    name = 'None'
    data_class = 'None'
    subscriber_listener = 'None'
    tcp_nodelay = 'False'
    latch = 'False'
    headers = 'None'
    queue_size = 'None'
    args = [name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size]
    dict = {'name':0, 'data_class':1, 'subscriber_listener':2, 'tcp_nodelay':3, 'latch':4, 'headers': 5, 'queue_size':6}
    round = 0
    begin += 10
    stop = False

    index_eq = begin
    while index_eq >= 0:
        if content[index_eq] == '=':
            break
        index_eq -= 1
    if index_eq != -1:
        index_var = index_eq - 1
        while index_var >= 0:
            if content[index_var] in ['=', ';', '\n', '\t', '\t']:
                break
            index_var -= 1
        if index_var != -1:
            var = content[(index_var + 1):index_eq].replace(' ', '')
        else:
            print('Instrumentation Error: All the publishers must bu assigned to a variable')
            exit(1)
    else:
        print('Instrumentation Error: All the publishers must bu assigned to a variable')
        exit(1)
    while(not stop):
        comma = content.find(',', begin, end)
        if comma == -1:
            comma = end
            stop = True
        assignment = content.find('=', begin, comma)
        if assignment == -1:
            value = content[begin:comma].replace(' ', '')
            args[round] = value
        else:
            key = content[begin:assignment].replace(' ', '')
            value = content[(assignment+1):comma].replace(' ', '')
            if key in dict:
                args[dict[key]] = value
            else:
                print('Argument ' + key + ' in the Publisher instantiation has not been recognized.. ignored..')
        round += 1
        begin = comma + 1
    return var, args[0], args[1], args[2], args[3], args[4], args[5], args[6]
