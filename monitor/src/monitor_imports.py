
from rosmon.msg import Person
from std_msgs.msg import Int32
from std_msgs.msg import String
get_topic_info = {
    'chatter' : (String, "std_msgs/String", None, False, False, None, 10),
    'count' : (Int32, "std_msgs/Int32", None, False, False, None, 10),
    'person' : (Person, "rosmon/Person", None, False, False, None, 10)
}