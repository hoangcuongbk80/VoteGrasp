#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from rl_task_plugins.msg import DesiredErrorDynamicsMsg

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def listener():
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("chatter", DesiredErrorDynamicsMsg, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()

