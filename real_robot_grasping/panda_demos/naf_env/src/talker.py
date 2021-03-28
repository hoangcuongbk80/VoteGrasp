#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from rl_task_plugins.msg import DesiredErrorDynamicsMsg

def talker():
	pub = rospy.Publisher('chatter', DesiredErrorDynamicsMsg, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(1)
	test = DesiredErrorDynamicsMsg()
	test.e_ddot_star = [1.1, 2.3]
	while not rospy.is_shutdown():
		#test.e_ddot_star[0] += 0.1
		#test.e_ddot_star[1] += 0.1
		rospy.loginfo(test)
		pub.publish(test)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass 
