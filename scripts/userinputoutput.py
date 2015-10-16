#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


def callback(axes):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", axes.axes)
    
def listener():

    # node name is user input
    rospy.init_node('userinput', anonymous=True)
    # subscribes to joy node with Joy data structur and uses callback 
    sub=rospy.Subscriber("joy", Joy, callback)
	#hopfully publshes?
    pub = rospy.Publisher('useroutput', Float32 , queue_size=10)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
	direction = sub.axes.axes[1]
	pub.publish(direction)
	rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

