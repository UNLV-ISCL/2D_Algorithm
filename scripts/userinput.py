#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import math
from std_msgs.msg import Float32

def callback(msg):
    axes=msg.axes
    x = msg.axes[0]
    y = msg.axes[1]
    
    if (y>0.1):
    	  direction= math.atan2(axes[1],-axes[0])*180/math.pi
    	  rospy.loginfo( 'direction= %s', direction)
    else:
	  direction=-1
	  rospy.loginfo( 'direction= %s', direction)

    pub = rospy.Publisher('direction', Float32 , queue_size=10)   
    pub.publish(direction)
    


def listener():
   
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('userinput', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

