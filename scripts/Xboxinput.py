#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from rospy.numpy_msg import numpy_msg
from mavros.msg import RCIn
from numpy import interp
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

def callback(msg):
	xboxinput = msg.axes
	x = -1*xboxinput[2]*400 +1500
	y = xboxinput[3]*400 +1500
	direction=math.atan2((y-1500),(x-1500))*180/math.pi
	magnitude=math.sqrt((x-1500)**2+(y-1500)**2)
	if direction<0:
		direction = direction+360
		
	#rospy.loginfo( 'direction= %s x=%s y=%s magnitude=%s', direction,x,y,magnitude)
        pub = rospy.Publisher('direction', Float32 , queue_size=1)
	pub.publish(direction)

        pub = rospy.Publisher('magnitude', Float32 , queue_size=1)
	pub.publish(magnitude)
def control():
    rospy.init_node('Xboxinput', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    control()
