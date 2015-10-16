#!/usr/bin/env python
import rospy
import math
import numpy
from sensor_msgs.msg import Joy
from rospy.numpy_msg import numpy_msg
from mavros.msg import RCIn
from std_msgs.msg import Float32
from mavros.msg import OverrideRCIn

def callback(msg):
    rcinput = msg.channels
    x = rcinput[0] - 1500
    y = (rcinput[1] - 1500)*-1
    direction=math.atan2(y,x)*180/math.pi
    magnitude=math.sqrt(x**2+y**2)
    if direction<0:
		  direction = direction+360

    rad=math.radians(direction)
    x=math.cos(rad) * magnitude + 1500
    y=math.sin(rad) * magnitude + 1500 

    RCoveride = OverrideRCIn()
    if x>1850:
      x=1850.0
    elif x<1150:
      x=1150.0
    if y>1850:
      y=1850.0
    elif y<1150:
      y=1150.0
    RCoveride.channels = numpy.array( [x,y,0,0,0,0,0,0], dtype=numpy.uint16)
    rospy.loginfo( 'RCoveride= %s', RCoveride)
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    pub.publish(RCoveride)
          
   



def control():
   
    
    rospy.init_node('Rcinput2', anonymous=True)

    rospy.Subscriber("/mavros/rc/in", RCIn, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    control()
