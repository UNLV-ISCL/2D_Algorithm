#!/usr/bin/env python
import rospy
from numpy import interp
from numpy import append
from std_msgs.msg import Int64
#from std_msgs.msg import Float64
#from std_msgs.msg import UInt
from sensor_msgs.msg import Joy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy
from mavros.msg import OverrideRCIn

def callback(msg):
    buttons=msg.buttons
    rtrig = buttons[7]
    axes= msg.axes
    RCoveride = OverrideRCIn()
    if rtrig ==1:
#invert values from 0 and 2 axes values into array x 
      n = numpy.array([-axes[2], -axes[3], axes[1], -axes[0]], dtype=numpy.float64)
	  
 #interpolate xbox axes from -1 to 1 and convert them to RC pwm values
      x = interp(n,[-1,1],[1000,2000])
      x=numpy.array(x, dtype=numpy.int16)
      RCoveride.channels = numpy.append(x, [0,0,0,0])
      RCoveride.channels = numpy.array(RCoveride.channels, dtype=numpy.uint16)
      rospy.loginfo( 'RCoveride= %s', RCoveride)
	 
        
    else:
	  RCoveride.channels = numpy.array( [0,0,0,0,0,0,0,0], dtype=numpy.uint16)
	  rospy.loginfo( 'RCoveride= %s', RCoveride)
    
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    pub.publish(RCoveride)
          
   



def control():
   
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('xboxinput', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    control()
