#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
#from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats
import numpy 

def callback(msg):
    buttons=msg.buttons
    rtrig = buttons[7]
    axes= msg.axes
    if rtrig == 1:
	 #interpolate xbox axes from -1 to 1 and convert them to RC pwm values.
	  #RCoveride=axes
	  #RCoveride[1]=-1*RCoveride[1]
	 
    	  x = interp(axes,[-1,1],[1000,2000])
	  RCoveride=numpy.array(x, dtype=numpy.int16)
    	  rospy.loginfo( 'RCoveride= %s', RCoveride)
	  
	 
        
    else:
	  RCoveride = numpy.array(62646, dtype=numpy.int16)
	  
	  rospy.loginfo( 'RCoveride= %s', RCoveride)
         
    pub = rospy.Publisher('rc/OverideRCIn', numpy_msg(int), queue_size=10)
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
