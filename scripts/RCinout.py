#!/usr/bin/env python
import rospy
import numpy
from rospy.numpy_msg import numpy_msg
from mavros.msg import OverrideRCIn
from mavros.msg import RCIn

def callback(msg):
    rcinput=msg.channels 
    RCoveride=OverrideRCIn()
    RCoveride.channels = numpy.array(rcinput, dtype=numpy.uint16)
    #RCoveride.channels = numpy.array([0,0,0,0,0,0,0,0], dtype=numpy.uint32)
    rospy.loginfo( 'RCoveride= %s', RCoveride)
    
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    pub.publish(RCoveride)
   
          
   



def control():
   
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('RCinout', anonymous=True)
    rospy.Subscriber("mavros/rc/in", RCIn, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    

if __name__ == '__main__':
    control()
