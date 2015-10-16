#!/usr/bin/env python
import rospy
import math
import numpy
from sensor_msgs.msg import Joy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64
from mavros.msg import OverrideRCIn
from std_msgs.msg import Float32
from numpy import interp

class RCsafeout:
  def __init__(self):
    self.pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=1)
    self.direction = None
    self.magnitude = None
    self.buttons = None
    rospy.Subscriber("det_out", Float32, self.direction_callback)
    rospy.Subscriber('magnitude', Float32, self.magnitude_callback)
    rospy.Subscriber('joy', Joy, self.button_callback)
    

  def direction_callback(self, msg):
    self.direction = msg.data
    self.safeRCout()
  def button_callback(self, msg):
    self.buttons = msg.buttons
    self.rtrig = self.buttons[7]
    self.axes=msg.axes
    self.safeRCout()
  def magnitude_callback(self, msg):
    self.magnitude = msg.data
    self.safeRCout()

  def safeRCout(self):
    #rospy.loginfo('mag=%s, dir=%s', self.magnitude, self.direction)
    if self.magnitude is None or self.direction is None or self.buttons is None:
      return
    RCout = OverrideRCIn()
    rad=math.radians(self.direction)
    r= math.cos(rad) * self.magnitude/2 + 1500
    p= -1*math.sin(rad) * self.magnitude/2 + 1500
    t= self.axes[1]
    y= -1*self.axes[0]
    t=interp(t,[-1,1],[1300,1700])
    y=interp(y,[-1,1],[1300,1700])
    
  

    if r>1850:
      r=1850
    elif r<1150:
      r=1150
    if p>1850:
      p=1850
    elif p<1150:
      p=1150
    
    if self.rtrig ==1:
      RCout.channels = numpy.array([ r , p , t , y , 0 , 0 , 0 , 0 ],dtype=numpy.uint16)
    else:
      RCout.channels = numpy.array([ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ],dtype=numpy.uint16)

    #rospy.loginfo('x=%s, y=%s rad=%s magnitude=%s RCout=%s direction=%s', x, y , rad, self.magnitude, RCout.channels , self.direction )
    
    self.pub.publish(RCout)
    
if __name__ == '__main__':
  rospy.init_node('RCsafeout', anonymous=True)
  try:
    stuff = RCsafeout()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass






